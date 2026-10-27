#!/usr/bin/env python3
"""Capture only PC↔Pico TCP packets on the PC wired interface.

This program never publishes ROS messages or starts the bridge. Run the
existing manual ServoJ test yourself in another terminal while it captures.
"""

import argparse
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time


DEFAULT_PREFIX = (Path(__file__).resolve().parent /
                  'results/Wrist_Z_L/latest_wire')


def clock_sample():
    before = time.monotonic_ns()
    realtime = time.time_ns()
    after = time.monotonic_ns()
    return {'pc_monotonic_ns': (before + after) // 2,
            'pc_realtime_ns': realtime,
            'realtime_minus_monotonic_ns': realtime - (before + after) // 2,
            'bracket_half_width_ns': (after - before) // 2}


def stderr_reader(stream, output):
    for raw in iter(stream.readline, b''):
        output.put(raw.decode(errors='replace').rstrip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.0,
                        help='Capture duration; start the manual test during this window')
    parser.add_argument('--interface', default='eno1',
                        help='PC wired interface; default eno1')
    parser.add_argument('--pc-ip', default='192.168.217.10')
    parser.add_argument('--pico-ip', default='192.168.217.66')
    parser.add_argument('--output-prefix', type=Path, default=DEFAULT_PREFIX,
                        help='PC output prefix; saves .pcap and _clock.json')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 180:
        parser.error('--seconds must be between 2 and 180')
    if not (Path('/sys/class/net') / args.interface).exists():
        parser.error(f'PC network interface {args.interface} does not exist')
    if not shutil.which('tcpdump'):
        parser.error('tcpdump is not installed on this PC')
    if os.geteuid() == 0:
        elevation = []
    else:
        ready = subprocess.run(['sudo', '-n', '-v'], capture_output=True, text=True)
        if ready.returncode:
            parser.error('PC packet capture needs sudo. Run sudo -v in this same terminal, '
                         'then rerun this script.')
        elevation = ['sudo', '-n']

    prefix = args.output_prefix.expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    pcap_path = Path(str(prefix) + '.pcap')
    clock_path = Path(str(prefix) + '_clock.json')
    with tempfile.NamedTemporaryFile(prefix='.wa1-wire-', suffix='.pcap',
                                     dir=prefix.parent, delete=False) as temporary:
        temp_path = Path(temporary.name)
    filter_expression = f'tcp and host {args.pc_ip} and host {args.pico_ip}'
    command = elevation + [
        'timeout', '--signal=INT', str(args.seconds), 'tcpdump',
        '-i', args.interface, '-j', 'host', '-nn', '-s', '0', '-U', '-Z', 'root',
        '--time-stamp-precision=nano', '-w', str(temp_path), filter_expression,
    ]
    status_lines = queue.Queue()
    samples = []
    process = None
    try:
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE)
        error_thread = threading.Thread(target=stderr_reader,
                                        args=(process.stderr, status_lines), daemon=True)
        error_thread.start()
        startup_deadline = time.monotonic() + 7.0
        ready = False
        while time.monotonic() < startup_deadline:
            if process.poll() is not None:
                break
            try:
                line = status_lines.get(timeout=0.1)
            except queue.Empty:
                continue
            if 'listening on ' in line:
                ready = True
                break
        if not ready:
            remaining = list(status_lines.queue)
            raise RuntimeError('tcpdump did not become ready: ' + '; '.join(remaining))
        print(f'Capture ready on PC {args.interface}. Run the manual ServoJ test '
              f'in another terminal now. Filter: {filter_expression}', flush=True)
        while process.poll() is None:
            samples.append(clock_sample())
            time.sleep(0.1)
        error_thread.join(timeout=2)
        if process.returncode not in (0, 124):
            raise RuntimeError(f'tcpdump exited {process.returncode}: '
                               + '; '.join(list(status_lines.queue)))
        if temp_path.stat().st_size <= 24:
            raise RuntimeError('pcap contains no captured packets')
        # tcpdump may drop privileges and chown its savefile to nobody even
        # when -Z root was requested. Restore the invoking PC user's ownership
        # before publishing the file as latest_wire.pcap.
        owner_uid = int(os.environ.get('SUDO_UID', os.getuid()))
        owner_gid = int(os.environ.get('SUDO_GID', os.getgid()))
        if os.geteuid() == 0:
            os.chown(temp_path, owner_uid, owner_gid)
        else:
            ownership = subprocess.run(
                ['sudo', '-n', 'chown', f'{owner_uid}:{owner_gid}', str(temp_path)],
                capture_output=True, text=True)
            if ownership.returncode:
                raise RuntimeError('tcpdump finished but PC pcap ownership could not be '
                                   f'restored: {ownership.stderr.strip()}')
        os.replace(temp_path, pcap_path)
        lines = list(status_lines.queue)
        offsets = [item['realtime_minus_monotonic_ns'] for item in samples]
        metadata = {
            'pcap_path': str(pcap_path), 'pcap_bytes': pcap_path.stat().st_size,
            'interface': args.interface, 'pc_ip': args.pc_ip, 'pico_ip': args.pico_ip,
            'filter': filter_expression, 'duration_requested_s': args.seconds,
            'clock_samples': samples,
            'offset_range_ns': max(offsets) - min(offsets) if offsets else None,
            'tcpdump_summary': lines,
            'timestamp_note': ('tcpdump host kernel timestamps use PC CLOCK_REALTIME; '
                               'paired samples map them to PC CLOCK_MONOTONIC. '
                               'These are software packet timestamps, not NIC hardware timestamps.'),
        }
        clock_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        print(f'PC pcap: {pcap_path}\nPC clock samples: {clock_path}')
        if metadata['offset_range_ns'] is not None:
            print(f'Clock offset range during capture: '
                  f'{metadata["offset_range_ns"] / 1e6:.3f} ms')
        for line in lines:
            if 'packet' in line:
                print(line)
    except KeyboardInterrupt:
        print('Capture interrupted; previous complete pcap was preserved.', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'Capture failed: {exc}; partial PC file: {temp_path}', file=sys.stderr)
        return 1
    finally:
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                process.kill()
                process.wait()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
