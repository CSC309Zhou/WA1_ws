#!/usr/bin/env python3
"""Match PC tcpdump packets to Wrist_Z_L ServoJ and JointState samples.

Only reads PC files. Kernel packet timestamps are software timestamps; this
program makes no claim about NIC hardware time or Pico motor execution.
"""

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import socket
import statistics
import struct

from probe import distribution, first_crossing


RESULT_DIR = Path(__file__).resolve().parent / 'results/Wrist_Z_L'
MAGICS = {
    b'\xd4\xc3\xb2\xa1': ('<', 1000),
    b'\xa1\xb2\xc3\xd4': ('>', 1000),
    b'\x4d\x3c\xb2\xa1': ('<', 1),
    b'\xa1\xb2\x3c\x4d': ('>', 1),
}


def ipv4_tcp(frame):
    """Return IPv4 TCP addresses, ports, sequence, and payload for Ethernet."""
    if len(frame) < 14:
        return None
    kind = struct.unpack_from('!H', frame, 12)[0]
    offset = 14
    while kind in (0x8100, 0x88a8, 0x9100):
        if len(frame) < offset + 4:
            return None
        kind = struct.unpack_from('!H', frame, offset + 2)[0]
        offset += 4
    if kind != 0x0800 or len(frame) < offset + 20:
        return None
    ihl = (frame[offset] & 15) * 4
    if frame[offset] >> 4 != 4 or ihl < 20 or len(frame) < offset + ihl:
        return None
    total = struct.unpack_from('!H', frame, offset + 2)[0]
    flags_fragment = struct.unpack_from('!H', frame, offset + 6)[0]
    if frame[offset + 9] != 6 or flags_fragment & 0x1fff:
        return None
    end = min(len(frame), offset + total)
    tcp = offset + ihl
    if end < tcp + 20:
        return None
    tcp_header_size = (frame[tcp + 12] >> 4) * 4
    if tcp_header_size < 20 or end < tcp + tcp_header_size:
        return None
    payload = frame[tcp + tcp_header_size:end]
    if not payload:
        return None
    return (socket.inet_ntoa(frame[offset + 12:offset + 16]),
            struct.unpack_from('!H', frame, tcp)[0],
            socket.inet_ntoa(frame[offset + 16:offset + 20]),
            struct.unpack_from('!H', frame, tcp + 2)[0],
            struct.unpack_from('!I', frame, tcp + 4)[0], payload)


def read_pcap(path, pc_ip, pico_ip):
    flows = {}
    total_packets = 0
    with open(path, 'rb') as capture:
        header = capture.read(24)
        if len(header) != 24 or header[:4] not in MAGICS:
            raise ValueError('expected a classic pcap file from tcpdump')
        endian, fraction_scale = MAGICS[header[:4]]
        linktype = struct.unpack_from(endian + 'I', header, 20)[0] & 0xffff
        if linktype != 1:
            raise ValueError(f'expected Ethernet link type 1 on eno1, got {linktype}')
        while True:
            packet_header = capture.read(16)
            if not packet_header:
                break
            if len(packet_header) != 16:
                raise ValueError('truncated pcap packet header')
            sec, subsec, captured, original = struct.unpack(endian + 'IIII', packet_header)
            if captured > 10_000_000 or original < captured:
                raise ValueError('invalid packet length in pcap')
            frame = capture.read(captured)
            if len(frame) != captured:
                raise ValueError('truncated pcap packet data')
            total_packets += 1
            decoded = ipv4_tcp(frame)
            if decoded is None:
                continue
            src, sport, dst, dport, seq, payload = decoded
            if not ((src == pc_ip and dst == pico_ip) or
                    (src == pico_ip and dst == pc_ip)):
                continue
            key = (src, sport, dst, dport)
            realtime_ns = sec * 1_000_000_000 + subsec * fraction_scale
            flows.setdefault(key, []).append((seq, realtime_ns, payload))
    return flows, total_packets


class Chunk:
    def __init__(self):
        self.data = bytearray()
        self.spans = []
        self.ends = []

    def append(self, payload, timestamp_ns):
        start = len(self.data)
        self.data.extend(payload)
        end = len(self.data)
        self.spans.append((start, end, timestamp_ns))
        self.ends.append(end)

    def completed_ns(self, byte_index):
        index = bisect_right(self.ends, byte_index)
        return self.spans[index][2]


def reassemble(segments):
    """Deduplicate retransmissions; split TCP streams at capture gaps."""
    if not segments:
        return [], 0
    base = segments[0][0]

    def relative(seq):
        return ((seq - base + 2**31) % 2**32) - 2**31

    ordered = sorted(segments, key=lambda item: (relative(item[0]), item[1]))
    chunks = []
    current = Chunk()
    expected = None
    gaps = 0
    for seq, timestamp_ns, payload in ordered:
        begin = relative(seq)
        if expected is None or begin > expected:
            if current.data:
                chunks.append(current)
                gaps += 1
            current = Chunk()
            expected = begin
        overlap = max(0, expected - begin)
        if overlap >= len(payload):
            continue
        current.append(payload[overlap:], timestamp_ns)
        expected = begin + len(payload)
    if current.data:
        chunks.append(current)
    return chunks, gaps


def parse_joint_state(data, start, expected_stamp):
    """Validate one ROS1 length-prefixed sensor_msgs/JointState frame."""
    if start < 0 or start + 20 > len(data):
        return None
    length = struct.unpack_from('<I', data, start)[0]
    end = start + 4 + length
    if not 200 <= length <= 8192 or end > len(data):
        return None
    secs, nsecs = struct.unpack_from('<II', data, start + 8)
    if secs * 1_000_000_000 + nsecs != expected_stamp:
        return None
    position = start + 16
    frame_id_size = struct.unpack_from('<I', data, position)[0]
    position += 4 + frame_id_size
    if frame_id_size > 256 or position + 4 > end:
        return None
    name_count = struct.unpack_from('<I', data, position)[0]
    position += 4
    if name_count != 19:
        return None
    names = []
    for _ in range(name_count):
        if position + 4 > end:
            return None
        size = struct.unpack_from('<I', data, position)[0]
        position += 4
        if size > 128 or position + size > end:
            return None
        names.append(bytes(data[position:position + size]).decode('utf-8', errors='replace'))
        position += size
    if 'Wrist_Z_L' not in names:
        return None
    for _ in range(3):
        if position + 4 > end:
            return None
        count = struct.unpack_from('<I', data, position)[0]
        position += 4 + count * 8
        if count > 64 or position > end:
            return None
    return end if position == end else None


def command_frames(chunks, wanted_bits):
    prefix = struct.pack('<II', 60, 7)
    records = []
    for chunk in chunks:
        offset = 0
        while True:
            offset = chunk.data.find(prefix, offset)
            if offset < 0:
                break
            if offset + 64 <= len(chunk.data):
                values = struct.unpack_from('<7d', chunk.data, offset + 8)
                value_bits = f'{struct.unpack_from("<Q", chunk.data, offset + 40)[0]:016x}'
                if value_bits in wanted_bits and all(math.isfinite(x) for x in values):
                    records.append({'target_bits': value_bits, 'target_rad': values[4],
                                    'pc_egress_realtime_ns': chunk.completed_ns(offset + 63)})
            offset += 1
    return records


def joint_frames(chunks, wanted_stamps):
    records = []
    for stamp in wanted_stamps:
        needle = struct.pack('<II', stamp // 1_000_000_000, stamp % 1_000_000_000)
        for chunk in chunks:
            offset = 0
            while True:
                offset = chunk.data.find(needle, offset)
                if offset < 0:
                    break
                frame_start = offset - 8
                frame_end = parse_joint_state(chunk.data, frame_start, stamp)
                if frame_end is not None:
                    records.append({'robot_stamp_ns': stamp,
                                    'pc_ingress_realtime_ns': chunk.completed_ns(frame_end - 1)})
                offset += 1
    return records


def analyze(report, metadata, pcap_path, pico_analysis=None):
    if report.get('mode') != 'EXECUTE' or report.get('joint_name') != 'Wrist_Z_L':
        raise ValueError('requires an executed Wrist_Z_L latency report')
    if pico_analysis and pico_analysis.get('report_move_start_pc_monotonic_ns') != \
            report['move_start_pc_monotonic_ns']:
        raise ValueError('Pico analysis belongs to another run; fetch Pico logs for this report')
    samples = metadata['clock_samples']
    if not samples:
        raise ValueError('missing PC realtime↔monotonic calibration samples')
    report_start = report['published'][0]['pc_publish_call_start_ns']
    report_end = report['published'][-1]['pc_monotonic_ns']
    capture_start = min(row['pc_monotonic_ns'] for row in samples)
    capture_end = max(row['pc_monotonic_ns'] for row in samples)
    if report_end < capture_start - 500_000_000 or \
            report_start > capture_end + 500_000_000:
        gap_s = ((capture_start - report_end) if report_end < capture_start else
                 (report_start - capture_end)) / 1e9
        raise ValueError(f'The pcap capture window does not overlap this trajectory '
                         f'report (gap {gap_s:.1f} s). '
                         'Start wire_capture.py, wait for Capture ready, then run the '
                         'Wrist_Z_L --execute test during that capture window.')
    offsets = [row['realtime_minus_monotonic_ns'] for row in samples]
    offset = int(statistics.median(offsets))
    offset_range = max(offsets) - min(offsets)
    if offset_range > 2_000_000:
        raise ValueError('PC clock offset changed by over 2 ms during capture')
    flows, total_packets = read_pcap(pcap_path, metadata['pc_ip'], metadata['pico_ip'])
    wanted_bits = {row['target_bits'] for row in report['published']
                   if row['phase'] == 'move' and 0 < row['scheduled_t_s'] < report['duration_s']}
    wanted_stamps = {row['robot_stamp_ns'] for row in report['feedback_samples']}
    outgoing = []
    incoming = []
    gaps = 0
    for (src, _, dst, _), segments in flows.items():
        chunks, chunk_gaps = reassemble(segments)
        gaps += chunk_gaps
        if src == metadata['pc_ip'] and dst == metadata['pico_ip']:
            outgoing.extend(command_frames(chunks, wanted_bits))
        elif src == metadata['pico_ip'] and dst == metadata['pc_ip']:
            incoming.extend(joint_frames(chunks, wanted_stamps))
    for row in outgoing:
        row['pc_egress_monotonic_ns'] = row['pc_egress_realtime_ns'] - offset
    for row in incoming:
        row['pc_ingress_monotonic_ns'] = row['pc_ingress_realtime_ns'] - offset
    outbound_by_bits = {}
    for row in outgoing:
        outbound_by_bits.setdefault(row['target_bits'], []).append(row)
    inbound_by_stamp = {}
    for row in incoming:
        inbound_by_stamp.setdefault(row['robot_stamp_ns'], []).append(row)
    diagnostics = {row['target_bits']: row for row in report['command_timing']}
    paired_commands = []
    for sent in report['published']:
        if sent['target_bits'] not in wanted_bits:
            continue
        matches = outbound_by_bits.get(sent['target_bits'], [])
        if len(matches) == 1 and sent['target_bits'] in diagnostics:
            paired_commands.append((sent, diagnostics[sent['target_bits']], matches[0]))
    receive_timing = {row['robot_stamp_ns']: row for row in report['joint_receive_timing']}
    paired_states = []
    for state in report['feedback_samples']:
        matches = inbound_by_stamp.get(state['robot_stamp_ns'], [])
        if len(matches) == 1 and state['robot_stamp_ns'] in receive_timing:
            paired_states.append((state, receive_timing[state['robot_stamp_ns']], matches[0]))

    segments = {
        'bridge_ros1_publish_return_to_pc_packet_egress_ms': distribution([
            (wire['pc_egress_monotonic_ns'] - bridge['ros1_publish_return_ns']) / 1e6
            for _, bridge, wire in paired_commands]),
        'pc_ros2_publish_start_to_pc_packet_egress_ms': distribution([
            (wire['pc_egress_monotonic_ns'] - sent['pc_publish_call_start_ns']) / 1e6
            for sent, _, wire in paired_commands]),
        'pc_packet_ingress_to_bridge_ros1_joint_callback_ms': distribution([
            (bridge['bridge_ros1_callback_ns'] - wire['pc_ingress_monotonic_ns']) / 1e6
            for _, bridge, wire in paired_states]),
        'pc_packet_ingress_to_pc_ros2_joint_callback_ms': distribution([
            (state['pc_monotonic_ns'] - wire['pc_ingress_monotonic_ns']) / 1e6
            for state, _, wire in paired_states]),
    }
    wire_crossings = {}
    command_curve = [{'time': wire['pc_egress_monotonic_ns'],
                      'position': sent['target_rad']} for sent, _, wire in paired_commands]
    state_curve = [{'time': wire['pc_ingress_monotonic_ns'],
                    'position': state['position_rad']} for state, _, wire in paired_states]
    for step in range(1, 10):
        level = step / 10
        a = first_crossing(command_curve, 'time', 'position',
                           report['start_rad'], report['delta_rad'], level)
        b = first_crossing(state_curve, 'time', 'position',
                           report['start_rad'], report['delta_rad'], level)
        if a is None or b is None:
            continue
        row = {'pc_packet_egress_to_pc_packet_ingress_apparent_ms': (b - a) / 1e6}
        if pico_analysis:
            pico = pico_analysis['threshold_crossings'].get(str(level))
            if pico:
                row['outside_pico_network_and_timestamp_ms'] = (
                    row['pc_packet_egress_to_pc_packet_ingress_apparent_ms'] -
                    pico['pico_command_log_to_joint_state_stamp_ms'])
        wire_crossings[str(level)] = row
    coverage_warning = None
    if len(paired_commands) < 0.8 * len(wanted_bits) or \
            len(paired_states) < 0.8 * len(wanted_stamps):
        coverage_warning = ('Less than 80% of report samples were found as complete '
                            'TCPROS frames; check capture timing, packet drops and interface.')
    return {
        'pcap_packets': total_packets, 'tcp_flows_with_payload': len(flows),
        'tcp_reassembly_gaps': gaps,
        'pc_clock_offset_range_ms': offset_range / 1e6,
        'pc_clock_bracket_max_ms': max(row['bracket_half_width_ns'] for row in samples) / 1e6,
        'moving_commands_expected': len(wanted_bits),
        'moving_commands_with_unique_egress_packet': len(paired_commands),
        'joint_feedback_expected': len(wanted_stamps),
        'joint_feedback_with_unique_ingress_packet': len(paired_states),
        'coverage_warning': coverage_warning,
        'segments': segments,
        'wire_threshold_crossings': wire_crossings,
        'timestamp_note': ('PC kernel software packet timestamp mapped from CLOCK_REALTIME '
                           'to PC CLOCK_MONOTONIC. Packet egress means observed by the PC '
                           'capture stack, not verified arrival at Pico. Captures with TCP '
                           'gaps or low matching coverage are not conclusive.'),
    }, paired_commands, paired_states


def plot(report, commands, states, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    origin = report['move_start_pc_monotonic_ns']
    fig, ax = plt.subplots(figsize=(11, 5))
    if commands:
        ax.plot([(wire['pc_egress_monotonic_ns'] - origin) / 1e9
                 for _, _, wire in commands],
                [sent['target_rad'] for sent, _, _ in commands],
                label='PC kernel command packet egress')
    if states:
        ax.plot([(wire['pc_ingress_monotonic_ns'] - origin) / 1e9
                 for _, _, wire in states],
                [state['position_rad'] for state, _, _ in states],
                label='PC kernel JointState packet ingress')
    ax.set(title='Wrist_Z_L at PC TCP packet timestamps',
           xlabel='PC monotonic time relative to motion start (s)',
           ylabel='Joint position (rad)')
    ax.grid(True, alpha=0.3)
    if commands or states:
        ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, default=RESULT_DIR / 'latest.json')
    parser.add_argument('--pcap', type=Path, default=RESULT_DIR / 'latest_wire.pcap')
    parser.add_argument('--clock', type=Path, default=RESULT_DIR / 'latest_wire_clock.json')
    parser.add_argument('--pico-analysis', type=Path,
                        default=RESULT_DIR / 'latest_pico_analysis.json',
                        help='Optional same-run Pico log analysis; ignored if missing')
    args = parser.parse_args()
    try:
        report = json.loads(args.report.read_text())
        metadata = json.loads(args.clock.read_text())
        pico = json.loads(args.pico_analysis.read_text()) if args.pico_analysis.is_file() else None
        result, commands, states = analyze(report, metadata, args.pcap, pico)
    except PermissionError as exc:
        parser.error(f'PC capture is not readable by your user: {exc.filename}. '
                     'Change that PC file to your user ownership, then rerun.')
    except (ValueError, FileNotFoundError) as exc:
        parser.error(str(exc))
    stem = args.report.with_suffix('')
    summary_path = Path(str(stem) + '_wire_analysis.json')
    plot_path = Path(str(stem) + '_wire.png')
    summary_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    plot(report, commands, states, plot_path)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f'Wire analysis: {summary_path}\nWire plot: {plot_path}')


if __name__ == '__main__':
    raise SystemExit(main())
