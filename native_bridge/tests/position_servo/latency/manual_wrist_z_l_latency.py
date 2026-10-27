#!/usr/bin/env python3
"""Wrist_Z_L ServoJ latency probe. Preview unless --execute is supplied."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from servoj_single_joint import main  # noqa: E402


if __name__ == '__main__':
    raise SystemExit(main('Wrist_Z_L', latency_probe=True))
