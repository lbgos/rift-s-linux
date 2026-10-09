#!/usr/bin/env python3
# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
"""Install and enable the optional passthrough overlay in the user's systemd session."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys


def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', type=Path, default=Path(sys.executable), help='Python interpreter with openvr installed')
    args = parser.parse_args()
    python = args.python.expanduser().absolute()
    subprocess.run([str(python), '-c', 'import openvr'], check=True)
    target = Path.home() / '.local/lib/rifts-passthrough'
    target.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parent
    for name in ('rifts-passthrough-overlay.py', 'rifts_passthrough.py'):
        shutil.copy2(source / name, target / name)
    units = Path.home() / '.config/systemd/user'
    units.mkdir(parents=True, exist_ok=True)
    unit = f'''[Unit]
Description=Rift S passthrough overlay

[Service]
ExecStart={quote(python)} -u {quote(target / 'rifts-passthrough-overlay.py')} --start-hidden
Restart=on-failure
RestartSec=5
NoNewPrivileges=true

[Install]
WantedBy=default.target
'''
    (units / 'rifts-passthrough-overlay.service').write_text(unit)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', '--user', 'enable', '--now', 'rifts-passthrough-overlay.service'], check=True)
    print('Overlay installed and enabled; starts hidden. Native Room View is preferred.')


if __name__ == '__main__':
    main()
