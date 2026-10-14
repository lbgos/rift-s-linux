#!/usr/bin/env python3
"""Collect locked Settings dependencies' declarations and bundled notices."""
import json
from pathlib import Path
import sys


fallbacks = json.loads(Path(sys.argv[3]).read_text())


def emit(name, version, license_id, roots, license_file=None, source=None):
    print(f"\n{name} {version}\nLicense: {license_id or 'See bundled notice'}\n")
    if source:
        print(f"Source: {source}\n")
    notices = set()
    for root in roots:
        root = Path(root)
        for item in root.rglob('*'):
            if item.name.lower().startswith(('license', 'licence', 'copying', 'copyright', 'notice')):
                if item.is_file():
                    notices.add(item)
                elif item.is_dir():
                    notices.update(p for p in item.rglob('*') if p.is_file())
        if license_file:
            notices.add(root / license_file)
    for notice in sorted(notices):
        print(f"--- {notice.relative_to(next(Path(root) for root in roots if notice.is_relative_to(root)))} ---")
        print(notice.read_text(encoding='utf-8', errors='replace'))
    if not notices:
        fallback = fallbacks.get(f'{name}@{version}')
        if not fallback:
            raise SystemExit(f'Missing license notice for {name} {version}')
        print(f"Notice source: {fallback['source']}\n")
        print(fallback['notice'])


rust = json.loads(Path(sys.argv[1]).read_text())
resolved = {node['id'] for node in rust['resolve']['nodes']}
for package in sorted(rust['packages'], key=lambda p: (p['name'], p['version'])):
    # The application's own MIT notice is installed separately.
    if package['id'] in resolved and package['id'] not in rust['workspace_members']:
        emit(package['name'], package['version'], package['license'],
             [Path(package['manifest_path']).parent], package.get('license_file'),
             f"https://crates.io/api/v1/crates/{package['name']}/{package['version']}/download")

javascript = json.loads(Path(sys.argv[2]).read_text())
for packages in javascript.values():
    for package in sorted(packages, key=lambda p: p['name']):
        emit(package['name'], ', '.join(package['versions']), package['license'], package['paths'])
