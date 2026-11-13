"""Check release assets before they are published.

    verify.py <dist> <version> [forbidden string ...]

Run by packaging/release/build.sh in its Arch root (needs bsdtar and readelf). Unpacks every asset next to
<dist> and stops at the first failed check. Forbidden strings are host paths that must not leak into assets.
"""

import hashlib
import re
import subprocess
import sys
import tempfile
from pathlib import Path, PurePosixPath

GLIBC_MAX = (2, 35)  # the universal build's floor, stated in the README


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def one(dist, pattern):
    found = sorted(dist.glob(pattern))
    assert len(found) == 1, (pattern, [p.name for p in found])
    return found[0]


def unpack(archive, dest):
    """Extract an asset after checking that root owns every member and no path leaves dest."""
    for line in subprocess.check_output(['bsdtar', '--numeric-owner', '-tvf', archive], text=True).splitlines():
        assert line.split()[2:4] in (['0', '0'], ['root', 'root']), (archive.name, line)
    for name in subprocess.check_output(['bsdtar', '-tf', archive], text=True).splitlines():
        path = PurePosixPath(name)
        assert not path.is_absolute() and '..' not in path.parts, (archive.name, name)
    dest.mkdir()
    subprocess.run(['bsdtar', '-xf', archive, '-C', dest], check=True)
    for path in dest.rglob('*'):
        if path.is_symlink():
            assert path.exists() and path.resolve().is_relative_to(dest.resolve()), path
    return dest


def glibc_needed(path):
    found = re.findall(r'GLIBC_(\d+(?:\.\d+)+)', subprocess.check_output(['readelf', '--version-info', path], text=True))
    return max(tuple(map(int, v.split('.'))) for v in found)


def main():
    dist, version, *forbidden = sys.argv[1:]
    dist = Path(dist)
    binver = version[1:].replace('-', '.')
    basalt = one(dist, 'basalt-monado-rifts-[0-9]*-x86_64.pkg.tar.zst')
    git = one(dist, 'rift-s-linux-git-[0-9]*-x86_64.pkg.tar.zst')
    binpkg = dist / f'rift-s-linux-bin-{binver}-1-x86_64.pkg.tar.zst'
    payload = dist / f'rift-s-linux-{version}-x86_64.tar.zst'
    universal = dist / f'rift-s-linux-{version}-x86_64.tar.xz'
    stable = {dist / 'rift-s-linux-bin-x86_64.pkg.tar.zst': binpkg, dist / 'rift-s-linux-x86_64.tar.xz': universal}
    found = {p for p in dist.iterdir() if p.name != 'SHA256SUMS'}
    assert found == {basalt, git, binpkg, payload, universal, *stable}, sorted(p.name for p in found)
    for copy, original in stable.items():
        assert sha(copy) == sha(original), copy.name

    with tempfile.TemporaryDirectory(dir=dist.parent) as tmp:
        roots = {a: unpack(a, Path(tmp) / a.name) for a in (basalt, git, binpkg, payload, universal)}
        g, b, p, ba = (roots[a] / 'usr' for a in (git, binpkg, payload, basalt))
        u = roots[universal] / f'rift-s-linux-{version}-x86_64' / 'files'

        # The payload and -bin are the -git build plus Basalt.
        for rel in ['lib/steamvr-monado/bin/linux64/driver_monado.so', 'bin/rifts-desktop', 'bin/rifts-settings',
                    'bin/rifts-setup', 'lib/udev/rules.d/70-monado-rifts.rules']:
            assert sha(g / rel) == sha(b / rel) == sha(p / rel), rel
        for root in (b, p):
            assert sha(root / 'lib/libbasalt.so.2') == sha(ba / 'lib/libbasalt.so.2')

        assert (u / 'version').read_text().strip() == version
        assert not (u / 'share/openxr').exists(), 'the universal build registers no system OpenXR runtime'
        for rel in ['share/steamvr-monado/bin/linux64/driver_monado.so', 'lib/libbasalt.so.2', 'bin/rifts-settings',
                    'bin/rifts-desktop']:
            assert glibc_needed(u / rel) <= GLIBC_MAX, (rel, glibc_needed(u / rel))

        for root in roots.values():
            for path in root.rglob('*'):
                if path.is_file() and not path.is_symlink():
                    data = path.read_bytes()
                    for text in forbidden:
                        assert text.encode() not in data, (path, text)
    print(f'{version}: {len(found)} assets checked')


main()
