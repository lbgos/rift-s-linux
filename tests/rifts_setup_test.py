# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
"""Checks how rifts-setup edits SteamVR's launch option in Steam's localconfig.vdf
and how triage and update map an installed package to its source and release asset.

Usage: rifts_setup_test.py <configured rifts-setup script>
"""
import importlib.machinery
import importlib.util
import sys
import unittest
import tempfile
from pathlib import Path

loader = importlib.machinery.SourceFileLoader('rifts_setup', sys.argv.pop(1))
setup = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
loader.exec_module(setup)

OURS = '/home/u/.local/share/Steam/steamapps/common/SteamVR/bin/vrmonitor.sh %command%'


def config(app_lines):
    body = ''.join('\t\t\t\t\t\t' + line + '\n' for line in app_lines)
    return ('"UserLocalConfigStore"\n{\n\t"Software"\n\t{\n\t\t"Valve"\n\t\t{\n\t\t\t"Steam"\n\t\t\t{\n'
            '\t\t\t\t"apps"\n\t\t\t\t{\n\t\t\t\t\t"250820"\n\t\t\t\t\t{\n' + body +
            '\t\t\t\t\t}\n\t\t\t\t}\n\t\t\t}\n\t\t}\n\t}\n}\n')


def launch_options(text):
    block, _ = setup.vdf_block(setup.vdf_tokens(text), close=len(text))
    for key in setup.LAUNCH_PATH:
        block = next(v for k, v in block['items'] if k.lower() == key.lower())
    return next((v[3] for k, v in block['items'] if k == 'LaunchOptions'), None)


class LaunchOptions(unittest.TestCase):
    def test_adds_line_and_keeps_the_rest(self):
        before = config(['"LastPlayed"\t\t"1"'])
        after = setup.with_launch_options(before, OURS)
        self.assertEqual(after, config(['"LastPlayed"\t\t"1"', '"LaunchOptions"\t\t"' + OURS + '"']))

    def test_fills_empty_option(self):
        after = setup.with_launch_options(config(['"LaunchOptions"\t\t""']), OURS)
        self.assertEqual(after, config(['"LaunchOptions"\t\t"' + OURS + '"']))

    def test_keeps_custom_option(self):
        self.assertIsNone(setup.with_launch_options(config(['"LaunchOptions"\t\t"gamemoderun %command%"']), OURS))

    def test_updates_our_option_after_steamvr_moves(self):
        old = config(['"LaunchOptions"\t\t"/old/SteamVR/bin/vrmonitor.sh %command%"'])
        self.assertEqual(launch_options(setup.with_launch_options(old, OURS)), OURS)

    def test_uninstall_removes_only_our_line(self):
        installed = config(['"LaunchOptions"\t\t"' + OURS + '"', '"LastPlayed"\t\t"1"'])
        self.assertEqual(setup.with_launch_options(installed, None), config(['"LastPlayed"\t\t"1"']))
        custom = config(['"LaunchOptions"\t\t"gamemoderun %command%"'])
        self.assertIsNone(setup.with_launch_options(custom, None))

    def test_creates_missing_app_block(self):
        before = config([]).replace('\t\t\t\t\t"250820"\n\t\t\t\t\t{\n\t\t\t\t\t}\n', '')
        after = setup.with_launch_options(before, OURS)
        self.assertEqual(launch_options(after), OURS)
        self.assertEqual(after, config(['"LaunchOptions"\t\t"' + OURS + '"']))

    def test_quoted_values_round_trip(self):
        spaced = '"/my games/SteamVR/bin/vrmonitor.sh" %command%'
        self.assertEqual(launch_options(setup.with_launch_options(config([]), spaced)), spaced)


class ReleaseRef(unittest.TestCase):
    def test_package_versions_map_to_source(self):
        self.assertEqual(setup.release_ref('rift-s-linux-bin', '31-1'), 'r31')
        self.assertEqual(setup.release_ref('rift-s-linux-git', '25.1.0.r8702.gd82b8cbda-1'), 'd82b8cbda')
        self.assertIsNone(setup.release_ref('other', '1.0-1'))


class ReleaseAsset(unittest.TestCase):
    NAMES = ['rift-s-linux-bin-32-1-x86_64.pkg.tar.zst', 'rift-s-linux-r32-x86_64.tar.xz',
             'rift-s-linux-r32-x86_64.tar.zst', 'SHA256SUMS']

    def test_each_install_type_gets_its_asset(self):
        self.assertEqual(setup.release_asset('universal build', 'r32', self.NAMES), 'rift-s-linux-r32-x86_64.tar.xz')
        self.assertEqual(setup.release_asset('Arch package rift-s-linux-bin', 'r32', self.NAMES), self.NAMES[0])
        self.assertIsNone(setup.release_asset('Arch package rift-s-linux-git', 'r32', self.NAMES))


class SafeMode(unittest.TestCase):
    def test_warns_without_editing_settings_or_exposing_other_values(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'config').mkdir()
            path = root / 'config/steamvr.vrsettings'
            original = '{"driver_monado":{"blocked_by_safe_mode":true},"unrelated":"private"}'
            path.write_text(original)
            status = list(setup.steamvr_driver_status([root]))
            self.assertEqual(len(status), 1)
            self.assertIn('blocked_by_safe_mode=true', status[0])
            self.assertIn('Manage Add-ons', status[0])
            self.assertNotIn('private', status[0])
            self.assertEqual(path.read_text(), original)

    def test_missing_unblocked_and_malformed_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(list(setup.steamvr_driver_status([root])), [])
            (root / 'config').mkdir()
            path = root / 'config/steamvr.vrsettings'
            path.write_text('{"driver_monado":{"blocked_by_safe_mode":false}}')
            self.assertIn('not blocked', next(setup.steamvr_driver_status([root])))
            for text in ['{', '[]', '{"driver_monado":null}']:
                path.write_text(text)
                self.assertIn('unable to inspect', next(setup.steamvr_driver_status([root])))


if __name__ == '__main__':
    unittest.main()
