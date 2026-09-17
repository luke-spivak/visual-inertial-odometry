#!/usr/bin/env python3
"""Fetch completed Pi recordings and render them locally. No onboard encoding."""
import argparse
import json
from pathlib import Path
import re
import subprocess
from render_tracking import render


def fetch(host, remote_dir, destination):
    if not re.fullmatch(r'[A-Za-z0-9_.@-]+', host) or host.startswith('-'):
        raise ValueError('Invalid SSH host')
    # Restrict remote paths to avoid remote-shell interpretation. ~ is supported.
    if not re.fullmatch(r'[A-Za-z0-9_./~-]+', remote_dir) or remote_dir.startswith('-'):
        raise ValueError('Remote directory must be a simple path without spaces')
    destination.mkdir(parents=True, exist_ok=True)
    subprocess.run(['rsync', '-a', '--include=*.complete.json', '--exclude=*',
                    f'{host}:{remote_dir.rstrip("/")}/', str(destination) + '/'], check=True)
    prefixes = []
    for marker in sorted(destination.glob('*.complete.json')):
        name = marker.name.removesuffix('.complete.json')
        if not re.fullmatch(r'run-[A-Za-z0-9_-]+', name):
            raise ValueError('Unexpected recording name')
        subprocess.run(['rsync', '-a', '--exclude=*.tracking*',
                        f'{host}:{remote_dir.rstrip("/")}/{name}.*', str(destination) + '/'], check=True)
        if json.loads(marker.read_text()).get('vio_exit') != 0:
            raise ValueError(f'Unsuccessful recording: {marker}')
        prefixes.append(destination / name)
    return prefixes


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('destination', type=Path)
    p.add_argument('--host', default='viopi'); p.add_argument('--remote-dir', default='~/vio')
    p.add_argument('--fps', type=float, default=20)
    a = p.parse_args()
    failures = []
    for prefix in fetch(a.host, a.remote_dir, a.destination):
        try:
            manifest = Path(str(prefix) + '.recording.json')
            dims = json.loads(manifest.read_text()) if manifest.exists() else {}
            render(prefix, dims.get('width', 1280), dims.get('height', 800), a.fps)
        except Exception as e:
            failures.append(str(prefix)); print(f'FAILED {prefix}: {e}')
    if failures: raise SystemExit(1)
