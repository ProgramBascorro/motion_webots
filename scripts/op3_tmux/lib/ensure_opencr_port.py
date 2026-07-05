#!/usr/bin/env python3
"""Provision a stable /dev/ttyOP3 symlink pointing at the live OpenCR board.

Inside the privileged docker container the serial node is a fixed ``--device`` from
container-start time, so after the board re-enumerates (unplug / re-flash, or swapping
between the two OpenCR boards) it lands on a different /dev/ttyUSB* minor and the old
node/symlink goes stale -> "Error opening serial port" / "Error Set port".

This scans /dev/ttyUSB0..N, (re)creating the raw nodes if missing, finds the first one
that actually opens (the live board; dead minors return ENXIO), and repoints the symlink
at it. Stdlib only, idempotent, safe to run on every launch. Degrades quietly
off-hardware. Mirrors op3_action_editor/scripts/executor.py::ensure_opencr_port so the
manager pane no longer depends on the action-editor pane having run first.

Usage: ensure_opencr_port.py [link_path=/dev/ttyOP3] [max_minor=15]
"""
import os
import stat
import sys

TTY_MAJOR = 188


def ensure_opencr_port(link_path='/dev/ttyOP3', max_minor=15):
    # Ensure raw nodes exist so a re-enumerated board is reachable in the container.
    for minor in range(max_minor + 1):
        dev = f'/dev/ttyUSB{minor}'
        if not os.path.exists(dev):
            try:
                os.mknod(dev, 0o666 | stat.S_IFCHR, os.makedev(TTY_MAJOR, minor))
            except OSError:
                pass  # not permitted / already exists -> fall through to open probe

    # First node that actually opens is the live board (dead minors return ENXIO).
    live = None
    for minor in range(max_minor + 1):
        dev = f'/dev/ttyUSB{minor}'
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            continue
        os.close(fd)
        live = dev
        break

    if live is None:
        print(f'ensure_opencr_port: no live /dev/ttyUSB* found; leaving {link_path} as-is',
              file=sys.stderr)
        return False

    try:
        if os.path.realpath(link_path) == live:
            print(f'ensure_opencr_port: {link_path} -> {live} (unchanged)', file=sys.stderr)
            return True
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(live, link_path)
        print(f'ensure_opencr_port: {link_path} -> {live}', file=sys.stderr)
        return True
    except OSError as e:
        print(f'ensure_opencr_port: could not set {link_path}: {e}', file=sys.stderr)
        return False


if __name__ == '__main__':
    link = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyOP3'
    max_minor = int(sys.argv[2]) if len(sys.argv) > 2 else 15
    ensure_opencr_port(link, max_minor)
    # Never hard-fail the launch chain: off-hardware this is a harmless no-op.
    sys.exit(0)
