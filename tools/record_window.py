#!/usr/bin/env python3
"""Record ONE explicitly selected visible X11 application window to WebM.

Read-only: no UI input, navigation goals, robot commands or synthetic frames.
Requires GStreamer ximagesrc, videoconvert, vp8enc and webmmux. Find the RViz
window ID with `xwininfo -root -tree`. Ctrl+C finalizes the WebM container.
"""
import argparse
from pathlib import Path
import signal
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=lambda s: int(s, 0), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    info = subprocess.check_output(['xwininfo', '-id', hex(args.window)], text=True)
    if 'IsViewable' not in info:
        raise RuntimeError('Target application window is not visible')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen([
        'gst-launch-1.0', '-q', '-e', 'ximagesrc', f'xid={args.window}',
        'use-damage=false', 'show-pointer=false', '!', 'video/x-raw,framerate=8/1',
        '!', 'videoconvert', '!', 'vp8enc', 'deadline=1', 'cpu-used=8',
        'target-bitrate=1400000', '!', 'webmmux', '!', 'filesink', f'location={args.output}'],
        start_new_session=True)
    try:
        process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
    if process.returncode:
        raise RuntimeError(f'Recorder exit code: {process.returncode}')
    print(args.output)


if __name__ == '__main__':
    main()
