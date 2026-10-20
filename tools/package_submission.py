#!/usr/bin/env python3
"""Create a fresh source bundle; omit venv, weights, videos and failed attempts.

Original submission_github/ and the previous SLAM ZIP are preserved. The new
directory is COLCON_IGNORE'd to prevent duplicate packages in this workspace.
"""
import argparse
from pathlib import Path
import shutil
import zipfile


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=package.parents[1])
    args = parser.parse_args()
    parent = args.workspace / 'submission_nav_yolo'
    target = parent / package.name
    parent.mkdir(exist_ok=True)
    (parent / 'COLCON_IGNORE').touch()
    target.mkdir(exist_ok=True)
    for name in ('launch', 'config', 'scripts', 'tools', 'maps', 'models', 'rviz', 'test', 'docs'):
        shutil.copytree(package / name, target / name, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for name in ('README.md', 'CMakeLists.txt', 'package.xml', 'LICENSE', '.gitignore', 'requirements-yolo.txt'):
        shutil.copy2(package / name, target / name)
    (target / 'videos').mkdir(exist_ok=True)
    shutil.copy2(package / 'videos/README.md', target / 'videos/README.md')
    artifacts = target / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    for pattern in ('AUTONOMY_VALIDATION.md', 'autonomy_*.txt', 'autonomy_results.json',
                    'autonomy_parallel_results.json', 'map_preview.png', 'map_autonomy.png', 'VALIDATION.md'):
        for path in (package / 'artifacts').glob(pattern):
            shutil.copy2(path, artifacts / path.name)
    demo = package / 'artifacts/autonomy_demo'
    if demo.is_dir():
        shutil.copytree(demo, artifacts / 'autonomy_demo', dirs_exist_ok=True)
    recorded = package / 'artifacts/autonomy_recorded'
    if recorded.is_dir():
        shutil.copytree(recorded, artifacts / 'autonomy_recorded', dirs_exist_ok=True)
    video = package / 'artifacts/autonomy_video'
    if video.is_dir():
        shutil.copytree(video, artifacts / 'autonomy_video', dirs_exist_ok=True)
    clean = package / 'artifacts/autonomy_video_clean'
    if (clean / 'validation.json').is_file():
        shutil.copytree(clean, artifacts / clean.name, dirs_exist_ok=True)
    archive = args.workspace / 'turtlebot4_nav_yolo_submission.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted(target.rglob('*')):
            if path.is_file():
                bundle.write(path, path.relative_to(parent))
    print(target)
    print(archive)


if __name__ == '__main__':
    main()
