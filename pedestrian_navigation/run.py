"""Launch a reproducible development batch in Webots. Predictive mode needs NumPy."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from navigation.scenarios import FAMILIES

ROOT = Path(__file__).resolve().parent


def find_webots(explicit=None):
    candidates = [explicit, shutil.which('webots'),
                  '/Applications/Webots.app/Contents/MacOS/webots',
                  '/usr/local/webots/webots',
                  str(Path(os.environ.get('WEBOTS_HOME', 'C:/Program Files/Webots')) / 'msys64/ucrt64/bin/webots.exe'),
                  'C:/Program Files/Webots/msys64/mingw64/bin/webots.exe']
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(Path(candidate).resolve())
    raise FileNotFoundError('Webots executable not found. Use --webots with its full path.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=FAMILIES, action='append', help='Repeat to run several development cases.')
    parser.add_argument('--planner', choices=('reactive', 'predictive'), default='reactive')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--webots')
    parser.add_argument('--show', action='store_true', help='Show Webots while the batch runs.')
    parser.add_argument('--snapshot', action='store_true', help='Export a scene image at each trial start.')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    output = ROOT / 'results' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output.mkdir(parents=True)
    config = output / 'launch_config.json'
    config.write_text(json.dumps(dict(planner=args.planner, scenarios=args.scenario or ['empty'],
                                     seed=args.seed, parameters={'timeout': args.timeout},
                                     quit_on_complete=True, capture_image=args.snapshot,
                                     output_dir=str(output)), indent=2), encoding='utf-8')
    env = os.environ.copy()
    env['NAV_RUN_CONFIG'] = str(config)
    # Webots uses the same Python interpreter as this launcher.
    env['PATH'] = str(Path(sys.executable).parent) + os.pathsep + env.get('PATH', '')
    command = [find_webots(args.webots), '--batch', '--mode=fast', '--stdout', '--stderr']
    if not args.show:
        command.append('--minimize')
    if not args.show and not args.snapshot:
        command.append('--no-rendering')
    command.append(str(ROOT / 'worlds/walkway.wbt'))
    print(f'Running {args.planner} {args.scenario or ["empty"]}; results: {output}', flush=True)
    with (output / 'webots.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, env=env, cwd=ROOT, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
        try:
            for line in process.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
            code = process.wait()
        except KeyboardInterrupt:
            process.terminate()
            process.wait()
            raise
    if code:
        return code
    summary_path = output / 'summary.json'
    expected = args.scenario or ['empty']
    if not summary_path.exists() or len(json.loads(summary_path.read_text())) != len(expected):
        print('ERROR: Webots ended without completing the requested batch. Inspect webots.log.', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
