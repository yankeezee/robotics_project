"""Launch a reproducible development batch in Webots. Predictive mode needs NumPy."""
import argparse
from datetime import datetime, timezone
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from navigation.core import Parameters

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
    parser.add_argument('--timeout', type=float)
    parser.add_argument('--parameters', type=Path, help='JSON file with the shared Parameters values.')
    parser.add_argument('--output-dir', type=Path, help='New directory for this run; existing directories are refused.')
    parser.add_argument('--wall-timeout', type=float, default=600.0, help='Infrastructure watchdog, not simulation timeout.')
    parser.add_argument('--port', type=int, help='Dedicated Webots port for independent concurrent instances.')
    parser.add_argument('--webots')
    parser.add_argument('--show', action='store_true', help='Show Webots while the batch runs.')
    parser.add_argument('--snapshot', action='store_true', help='Export a scene image at each trial start.')
    args = parser.parse_args()
    values = json.loads(args.parameters.read_text(encoding='utf-8')) if args.parameters else {}
    if args.timeout is not None:
        values['timeout'] = args.timeout
    parameters = asdict(Parameters(**values))
    if parameters['timeout'] <= 0 or args.wall_timeout <= 0:
        parser.error('Timeouts must be positive')
    output = (args.output_dir or ROOT / 'results' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = output / 'launch_config.json'
    config.write_text(json.dumps(dict(planner=args.planner, scenarios=args.scenario or ['empty'],
                                     seed=args.seed, parameters=parameters,
                                     quit_on_complete=True, capture_image=args.snapshot,
                                     output_dir=str(output)), indent=2), encoding='utf-8')
    env = os.environ.copy()
    env['NAV_RUN_CONFIG'] = str(config)
    # Webots uses the same Python interpreter as this launcher.
    env['PATH'] = str(Path(sys.executable).parent) + os.pathsep + env.get('PATH', '')
    command = [find_webots(args.webots), '--batch', '--mode=fast', '--stdout', '--stderr']
    if args.port:
        command.append(f'--port={args.port}')
    if not args.show:
        command.append('--minimize')
    if not args.show and not args.snapshot:
        command.append('--no-rendering')
    command.append(str(ROOT / 'worlds/walkway.wbt'))
    print(f'Running {args.planner} {args.scenario or ["empty"]}; results: {output}', flush=True)
    with (output / 'webots.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, env=env, cwd=ROOT, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=os.name != 'nt')
        try:
            code = process.wait(timeout=args.wall_timeout)
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            if os.name == 'nt':
                subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True)
            else:
                import signal
                os.killpg(process.pid, signal.SIGTERM)
            process.wait()
            raise
    print((output/'webots.log').read_text(encoding='utf-8', errors='replace'), end='', flush=True)
    if code:
        return code
    summary_path = output / 'summary.json'
    expected = args.scenario or ['empty']
    if not summary_path.exists() or len(json.loads(summary_path.read_text())) != len(expected):
        print('ERROR: Webots ended without completing the requested batch. Inspect webots.log.', file=sys.stderr)
        return 1
    print('RUN_COMPLETE '+str(output), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
