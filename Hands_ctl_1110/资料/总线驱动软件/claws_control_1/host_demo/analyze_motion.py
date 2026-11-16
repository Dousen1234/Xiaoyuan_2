"""Offline Hall-position comparison for a single-motion JSONL; never opens CAN."""
import argparse
import csv
import json
from pathlib import Path


def analyze(rows):
    commands = [r for r in rows if r.get('direction') == 'tx' and r.get('id') == 0x180
                and len(bytes.fromhex(r['data'])) == 8
                and bytes.fromhex(r['data'])[2] in (1, 6, 7, 8)]
    if len(commands) != 1:
        raise ValueError('Expected exactly one motion command per log')
    start = commands[0]['t_s']
    samples = [r for r in rows if r.get('direction') == 'rx' and r.get('id') == 0x181
               and r['t_s'] >= start and 'angles_deg' in r.get('feedback', {})]
    if len(samples) < 2:
        raise ValueError('Insufficient position feedback')
    # Only accept a completed move; faulted/timed-out traces are not arrival data.
    if samples[-1]['feedback']['state'] != 2:
        raise ValueError('Last position is not HOLD; inspect the fault/timeout log')
    initial = samples[0]['feedback']['angles_deg']
    final = samples[-1]['feedback']['angles_deg']
    curve = []
    for k, row in enumerate(samples):
        t = row['t_s']
        angles = row['feedback']['angles_deg']
        # Difference over at least 100 ms reduces single-Hall-step quantization.
        previous = next((r for r in reversed(samples[:k]) if t-r['t_s'] >= .1), None)
        speeds = [(angles[m]-previous['feedback']['angles_deg'][m]) /
                  (t-previous['t_s']) if previous else None for m in range(3)]
        progress = [angles[m]-initial[m] for m in range(3)]
        curve.append([t-start, *angles, *speeds, max(progress)-min(progress)])
    metrics = []
    for m in range(3):
        changed = [r for a, r in zip(samples, samples[1:])
                   if a['feedback']['angles_deg'][m] != r['feedback']['angles_deg'][m]]
        duration = changed[-1]['t_s']-start if changed else None
        metrics.append({'motor': m+1, 'start_deg': initial[m], 'final_deg': final[m],
                        'last_change_s': duration,
                        'mean_deg_s': (final[m]-initial[m])/duration if duration else None})
    return metrics, curve


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    with args.log.open(encoding='utf-8-sig') as stream:
        metrics, curve = analyze([json.loads(line) for line in stream if line.strip()])
    output = args.log.with_suffix('.motion.csv')
    with output.open('x', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_s', 'f1_deg', 'f2_deg', 'f3_deg', 'f1_deg_s_100ms',
                         'f2_deg_s_100ms', 'f3_deg_s_100ms', 'progress_spread_deg'])
        writer.writerows(curve)
    print(json.dumps(metrics, indent=2))
    print('Hall estimates only; last_change_s is last observed motion, not exact target arrival.')
    print('Mean speed includes startup; progress spread is comparable for equal signed travel.')
    print(output)


if __name__ == '__main__':
    main()
