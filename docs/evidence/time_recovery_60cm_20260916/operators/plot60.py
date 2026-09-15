"""Plot observed recovery and accepted camera anchors for this campaign."""
from pathlib import Path
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path('/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_60cm_20260916')
NAMES = sorted(p.name.removesuffix('_collection_summary.json') for p in OUT.glob('*-d60-*_collection_summary.json') if json.loads(p.read_text())['accepted'] > 0 and (OUT/(p.name.removesuffix('_collection_summary.json')+'_anchor_states.json')).exists())
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> None:
    fig, axes = plt.subplots(len(NAMES), 1, figsize=(12, 3.5*len(NAMES)), sharex=True, constrained_layout=True, squeeze=False)
    axes = axes[:, 0]
    colors = dict(P00='#1264ab', S00='#bb4b00', P02='#16825d', P03='#963caa', P04='#756500', P05='#5d596e')
    provenance = {}
    for ax, name in zip(axes, NAMES):
        report_path = OUT / (name + '_collection_summary.json')
        states_path = OUT / (name + '_anchor_states.json')
        controls_path = RAW / name / 'control.jsonl'
        report = read(report_path)
        states = read(states_path) if states_path.exists() else []
        controls = [json.loads(s) for s in controls_path.read_text().splitlines()]
        sign = -1 if '-right-' in name else 1
        low, high = sorted([sign * .55, sign * .65])
        ax.axhspan(low, high, color='#dce8f0', alpha=.85, label='55-65 cm target band')
        ax.axhline(0, color='#555555', linewidth=1)
        for event in report['events']:
            if event['release_ns'] is None:
                continue
            release = event['release_ns']
            site = event['site_id']
            rows = [r for r in controls if r['phase'] == 'recovery'
                    and r['large_recovery']['state']['event_id'] == event['event_id']]
            points = [r for r in states if r['event_id'] == event['event_id']]
            ax.plot([(r['publication']['sim_ns'] - release) / 1e9 for r in rows],
                    [r['large_recovery']['lateral_error_m'] for r in rows],
                    color=colors[site], linewidth=1.3, alpha=.7 if points else .4,
                    linestyle='-' if points else '--')
            ax.scatter([(r['observation_ns'] - release) / 1e9 for r in points],
                       [r['lateral_m'] for r in points], color=colors[site], s=10,
                       label=(f"{site}: {len(points)} anchors ({sum(r['target_band'] for r in points)} in band)"
                              if points else f"{site}: excluded recovery"))
        short = name.removeprefix('codex-time-recovery-60cm-')
        completed = sum(e['completed'] for e in report['events'])
        ax.set_title(f"{short} | {completed}/{report['event_cap']} recovery events | {report['result_status']}", loc='left')
        ax.set_ylabel('Lateral error [m]')
        ax.set_ylim(-.75, .75)
        ax.set_xlim(0, 10.1)
        ax.grid(alpha=.2)
        ax.legend(loc='upper right' if sign < 0 else 'lower right', ncol=2, fontsize=8)
        provenance[name] = dict(control_sha256=sha(controls_path), summary_sha256=sha(report_path),
                                anchor_states_sha256=sha(states_path) if states_path.exists() else None)
    axes[-1].set_xlabel('Simulation seconds after actual nominal PP command handover')
    fig.suptitle('AWSIM teacher collection: observed recovery and accepted camera anchors\n'
                 'Lines: control observations; dots: causal camera anchors. Normal-driving-line-relative error.', fontsize=12)
    path = OUT / 'accepted_recovery_anchors.png'
    assert not path.exists()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    with (OUT / 'accepted_recovery_anchors_provenance.json').open('x') as f:
        json.dump(dict(sources=provenance, png_sha256=sha(path),
                       scope='Actual AWSIM teacher PP runs; no learned model or generalized recovery claim'), f, indent=2)
    print('ACCEPTED_RECOVERY_PLOT_OK')


if __name__ == '__main__':
    main()
