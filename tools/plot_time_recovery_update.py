"""Export offline comparison plots from completed, frozen predictions."""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from train_time_recovery_update import read_json, write
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, _sha


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    args = parser.parse_args()
    root, plan = args.root.resolve(), read_json(args.plan)
    out = root/plan['output']/'comparison'
    summary = read_json(out/'summary.json')
    if summary['status'] != 'COMPLETE' or summary['reserved_test_read'] or summary['new_awsim_trials']:
        raise ValueError('completed offline comparison required')
    colors = {'before': '#c77420', 'after': '#1769aa'}
    groups = [('nominal', 'Normal (4 runs)'), ('old_recovery', 'Original recovery (2 runs)'),
        ('expanded_recovery', 'Expanded recovery (2 runs)'), ('expanded_outward', 'Prior outward subset (6 frames)'),
        ('random_recovery', 'Random recovery (1 run, 3 events)'), ('random_outward', 'New outward subset (6 frames)')]
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), sharex=True, constrained_layout=True)
    horizons = ['0.5s', '1s', '2s', '3s']
    for ax, (group, title) in zip(axes.flat, groups, strict=True):
        for name in ('before', 'after'):
            metrics = summary['models'][name]['groups'][group]['xy']['run_macro_mean']
            ax.plot([.5, 1, 2, 3], [100*metrics[h]['raw_error_m'] for h in horizons],
                    'o-', color=colors[name], label=name.capitalize())
        ax.set(title=title, xlabel='Future time [s]', ylabel='Position error [cm]', ylim=(0, None))
        ax.grid(alpha=.25)
        ax.legend()
    fig.suptitle('Offline XY error: equal weight per supported run\nSame initialization and training budget; no closed-loop claim')
    fig.savefig(out/'horizon_errors.png', dpi=180)
    fig.savefig(out/'horizon_errors.pdf')
    plt.close(fig)
    ds = TimeTrainingCacheDataset(root/plan['cache'], 'validation', verify_hashes=False)
    values = {name: np.load(out/(name+'_predictions.npy'), allow_pickle=False) for name in colors}
    if any(p.shape != (len(ds), 30, 2) for p in values.values()):
        raise ValueError('prediction/cache population mismatch')
    spec = next(r for r in plan['additions'] if r['split'] == 'validation')
    collection_file = root/spec['analysis']/spec['proof']
    if _sha(collection_file) != spec['proof_sha256']:
        raise ValueError('collection changed')
    collection = next(r for r in read_json(collection_file)['runs'] if r['run_id'] == spec['run_id'])
    lookup = {aid: i for i, aid in enumerate(ds.anchor_ids)}
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    chosen = []
    for ax, event in zip(axes, collection['events'], strict=True):
        # Fixed first audited target per event; no best/worst example selection.
        anchor = event['target_anchor_ids'][0]
        i = lookup[anchor]
        chosen.append(anchor)
        target = ds.targets[i]
        if not ds.xy_mask[i].all() or not all(np.isfinite(p[i]).all() for p in values.values()):
            raise ValueError('plot requires full observed teachers and finite predictions')
        ax.plot(target[:, 0], target[:, 1], 'k-', linewidth=2, label='Observed teacher')
        for name, predictions in values.items():
            p = predictions[i]
            ax.plot(p[:, 0], p[:, 1], '--', color=colors[name], label=name.capitalize())
            ax.scatter(p[[4, 9, 19, 29], 0], p[[4, 9, 19, 29], 1], color=colors[name], s=14)
        ax.set(title=f"Event {event['event_id']} / first audited outward frame",
               xlabel='Forward in observation body [m]', ylabel='Left in observation body [m]')
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.suptitle('Held-out random recovery: identical recorded inputs\nLateral axis enlarged for visibility; these are predictions, not driven paths')
    fig.savefig(out/'random_recovery_paths.png', dpi=180)
    fig.savefig(out/'random_recovery_paths.pdf')
    plt.close(fig)
    write(out/'plot_manifest.json', dict(selection='first_audited_outward_anchor_per_event',
        anchor_ids=chosen, summary_sha256=_sha(out/'summary.json'),
        prediction_sha256={name: _sha(out/(name+'_predictions.npy')) for name in values},
        files={p.name: _sha(p) for p in sorted(out.glob('*.png'))} |
              {p.name: _sha(p) for p in sorted(out.glob('*.pdf'))}))
    print('OFFLINE_PLOTS_COMPLETE', flush=True)


if __name__ == '__main__':
    main()
