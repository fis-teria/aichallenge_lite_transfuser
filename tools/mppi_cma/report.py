"""Export selected, tested reference files and a standalone comparison map."""
from __future__ import annotations
import io
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import tarfile
import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from verify_tracks import verify

ROOT=Path('/home/si26-pc008/cma_mppi_20260912')


def main() -> None:
    state=json.loads((ROOT/'search/state.json').read_text())
    if not state['completed']:raise RuntimeError('Search and repeats have not finished')
    output=ROOT/'deliverables';output.mkdir(exist_ok=True)
    summary={'scope':'Solo closed-loop AWSIM, second-lap time, 18 CMA candidates per condition',
             'ot_metric':'GNSS XY plus timestamp-aligned EKF yaw; bounding rectangle and 0.1 m margin',
             'controller_image_id':json.loads((ROOT/'environment.json').read_text())['controller_image_id'],
             'conditions':{}}
    with tarfile.open(ROOT/'snapshot/submission.tar.gz') as archive:
        def member(suffix: str) -> bytes:
            matches=[m for m in archive.getmembers() if m.isfile() and m.name.endswith(suffix)]
            if len(matches)!=1:raise ValueError('Archive map member is not unique')
            return archive.extractfile(matches[0]).read()
        wall=Image.open(io.BytesIO(member('reference_space_mppi_planner/config/awsim_wall_map/occupancy_grid_map.pgm')))
        wall_cfg=yaml.safe_load(member('reference_space_mppi_planner/config/awsim_wall_map/occupancy_grid_map.yaml'))
        wall_array=np.asarray(wall)
    origin=wall_cfg['origin'];res=wall_cfg['resolution']
    polygon=np.array(json.loads((ROOT/'calibration.json').read_text())['ot_lane_polygon_map_m'])
    base=np.genfromtxt(ROOT/'snapshot/base_reference.csv',delimiter=',',names=True)
    offset=np.array([89600.,43100.])
    fig,axes=plt.subplots(1,2,figsize=(13,8),constrained_layout=True)
    for ax,(condition,progress) in zip(axes,state['conditions'].items()):
        best=progress['best'];metrics=best['metrics']
        chosen=[metrics,*progress['best_repeats']]
        baseline=[progress['evaluations'][0]['metrics'],progress['baseline_repeat']]
        dense=[verify(ROOT,x['episode']) for x in chosen]
        ref=np.genfromtxt(best['reference'],delimiter=',',names=True)
        track=np.genfromtxt(ROOT/'episodes'/metrics['episode']/'track.csv',delimiter=',',names=True)
        ax.imshow(wall_array,cmap='gray',vmin=0,vmax=255,origin='upper',
                  extent=[origin[0]-offset[0],origin[0]+res*wall_array.shape[1]-offset[0],
                          origin[1]-offset[1],origin[1]+res*wall_array.shape[0]-offset[1]])
        ax.fill(polygon[:,0]-offset[0],polygon[:,1]-offset[1],color='#dc3545',alpha=.4,label='OT lane')
        ax.plot(base['x_m']-offset[0],base['y_m']-offset[1],color='#555',linestyle='--',lw=1,label='Baseline reference')
        ax.plot(ref['x_m']-offset[0],ref['y_m']-offset[1],color='#0065c8',lw=1.5,label='Selected reference')
        ax.plot(track['x_m']-offset[0],track['y_m']-offset[1],color='#00a676',alpha=.8,lw=1,label='Measured GNSS track')
        ax.set_aspect('equal');ax.set_xlim(8,83);ax.set_ylim(17,96)
        ax.set_xlabel('Map x - 89600 (m)');ax.set_ylabel('Map y - 43100 (m)')
        lap_values=[x['flying_lap_s'] for x in chosen]
        median=statistics.median(lap_values)
        baseline_median=statistics.median(x['flying_lap_s'] for x in baseline)
        ax.set_title(f'{condition.title()}: target {metrics["target_mps"]:.1f} m/s\nFlying lap median {median:.3f} s (3 runs)')
        ax.legend(loc='upper left',fontsize=8)
        case={'selected_episode':metrics['episode'],'target_mps':metrics['target_mps'],
              'handicap_enabled':metrics['handicap_enabled'],'rank_values':[1],
              'cma_candidates':len(progress['evaluations'])-2,
              'selected_laps_s':lap_values,'selected_median_s':median,
              'baseline_laps_s':[x['flying_lap_s'] for x in baseline],'baseline_median_s':baseline_median,
              'selected_minus_baseline_s':median-baseline_median,
              'selected_ot_overlap_s':[x['ot_overlap_s'] for x in chosen],
              'baseline_ot_overlap_s':[x['ot_overlap_s'] for x in baseline],
              'selected_penalties':[x['penalties'] for x in chosen],
              'validated_preferred_feasible':progress['validated_preferred_feasible'],
              'dense_checks':dense,
              'validated_with_interpolation':progress['validated_preferred_feasible'] and all(x['passed'] for x in dense),
              'anchors_m':best['anchors_m'],
              'measured_speed_median_mps':metrics['measured_speed_median_mps'],
              'selected_speed_medians_mps':[x['measured_speed_median_mps'] for x in chosen],
              'selected_speed_maxima_mps':[x['measured_speed_max_mps'] for x in chosen],
              'gentle_acceleration_in_corners_s':[x['gentle_acceleration_in_corners_s'] for x in chosen]}
        summary['conditions'][condition]=case
        shutil.copy2(best['reference'],output/(condition+'_reference.csv'))
        episode=ROOT/'episodes'/metrics['episode']
        shutil.copy2(episode/'mppi.yaml',output/(condition+'_mppi.yaml'))
        shutil.copy2(episode/'config.json',output/(condition+'_episode.json'))
        shutil.copy2(episode/'track.csv',output/(condition+'_measured_track.csv'))
    fig.savefig(output/'routes.png',dpi=170)
    fig.savefig(output/'routes.svg')
    plt.close(fig)
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    shutil.copy2(ROOT/'calibration.json',output/'calibration.json')
    shutil.copy2(ROOT/'snapshot/base_reference.csv',output/'baseline_reference.csv')
    (output/'README.md').write_text(
        '# Bounded MPPI raceline search\n\n'
        'The reference CSVs are the exact tested geometry files. Use each with its paired MPPI YAML '
        'and `reference_execution_speed_cap_mps:=10.0` (normal) or `:=7.5` (leader). '
        'The legacy CSV vx column is not the effective target; the execution-profile parameter '
        'and the MPPI cruise/overtake/proximity parameters set the evaluated target.\n\n'
        'Controller image: '+json.loads((ROOT/'environment.json').read_text())['controller_image_id']+'\n\n'
        'The current MPPI and its corner-acceleration controller remain enabled. '
        'These are solo closed-loop references, not a guarantee for traffic/overtaking or a global optimum. '
        'See summary.json for all repeated-run outcomes; a selected path is only preferred-feasible '
        'when all three checks completed without contact/over penalties or measured OT overlap. '
        'The dense_checks and validated_with_interpolation fields additionally check interpolated footprints. '
        'No production reference was replaced.\n')
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir())
              if p.is_file() and p.name!='sha256.json'}
    (output/'sha256.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
