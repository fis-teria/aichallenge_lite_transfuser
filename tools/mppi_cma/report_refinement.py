"""Export exactly the references selected by the completed repeated comparison."""
from __future__ import annotations

import hashlib
import argparse
import io
import json
from pathlib import Path
import shutil
import tarfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml
from continuation_state import study_path

ROOT = Path('/home/si26-pc008/cma_mppi_20260912')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--state-subdir', default='refinement')
    parser.add_argument('--output-subdir', default='refinement_deliverables')
    args = parser.parse_args()
    study = study_path(ROOT, args.state_subdir)
    output = study_path(ROOT, args.output_subdir)
    if output == study or output.is_relative_to(study / 'normal') or output.is_relative_to(study / 'leader'):
        raise ValueError('Export directory must not overwrite study state or optimizer inputs')
    state = json.loads((study/'state.json').read_text())
    if not state['completed'] or not all(p['validated_preferred_feasible'] for p in state['conditions'].values()):
        raise RuntimeError('The finite study and selected-route validations must be complete')
    environment = json.loads((ROOT/'environment.json').read_text())
    output.mkdir(parents=True, exist_ok=True)
    with tarfile.open(ROOT/'snapshot/submission.tar.gz') as archive:
        def member(suffix: str) -> bytes:
            matches = [m for m in archive.getmembers() if m.isfile() and m.name.endswith(suffix)]
            if len(matches) != 1:
                raise ValueError('Map member is not unique')
            return archive.extractfile(matches[0]).read()
        wall = np.asarray(Image.open(io.BytesIO(member('reference_space_mppi_planner/config/awsim_wall_map/occupancy_grid_map.pgm'))))
        cfg = yaml.safe_load(member('reference_space_mppi_planner/config/awsim_wall_map/occupancy_grid_map.yaml'))
    polygon = np.asarray(json.loads((ROOT/'calibration.json').read_text())['ot_lane_polygon_map_m'])
    original = np.genfromtxt(ROOT/'snapshot/base_reference.csv', delimiter=',', names=True)
    offset = np.array([89600.,43100.]); origin = cfg['origin']; resolution = cfg['resolution']
    figure, axes = plt.subplots(1, 2, figsize=(13, 8), constrained_layout=True)
    summary = {'scope': 'Bounded refinement, 24 CMA candidates per condition plus four runs of each compared route',
               'controller_image_id': environment['controller_image_id'],
               'new_vehicle_evaluations': state['new_episodes_started'], 'conditions': {}}
    for axis, (condition, progress) in zip(axes, state['conditions'].items()):
        comparison = progress['comparison']
        role = 'candidate' if comparison['promoted'] else 'incumbent'
        selected = progress['selected']; runs = progress['comparison_runs'][role]
        representative = runs[0]
        episode = ROOT/'episodes'/representative.get('output_episode', representative['episode'])
        suffix = f"-d{representative['vehicle_number']}" if representative.get('shared_course') else ''
        track_path = episode/f'track{suffix}.csv'
        reference = np.genfromtxt(selected['reference'], delimiter=',', names=True)
        incumbent = np.genfromtxt(progress['incumbent']['reference'], delimiter=',', names=True)
        trace = np.genfromtxt(track_path, delimiter=',', names=True)
        axis.imshow(wall, cmap='gray', vmin=0, vmax=255, origin='upper', extent=[
            origin[0]-offset[0],origin[0]+resolution*wall.shape[1]-offset[0],
            origin[1]-offset[1],origin[1]+resolution*wall.shape[0]-offset[1]])
        axis.fill(polygon[:,0]-offset[0], polygon[:,1]-offset[1], color='#dc3545', alpha=.4, label='OT lane')
        for data, color, label, style in [(original,'#666','Original baseline','--'),
                                          (incumbent,'#dc8b20','Previous selected',':'),
                                          (reference,'#0869c1','Selected reference','-'),
                                          (trace,'#00a676','Measured GNSS','-')]:
            axis.plot(data['x_m']-offset[0], data['y_m']-offset[1], color=color, lw=1.3,
                      linestyle=style, alpha=.85, label=label)
        lap = comparison['candidate_median_s'] if comparison['promoted'] else comparison['incumbent_median_s']
        axis.set_title(f"{condition.title()} · target {representative['target_mps']:.1f} m/s\nSelected lap median {lap:.3f} s (4 runs)")
        axis.set_aspect('equal'); axis.set_xlim(8,83); axis.set_ylim(17,96)
        axis.set_xlabel('Map x - 89600 (m)'); axis.set_ylabel('Map y - 43100 (m)')
        axis.legend(loc='upper left', fontsize=8)
        exported = output/f'{condition}_reference.csv'
        shutil.copy2(selected['reference'], exported)
        shutil.copy2(episode/'mppi.yaml', output/f'{condition}_mppi.yaml')
        shutil.copy2(track_path, output/f'{condition}_measured_track.csv')
        shutil.copy2(study/condition/'optimizer_config.json',
                     output/f'{condition}_optimizer_config.json')
        layout = 'four ghosts on one course, same D1 start, no other-vehicle controller input' if condition == 'normal' else 'independent single-car AWSIM, native rank 1 throughout'
        summary['conditions'][condition] = {
            'evaluation_layout': layout, 'target_mps': representative['target_mps'],
            'selected_candidate': selected.get('name', selected['metrics']['episode']),
            'compared_new_candidate': progress['best'].get('name', progress['best']['metrics']['episode']),
            'selected_anchors_m': selected['anchors_m'], 'comparison': comparison,
            'selected_reference_sha256': hashlib.sha256(exported.read_bytes()).hexdigest(),
            'selected_runs': runs, 'selected_dense_checks': progress['comparison_dense'][role],
            'evaluated_candidates': len(progress['evaluations'])}
        (output/f'{condition}_execution.json').write_text(json.dumps({
            'reference_file': exported.name, 'mppi_yaml_file': f'{condition}_mppi.yaml',
            'reference_execution_speed_cap_mps': representative['target_mps'],
            'native_handicap_enabled': condition == 'leader', 'evaluation_layout': layout,
            'corner_acceleration_allowance_mps2': .6}, indent=2))
    figure.savefig(output/'routes.png', dpi=170)
    figure.savefig(output/'routes.svg'); plt.close(figure)
    (output/'summary.json').write_text(json.dumps(summary, indent=2))
    shutil.copy2(study/'state.json', output/'search_state.json')
    shutil.copy2(ROOT/'calibration.json', output/'calibration.json')
    shutil.copy2(ROOT/'d1_start_pose.json', output/'d1_start_pose.json')
    if state.get('checkpoint_source_hashes'):
        (output/'preflight.json').write_text(json.dumps({
            'previous_study': state['previous_study'], 'previous_state_sha256': state['previous_search_sha256'],
            'checkpoint_source_hashes': state['checkpoint_source_hashes']}, indent=2))
    else:
        shutil.copy2(ROOT/'refinement_preflight.json', output/'preflight.json')
    shutil.copy2(ROOT/'refinement_optimizer_smoke.json', output/'optimizer_smoke.json')
    (output/'.gitattributes').write_text('* -text\n')
    (output/'README.md').write_text(
        '# Repeatedly validated MPPI refinement\n\n'
        'Use each exact reference CSV with its paired MPPI YAML and the '
        '`reference_execution_speed_cap_mps` in the corresponding execution JSON. '
        'The original CSV speed column alone is not the execution-speed contract.\n\n'
        'Normal candidates were compared as four ghosts sharing one AWSIM from the same D1 pose, '
        'with both planner and recovery other-vehicle inputs empty. Leader candidates were evaluated '
        'in isolated single-car simulations so native rank-1 handicap remained active throughout.\n\n'
        'Selection requires all four repeats and dense OT checks to pass, at least 0.05 seconds '
        'median improvement, and three wins in four ordered comparisons; otherwise the incumbent '
        'is retained. See summary.json for the actual decision, not just the fastest search sample. '
        'This is a finite local search and does not establish a global optimum.\n')
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir())
              if p.is_file() and p.name != 'sha256.json'}
    (output/'sha256.json').write_text(json.dumps(hashes, indent=2))
    print(json.dumps({'output':str(output), 'files':len(hashes)+1,
                      'comparisons':{k:v['comparison'] for k,v in summary['conditions'].items()}}))


if __name__ == '__main__':
    main()
