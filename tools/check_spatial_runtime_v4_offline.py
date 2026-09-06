"""One bounded offline diagnosis; no Dataset/raw access or ROS initialization."""
from __future__ import annotations

import argparse
import io
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import numpy as np
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_FIELDS
from aic_transfuser_lite.runtime.spatial_input_v4 import INPUT_ID, freeze_batch
from aic_transfuser_lite.runtime.spatial_recording_v4 import Records, PrivateWriter, encoded, sha
from aic_transfuser_lite.runtime.spatial_runtime_v4 import SpatialRuntimeV4, load_fixed, state_inventory, CHECKPOINT_SHA

ROOT=Path(__file__).resolve().parents[1]
PRIOR=Path('/home/thistle/e2e_autonomous/runs/spatial_validation_v4_20260906_153a22a')
SELECTION_ID='59393d98ad4e55a59da515ff324a147995889e98b48b3a7ac676660400e3851f'
TEACHER_ID='cdb668834d4baf60c31fa5f934d01d8782f02544bc6fa29053d5a850432cc344'
ALLOW={'artifacts/execution_manifest.json','artifacts/input_contract.json','artifacts/teacher_contract.json','artifacts/selection.json','artifacts/resolved_config.yaml',
       'evidence/validation_histories.json','evidence/validation_predictions.npz','evidence/input_examples/input_0.npz','evidence/input_examples/input_1.npz'}


def write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(encoded(value)+b'\n')


def binding_report() -> dict:
    return {'real_input_binding':'BLOCKED','graph_access':'NOT_EXECUTED','inputs':{
        'image':dict(topic_candidate='/sensing/camera/image_raw',type='sensor_msgs/msg/Image',fields='header.stamp, rgb8 data',qos='sensor_data from V3',source='transfuser_lite_v3_shadow.launch.py',live='UNKNOWN'),
        'lidar':dict(topic_candidate='/sensing/lidar/scan',type='sensor_msgs/msg/LaserScan',fields='header.stamp,ranges,range_min,range_max,angle geometry',qos='sensor_data from V3',live='UNKNOWN',missing='750-beam geometry and validity policy binding'),
        'ego':dict(topic_candidate=None,type='autoware_auto_vehicle_msgs/msg/VelocityReport + SteeringReport',fields='longitudinal_velocity,lateral_velocity,heading_rate,steering_tire_angle',qos='depth10 from V3',missing='exact camera alignment or approved causal interpolation; live timestamps/source'),
        'command':dict(topic_candidate='/nominal_control_cmd',type='autoware_auto_control_msgs/msg/AckermannControlCommand',fields='stamp,lateral.steering_tire_angle,longitudinal.speed,longitudinal.acceleration',source='V3 parameter default only, not producer verification',missing='passive producer/time availability/semantic confirmation',final_fallback='DISABLED_UNVERIFIED')},
        'timing':{'camera_delta':'camera.header-grid ns /1e9','lidar_delta':'lidar.header-camera.header ns /1e9','source':'canonical_converter_v3.py quality construction',
                  'wrapper_grid':'100ms nearest epoch-zero grid; actual training grid phase unknown: NOT live parity'},
        'live_requirements':['passive command producer and timestamp semantics','grid phase/camera selection','ego alignment','LiDAR geometry/range validity','QoS and clock epochs','separate live execution envelope and startup approval']}


def run(output: Path) -> int:
    output.mkdir(parents=True,exist_ok=False)
    for folder in ('artifacts','evidence','provenance','logs','reports'):
        (output/folder).mkdir()
    manifest=dict(implementation_authorized=True,synthetic_tests_authorized=True,offline_tensor_inference_authorized=True,
        live_sensor_connection_authorized=False,new_collection_authorized=False,shadow_connection_authorized=False,control_connection_enabled=False,
        raw_execution_authorized=False,training_authorized=False,runtime_promotion_authorized=False,approval_gate='PENDING_EXPLICIT_AUTHORIZATION_FOR_LIVE_SHADOW',
        execution_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        status='STARTED',offline_forward_calls=0,training_steps=0,live_connections=0,log_writes=0,
        schema_validation='NOT_EXECUTED_missing_draft2020_validator',semantic_validation='PENDING')
    write(output/'artifacts/execution_manifest.json',manifest)
    write(output/'artifacts/input_binding_report.json',binding_report())
    write(output/'artifacts/resolved_config.json',dict(queue_capacity=4,max_record_bytes=131072,max_file_bytes=8388608,
        device='cuda' if torch.cuda.is_available() else 'cpu',real_input_limit=2,forward_limit=6,rtol=1e-5,atol_m=1e-6,precision='float32',final_fallback=False))
    write(output/'provenance/environment.json',dict(python=sys.version,platform=platform.platform(),torch=torch.__version__,numpy=np.__version__,cuda=torch.version.cuda,
        cuda_available=torch.cuda.is_available(),gpu=torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,argv=sys.argv))
    core=writer=model=None
    accessed={}
    def read(name):
        if name not in ALLOW: raise ValueError('not allowlisted')
        path=PRIOR/name
        if path.is_symlink() or path.stat().st_size>40_000_000: raise ValueError('symlink/size cap')
        blob=path.read_bytes(); accessed[name]=dict(bytes=len(blob),sha256=sha(blob))
        dest=output/'evidence/source'/name
        dest.parent.mkdir(parents=True,exist_ok=True); dest.write_bytes(blob)
        return blob
    def contract(name,expected):
        v=json.loads(read(name))
        if v.get('identity')!=expected or sha(encoded({k:x for k,x in v.items() if k!='identity'}))!=expected:
            raise ValueError('contract identity '+name)
        return v
    try:
        prior_manifest=json.loads(read('artifacts/execution_manifest.json'))
        if prior_manifest['execution_commit']!='153a22a8b85ebcf21436abf9ab99c94800687cea': raise ValueError('execution version')
        contract('artifacts/input_contract.json',INPUT_ID)
        contract('artifacts/teacher_contract.json',TEACHER_ID)
        selection=contract('artifacts/selection.json',SELECTION_ID)
        histories=json.loads(read('evidence/validation_histories.json'))
        read('artifacts/resolved_config.yaml')
        ids=[x['sample_id'] for x in selection['selected'][:2]]
        if len(set(ids))!=2 or [h['anchor_id'] for h in histories[:2]]!=ids:
            raise ValueError('BLOCKED example selection/history identity')
        with np.load(io.BytesIO(read('evidence/validation_predictions.npz')),allow_pickle=False) as data:
            if any(data[k].dtype.hasobject for k in data.files): raise ValueError('object array')
            if data['sample_ids'][:2].tolist()!=ids or not data['processed'][:2].all(): raise ValueError('prediction ID/process mismatch')
            expected=data['xy'][:2].copy()
        batches=[]
        for i in range(2):
            with np.load(io.BytesIO(read(f'evidence/input_examples/input_{i}.npz')),allow_pickle=False) as data:
                if set(data.files)!=set(INPUT_FIELDS) or any(data[k].dtype.hasobject for k in data.files): raise ValueError('fixture fields')
                batch=ModelBatchV3(**{k:torch.from_numpy(data[k].copy()) for k in INPUT_FIELDS},targets=None,requested_outputs=frozenset({'trajectory'}))
                batches.append(freeze_batch(batch))
        write(output/'evidence/representative_join.json',dict(ids=ids,source='save_examples i<2, validation selected order at 153a22a',histories=histories[:2]))
        torch.set_num_threads(4)
        torch.backends.cuda.matmul.allow_tf32=False
        torch.backends.cudnn.allow_tf32=False
        torch.backends.cudnn.benchmark=False
        device='cuda' if torch.cuda.is_available() else 'cpu'
        model,loadmap=load_fixed(device)
        write(output/'provenance/checkpoint_load_map.json',loadmap)
        before=state_inventory(model); write(output/'provenance/state_before.json',before)
        records=Records(ROOT/'schemas',mode='OFFLINE_TENSOR_REPLAY')
        writer=PrivateWriter(output/'evidence/runtime_events.jsonl',records)
        core=SpatialRuntimeV4(model,records,writer,device=device,checkpoint_hash=CHECKPOINT_SHA)
        results=[]
        for i,batch in enumerate(batches):
            event=core.infer(batch)
            write(output/f'evidence/result_{i}.json',event)
            if event['payload']['output']['status']!='SHAPE_FINITE_ONLY': raise ValueError('offline output failure')
            results.append(np.frombuffer(bytes.fromhex(event['payload']['output']['float32_le_hex']),dtype='<f4').reshape(20,2).copy())
        actual=np.stack(results)
        difference=actual-expected
        passed=bool(np.isclose(actual,expected,rtol=1e-5,atol=1e-6).all())
        np.savez_compressed(output/'evidence/offline_comparison.npz',sample_ids=np.asarray(ids),expected=expected,actual=actual,difference=difference)
        comparison=dict(passed=passed,rtol=1e-5,atol_m=1e-6,max_abs_difference_m=float(np.abs(difference).max()),ids=ids,
                        per_anchor_max_abs_m=np.abs(difference).max(axis=(1,2)).tolist(),batch_size=1,original_batch_size=2,device=device)
        write(output/'artifacts/offline_comparison.json',comparison)
        after=state_inventory(model); write(output/'provenance/state_after.json',after)
        if before!=after: raise ValueError('model state mutated')
        records.validate(records.event('SESSION_END'))
        writer.enqueue(records.event('SESSION_END')); writer.drain()
        manifest.update(status='OFFLINE_PARITY_PASS' if passed else 'OFFLINE_PARITY_FAIL',state_unchanged=True,semantic_validation='PASSED',real_input_binding='BLOCKED',live_input_runtime_tested='NOT_EXECUTED',control_connection='NOT_IMPLEMENTED')
    except Exception as exc:
        manifest.update(status='BLOCKED',error=type(exc).__name__+': '+str(exc))
        traceback.print_exc()
    finally:
        if writer:
            writer.close()
            manifest.update(log_writes=writer.log_writes,writer_failed=writer.failed,writer_error=writer.error)
        if core: manifest['offline_forward_calls']=core.forward_calls
        write(output/'provenance/allowlisted_reads.json',accessed)
        write(output/'artifacts/execution_manifest.json',manifest)
        print(json.dumps(manifest,indent=2))
    return 0 if manifest['status']=='OFFLINE_PARITY_PASS' else 2


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True)
    raise SystemExit(run(p.parse_args().output))
