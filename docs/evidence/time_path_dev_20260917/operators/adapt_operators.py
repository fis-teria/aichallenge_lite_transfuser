from pathlib import Path
import json
here=Path(__file__).resolve().parent
previous=here.parent/'time_preview20_20260917'
names=['manage.py','prepare_remote.py','monitor.py','finish_and_evaluate.py','finalize_remote.py',
       'seal_archive.py','evaluate_native.py','pack_evidence.py']
for name in names:
 text=(previous/name).read_text(encoding='utf-8').replace('time_preview20_20260917','time_path_dev_20260917')
 text=text.replace('codex-time-timepreview20-lap01','codex-time-dev-lap01')
 text=text.replace('configs/control/time_path_timepreview_20kmh_20260917.json','configs/control/time_path_dev.json')
 if name=='manage.py':
  text=text.replace("'tests/fixtures']", "'tests/fixtures','Makefile','docs/time_path_make_dev.md']")
  old="command=['timeout','--signal=TERM','--kill-after=10s','710s','python3',str(root/manifest['source_dir']/'tools/run_time_path_awsim_trial.py'),'--deployment',str(root),'--run-id',{RUN_ID!r},'--display',':0','--config',{CONFIG!r},'--record-video']"
  new="command=['make','-C',str(root/manifest['source_dir']),'dev','DEV_CONTROLLER=time','MAX_SPEED_KMH=20','CORNER_MAX_SPEED_KMH=10','TIME_RECORD_VIDEO=1','TIME_RUN_ID='+{RUN_ID!r},'DISPLAY=:0']"
  assert old in text
  text=text.replace(old,new)
 if name=='prepare_remote.py':
  text += "\nlaunch_smoke=base[:3]+['-e','ROS_DOMAIN_ID=93','-e','CYCLONEDDS_URI=file:///time/smoke_dds.xml']+base[3:]+['source /aichallenge/workspace/install/setup.bash && source /time/install/setup.bash && timeout --signal=TERM --kill-after=10s 70s python3 /time/'+source.name+'/tools/check_time_dev_launch.py --checkpoint /time/command_off_best.pt --output /time/launch_smoke']\nrun('launch_smoke',launch_smoke,90)\nassert json.loads((root/'launch_smoke/summary.json').read_text())['status']=='PASS'\n"
 if name=='finish_and_evaluate.py':
  text=text.replace("'build_exit.json', 'smoke_exit.json',", "'build_exit.json', 'smoke_exit.json', 'launch_smoke_exit.json',")
  text=text.replace("(native/'deployment_transfer_verification.json').write_text", "run(['scp', HOST+':'+ROOT+'/launch_smoke/summary.json', str(native/'ros_launch_smoke_summary.json')])\n(native/'deployment_transfer_verification.json').write_text")
 (here/name).write_text(text,encoding='utf-8')
(here/'runtime_state.json').write_text(json.dumps({'checkpoint_sha256':'1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a'}),encoding='utf-8')
