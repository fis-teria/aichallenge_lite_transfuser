"""Copy small audit reports and operator sources, never bags or model tensors."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import ops_corner as m

destination = m.REPO / 'docs/evidence/time_corner_recovery_20260916'
assert not destination.exists()
assert json.loads((m.UNC / 'collection_index.json').read_bytes())['all_runs_closed_and_safe_end_verified']
remote_names = ['host_before.json', 'host_final_checks.json', 'campaign_final.json',
                'parallel_capacity_pair02.json', 'domain0_isolation.json', 'deployment.json',
                'c11_runtime_window_screen.json', 'c11_runtime_window_screen_meta.json',
                'synthetic_smoke02/synthetic_smoke_result.json']
remote_names += [f'parallel_plan_pair{n:02}.json' for n in range(1, 7)]
for name in remote_names:
    target = m.UNC / name
    assert not target.exists()
    target.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['scp', 'graneple@192.168.3.10:' + m.ROOT + '/' + name, str(target)],
                   check=True, timeout=120)
destination.mkdir()


def copy(source: Path, relative: str) -> None:
    assert source.is_file() and source.stat().st_size < 5_000_000
    target = destination / relative
    assert target.resolve().is_relative_to(destination.resolve()) and not target.exists()
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    assert hashlib.sha256(source.read_bytes()).digest() == hashlib.sha256(target.read_bytes()).digest()


for path in sorted(m.UNC.iterdir()):
    if not path.is_file():
        continue
    if path.name.endswith('_causal_probe.json') or path.name == 'skipped_window_diagnosis_before_start_window_fix.json':
        continue
    if path.suffix in ('.json', '.png') or path.name.startswith('full_') and path.suffix == '.log':
        copy(path, path.name)
copy(m.UNC / 'synthetic_smoke02/synthetic_smoke_result.json', 'synthetic_smoke_result.json')
for path in sorted((m.UNC / 'plans').glob('*.json')):
    copy(path, 'plans/' + path.name)
reference_manifest = {}
for path in sorted((m.UNC / 'references').glob('*/*.json')):
    reference = json.loads(path.read_bytes())
    relative = 'references/' + path.parent.name + '/' + path.name
    reference_manifest[relative] = dict(bytes=path.stat().st_size,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(), native_path=m.OUT + '/' + relative,
        map_screen_pass=reference['large_recovery']['map_screen_pass'],
        config=reference['large_recovery']['config'])
(destination / 'reference_manifest.json').write_text(json.dumps(reference_manifest, indent=2), encoding='utf-8')
for path in sorted(m.HERE.glob('*.py')):
    copy(path, 'operators/' + path.name)
copy(m.UNC / 'clock_audit.py', 'operators/clock_audit.py')
for path in sorted(m.HERE.glob('collect_pair*.log')):
    copy(path, 'operators/' + path.name)
for path in sorted(m.HERE.glob('audit_pair*.log')):
    copy(path, 'operators/' + path.name)
for module in (m.parallel, m.parallel.previous, m.parallel.previous.previous, m.transport):
    source = Path(module.__file__).resolve()
    copy(source, 'operator_dependencies/' + source.relative_to(m.REPO).as_posix())
raw_root = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered') / m.RAW.lstrip('/')
index = json.loads((m.UNC / 'collection_index.json').read_bytes())
for row in index['runs']:
    for name in ('result.json', 'disturbance_markers.json', 'parallel_admission.json'):
        source = raw_root / row['run_id'] / name
        if source.exists():
            copy(source, 'run_reports/' + row['run_id'] + '/' + name)
(destination / '.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
(destination / 'README.md').write_text('''# 実行証拠

実走は graneple@192.168.3.10 の2 AWSIM、検証・教師生成はnative WSL。
生bag、画像キャッシュ、教師配列、モデル重みは本ディレクトリに含めない。

- `collection_index.json`: 検証済み保存先、run split、採用数、教師ハッシュ。
- `reference_manifest.json`: 大きな経路配列はnative WSLに保持し、設定・地図判定・保存先・hashだけを記録。
- `coverage_final.json` / `coverage_final.png`: 厳格な入口状態の取得状況。未取得を含む。
- `critical_state_final.json`: 過去の失敗状態との同一座標基準・同一許容幅による比較。
- `skipped_window_diagnosis.json`: 取得窓の見送りを実測制御ログから確認。
- `corners_pair*_verified.json`: ファイル照合とSQLite検査。対応するcleanupは検証後だけ。
- `host_final_checks.json`: AWSIM全ファイルと元repoの不変、所有プロセスの終了。
- `parallel_capacity_pair02.json`: 2台同時走行中のシミュレーション時間の実測。
- `synthetic_smoke_result.json`: 公式ROSの配線試験。学習サンプルには含めない。
- `full_8cf58f1.log`: 最終実装の全pytest結果。
- `final_verification.json`: manifest対象とnative教師の最終照合記録。自己参照を避けmanifestには含めない。

`operators`は実行時ソースの保存で、ここから再起動するlauncherではない。
実行時配置は `tmp/time_corner_recovery_20260916`。依存ファイルは
`operator_dependencies`以下の元の相対パスを使用した。既存run ID・保存先を再使用しない。
再現コマンドと結果の解釈は `docs/time_corner_recovery_collection_20260916.md` を参照。
''', encoding='utf-8')
manifest = {p.relative_to(destination).as_posix(): dict(bytes=p.stat().st_size,
            sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for p in sorted(destination.rglob('*')) if p.is_file()}
(destination / 'manifest.json').write_text(json.dumps(dict(files=manifest), indent=2), encoding='utf-8')
print(json.dumps(dict(files=len(manifest), bytes=sum(r['bytes'] for r in manifest.values()),
                      destination=str(destination))))
