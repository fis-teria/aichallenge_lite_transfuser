"""Write verified final counts into the Windows report, without changing evidence."""
import json
from pathlib import Path
import manage as m

evidence=m.REPO/'docs/evidence/time_recovery_separated_collection_20260915'
def read(name: str) -> dict:
    return json.loads((evidence/name).read_bytes())
index=read('collection_index.json');verified=read('completion_verification.json')
assert verified['status']=='COMPLETE_VERIFIED' and not index['pending_runs']
holds=index['holds'];by_site={}
for h in holds:by_site.setdefault(h['site_id'],{})[h['sign']]=h
counts=index['accepted_anchors_by_split'];n=sum(counts.values())
lines=[
    '## 確定した収集結果', '',
    'graneple@192.168.3.10 のAWSIMで、過去のE2E試験の停止地点S00＋追加10地点の左右、計22復帰イベントを収集した。',
    f'正常完周は10周。外乱前の計算期限超過1回を含め、総試行は11回で終了し、追加の自動試行はない。',
    f'WSLで検証・教師化したサンプルは**{n:,}件（train {counts["train"]:,} / validation {counts["validation"]:,}）**。',
    '件数は重なりのある時系列アンカーであり、独立した復帰の試行数は22件。',
    'trainは8周、validationはR01/R04/R08の左右2周で、run単位で分割した。封印testは読み込んでいない。', '',
    '| 地点 | 左の採用数 | 右の採用数 | 左/右の連続保持 [s] | 左/右の最大横ずれ [cm] | split |',
    '|---|---:|---:|---|---|---|',
]
for site in ['S00',*(f'R{i:02d}' for i in range(1,11))]:
    left,right=by_site[site][1],by_site[site][-1]
    split=next(r['split'] for r in index['runs'] if r['run_id']==left['run_id'])
    lines.append(f'| {site} | {left["accepted"]} | {right["accepted"]} | '
        f'{left["issued_full_plateau_s"]:.3f} / {right["issued_full_plateau_s"]:.3f} | '
        f'{100*left["peak_signed_lateral_m"]:.2f} / {100*right["peak_signed_lateral_m"]:.2f} | {split} |')
lines += ['',
    f'連続保持の厳格基準（0.95 s以上、送信間隔150 ms以下）を満たしたのは{index["qualified_one_second_holds"]}/22件。',
    '表は小数3桁表示であり、合否は丸め前の値で判定した。例外の正確な値・時刻間隔は',
    '[hold_evidence_detail.json](evidence/time_recovery_separated_collection_20260915/hold_evidence_detail.json)を参照。',
    '22件すべてでPP復帰確認と60件以上の有効教師を確認している。保持判定が未達のイベントも、',
    '解除後の観測・実測未来が有効なサンプルを別条件で採用している。', '',
    '原本は失敗1回を含め11回分すべてWSLへ移送し、SHA-256・全ファイルとディレクトリ構造・',
    'SQLite整合性を確認した。教師用10周はセンサ形式・時刻逆転・因果履歴・未来実測を検証し、',
    '準備済み入力と元の観測からの入力再現一致も確認した。教師shapeは[N,30,2]、0.1 s間隔・3 s先まで。',
    '実行用594ファイルは5289101と一致し、元のリモートcheckoutのHEAD/作業状態も維持した。',
    'この収集用コンテナはすべて終了し、AWSIM側の検証済み転送元だけを片付けた。', '',
    '保存先（いずれもWSLのLinuxファイルシステム）:', '',
    '- 原本: `/home/thistle/e2e_autonomous/raw/time_recovery_separated_20260915`',
    '- 圧縮アーカイブ・分析: `/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915`',
    '- 実測教師: 上記分析ディレクトリの`materialized/`',
    '- run別の前処理済み入力・教師: 同`prepared/train/`と`prepared/validation/`', '',
    '[collection_index.json](evidence/time_recovery_separated_collection_20260915/collection_index.json)に全run・split・教師数・',
    '前処理ファイルのSHAを記録し、[completion_verification.json](evidence/time_recovery_separated_collection_20260915/completion_verification.json)',
    'で11地点×左右の網羅と、代替走行以外の地点・参照・seed・splitが元の計画と同一であることを検証した。',
    'run別キャッシュであり、既存学習コーパスへの統合・学習用の最終cache manifest生成は次の学習準備で行う。', '',
    f'今回観測した最大横ずれの範囲は{100*min(h["peak_signed_lateral_m"] for h in holds):.2f}～'
    f'{100*max(h["peak_signed_lateral_m"] for h in holds):.2f} cm。小さな逸脱からの復帰データとして扱う。',
    '大きな横逸脱への復帰や、再学習後モデルの完走性能はこの収集で検証していない。',
    '目標速度は5 km/h（1.3889 m/s）、速度制御は既存の`aligned_gain4_v1`で、実測速度と目標速度は区別する。', '',
    '最終集計コマンド（Windowsの実行スクリプトからnative WSLのworktree lockを通す）:', '',
    '```powershell',
    'python tmp/time_recovery_separated_20260915/finalize_native.py',
    'python tmp/time_recovery_separated_20260915/collect_evidence.py',
    'python tmp/time_recovery_separated_20260915/append_final_report.py',
    '```', '',
    '使用したスクリプトの保全コピーは`docs/evidence/time_recovery_separated_collection_20260915/`。',
    '出力は上書きせず既存ファイルがあれば停止するため、完了済み対象に上記コマンドを再実行しない。',
    'データ・重み・bagはGitへ追加していない。新規モデル学習はまだ開始していない。',
]
path=m.REPO/'docs/time_recovery_separated_collection_20260915.md'
text=path.read_text(encoding='utf-8')
old='全体の収集結果・データ一覧は処理完了後に追記する。'
assert text.count(old)==1
text=text.replace(old,'\n'.join(lines)).replace('出版','送信')
path.write_text(text,encoding='utf-8')
print(json.dumps(dict(report=str(path),accepted=n,train=counts['train'],validation=counts['validation'])))
