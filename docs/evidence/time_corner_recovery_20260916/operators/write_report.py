"""Append audited results to the Windows source document."""
import json
import ops_corner as m

read = lambda name: json.loads((m.UNC / name).read_bytes())
index = read('collection_index.json')
coverage = read('coverage_final.json')
critical = read('critical_state_final.json')
host = read('host_final_checks.json')
path = m.REPO / 'docs/time_corner_recovery_collection_20260916.md'
text = path.read_text(encoding='utf-8')
assert '## 実走収集結果' not in text
lines = ['\n## 実走収集結果\n',
    f"有限予算12 runのうち{len(index['runs'])} runを実行した。"
    f"競技judgeの完走確認は{sum(r['judge_lap_confirmed'] for r in index['runs'])} run、"
    f"最終 `COMPLETE_LAP` は{sum(r['status']=='COMPLETE_LAP' for r in index['runs'])} run。"
    '教師採用は完走・停止・bag終了・無fault・センサ健全性と各anchorの因果入力/全未来を要求した。\n',
    '| pair | domain 1 / train | domain 2 / validation |',
    '|---|---|---|']
for pair in range(1, 7):
    cells = []
    for domain in (1, 2):
        row = next(r for r in index['runs'] if r['run_id'] == f'codex-time-recovery-corners-p{pair:02}-d{domain}')
        events = ', '.join(e['site_id'] + ':' + str(e['accepted']) for e in row['teacher_anchors_per_event']) or 'イベントなし'
        cells.append(f"`{row['status']}`、採用{row['accepted']}（{events}）")
    lines.append(f'| {pair:02} | {cells[0]} | {cells[1]} |')
lines += ['\n| split | 独立採用run | 採用復帰イベント | camera anchor | 目標横ずれ±5 cmのanchor |',
          '|---|---:|---:|---:|---:|']
for split, total in index['totals'].items():
    lines.append(f"| {split} | {total['runs']} | {total['events']} | {total['anchors']} | {total['target_band_anchors']} |")
lines += ['\nanchorは同一復帰内の連続カメラ観測であり、独立シナリオ数ではない。'
          '横ずれ帯の件数と、横ずれ・向き・進入位置・速度が同時に条件を満たす件数を分けて扱う。',
          f"\nraw合計は{index['raw_bytes']/1e9:.3f} GB（{index['raw_bytes']/2**30:.3f} GiB）。"
          '各pairのarchiveとrawはnative WSLへ移管し、全ファイル・構造・SQLiteを照合済み。',
          '\n保存先:',
          '\n- raw: `/home/thistle/e2e_autonomous/raw/time_corner_recovery_20260916`',
          '- 教師・入力cache・集計: `/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916`',
          '- `collection_index.json` にmaterialized/preparedパス、run split、教師hashを記録。',
          '- 既存コーパスへの統合・再学習はこの収集タスクでは未実施。',
          '\n## コーナーごとの取得状態\n',
          '| 地点 | release [m] | train採用anchor | validation採用anchor | 入口状態の完了判定 train / validation |',
          '|---|---:|---:|---:|---|']
for corner, row in coverage['corners'].items():
    counts = {split: sum(e['accepted'] for r in index['runs'] if r['split'] == split
                        for e in r['teacher_anchors_per_event'] if e['corner_id'] == corner)
              for split in ('train', 'validation')}
    qualified = ['取得条件達成' if row[split] else '未達' for split in ('train', 'validation')]
    lines.append(f"| {corner} | {row['target']['release_s_m']:.2f} | {counts['train']} | {counts['validation']} | {' / '.join(qualified)} |")
lines += ['\n![取得状態](evidence/time_corner_recovery_20260916/coverage_final.png)',
    '\n必須11地点の全条件達成: **' + ('達成' if coverage['complete'] else '未達') + '**。',
    '\n- C03・C06: 現在の準備/復帰候補は1.4 m円による地図審査を通らず、実走外乱を未実施。'
    '準備長、整定区間、符号、周辺release位置も調べたが、同じ入口目標の安全な候補を確定できていない。',
    '- C07: 短い準備区間に変更した再試行でも、速度が1.4 m/sを超えるため開始条件の1秒安定を満たせず見送り。'
    '地図審査合格だけで動的取得成功とは扱わない。',
    '- C01/trainとC04/validation: 有効な復帰教師は得られたが、横ずれ・向き・位置・速度が同時に目標に一致する入口anchorは各2件で、3件/runの条件には未達。',
    '- C10: 復帰教師の採用数と入口状態3件/runの判定は別。初期のtrain 2 runは該当状態が各2件で、'
    '合計4件を1 runの3件として足し合わせていない。',
    '- C11: pair05/trainは開始窓の安定時間が約0.9秒で見送り、validationは取得。'
    '同じ実測ログで開始を302.67 mから304.67 mへ移した場合の安定を確認し、'
    '最後のvalidationには準備長6 m＋整定4 mを設定した。release 314.67 mと目標状態は同一。'
    'ただし先行するC05_LATEが中断し、C11の短い準備経路は実走未検証。',
    '- pair01/domain2: 公式startは受付されたがhelper終了が20秒待ちを超過し、走行許可前に失敗。'
    'parallel時60秒待ちへ修正後は、2台同時走行を確認した。',
    '- pair03/domain1: 完走後、停止確認中のIMU Z角速度1メッセージが非有限値。'
    '停止開始2.05秒後、最後の復帰＋将来3秒の区間から225.73秒離れていた。最終faultを理由にrun全体を採用0とした。',
    '\n## 過去の失敗状態との照合\n',
    'C05の入口181.57 mに加え、失敗前の185.61 mへ外向き状態を置く `C05_LATE / C05A` を追加候補にした。'
    '両runとも準備中に速度1.4 m/sの条件を超えて中断し、追加目標の教師は未取得。'
    'これはC05入口の取得判定を置き換える地点ではない。下表は新規採用anchorだけを、'
    '元の診断と同じr30実測基準・同じ許容幅へ変換して照合した結果。',
    '\n| 失敗走行の時刻 [s] | base進行 [m] | narrow train / validation | wide train / validation |',
    '|---|---:|---|---|']
for row in critical['matched_new_anchors']:
    lines.append(f"| {row['time_s']:.2f} | {row['base_s_m']:.2f} | {row['train']['narrow']} / {row['validation']['narrow']} | {row['train']['wide']} / {row['validation']['wide']} |")
lines += ['\nnarrow: 進行±3 m、横±0.15 m、向き±5度、速度±0.25 m/s。'
          'wide: ±5 m、±0.25 m、±10度、±0.4 m/s。'
          '同じ許容幅で比較したデータ分布の件数であり、新モデルの復帰成功率ではない。',
          '\n## 終了時の確認と残作業\n',
          f"AWSIM {host['awsim_files']}ファイルの内容と元repoのHEAD/statusは収集前と一致。"
          f"所有container/supervisorは終了し、実行側の空きは{host['free_gib']:.2f} GiB。"
          '12 runの台帳をsealし、確認済み転送対象だけを実行側から整理した。',
          f"\n採用イベントの最大制御sim間隔は{index['maximum_accepted_control_sim_gap_ms']:.3f} ms。"
          'pair02の同時走行25秒区間では、両方のsim/wall比が約0.999、全観測時点で両台が走行中だった。',
          '\n次の優先順位は、固定5 km/hの目標と収集用速度条件の整合性確認・速度追従の対処、'
          'C05_LATE/C07の単独確認とC11短縮経路の確認、C03/C06の地図を通せる準備経路、'
          '不足runの追加、採用済み教師と既存コーパスの統合、その後の再学習・E2E完走比較。'
          '今回の有限収集結果だけで、全11地点取得・復帰能力改善・E2E完走を達成したとはしない。',
          '\n詳細証拠: [実行証拠](evidence/time_corner_recovery_20260916/README.md)。\n']
path.write_text(text + '\n'.join(lines), encoding='utf-8')
print(path)
