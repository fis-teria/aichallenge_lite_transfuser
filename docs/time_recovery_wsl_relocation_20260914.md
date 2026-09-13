# 復帰収集データのWSL移動記録

2026-09-14 JST、ユーザーの「そのデータをWSL側に移す」指示により、
2026-09-13のAWSIM復帰収集20記録をWSLへ移動した。
以前の「移行済みデータは削除してよい」という指示も踏まえ、
WSLとの全件一致を確認後、AWSIM側の移動元を整理した。

## 完了結果

- 対象: 完走した復帰pilot 2本、復帰区間を含む途中終了run 2本、診断記録16本。
  **20本すべてが学習に使えるという意味ではない。** 採用区分と既存の教師再現結果は保持した。
- 実データ: 11,882,208,055 bytes（約11.07GiB）。ファイル・symlink計1,069項目。
- 全ファイルのSHA-256、サイズ、symlink先、ディレクトリ構造がAWSIM側とWSL側で一致。
  記録DBのSQLite quick_checkは全件 `ok`。r01は起動失敗でDBを持たない診断記録として保管。
- 未転送だったr01/r02/r09/r11は新たに圧縮・転送・展開し、既存の16記録と合わせて検証した。
- AWSIM側の空きは約9.95GiBから**21.02GiB**へ増加。
  移動先確認後の一時輸送archiveも削除した。
- AWSIM側には各runの `MOVED_TO_WSL.json` を残した。保存先を示し、同名runの再利用も防ぐ。
  実行source、参照経路、元repositoryのHEADと既存dirty 155件を保持。実行中containerは0。
- 移動後もWSLの20記録・元manifest・完走/部分復帰4本の圧縮版を確認した。
  学習用materialize・split・追加学習は実施していない。

## 現在の保存先

Distro: `Ubuntu-22.04-Recovered`。

| 内容 | WSL内の絶対パス |
|---|---|
| 原本・元ログ・run別metadata | `/home/thistle/e2e_autonomous/raw/time_recovery_collection_20260913/` |
| 圧縮版・監査結果・移動証跡 | `/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/` |

Windowsエクスプローラーからの原本パス:

```text
\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\raw\time_recovery_collection_20260913
```

正常完走runは `codex-time-recovery-right020-r19` と `codex-time-recovery-left020-r20`。
復帰の部分runは `codex-time-recovery-left020-r17` と `codex-time-recovery-left020-r18`。
これら4本の復帰候補227窓の検証結果は
[収集結果](time_recovery_collection_result_20260913.md) を参照する。

## 移動時の確認方法

AWSIM側で元transfer manifestと実ファイルを照合し、20run限定の新しいsnapshotを作成した。
WSLのnative filesystemへ未転送分を展開し、worktree lockを通したPython処理で
snapshotと全項目を比較、SQLiteをread-onlyで検査した。
元データの削除直前にもAWSIM側の全項目を再度照合し、全runが一致してから整理した。

一部の記録がcontainerのroot所有だったため、収集rootだけをbind mountした専用containerで整理した。
networkなし、root filesystemはread-only、capabilityは `DAC_OVERRIDE` のみ。
対象を固定した20run以外のpathを削除せず、各runをWSL保存先の案内へ置き換えた。
実行処理は [限定した整理処理](evidence/time_recovery_wsl_relocation_20260914/verified_relocation_cleanup_20260913.py)、
実行結果は [移動結果](evidence/time_recovery_wsl_relocation_20260914/wsl_relocation_result_20260913.json) に保存した。

保存済みの照合記録を表示するコマンド（WSL checkoutで実行）:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m json.tool \
  /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/wsl_relocation_verified_20260913.json
tools/with_wsl_training_lock.sh .venv/bin/python -m json.tool \
  /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/wsl_relocation_postcheck_20260914.json
```

証跡は [移動検証フォルダ](evidence/time_recovery_wsl_relocation_20260914/) に保存。
snapshot名の20260913は収集日の識別子で、実際の移動完了時刻は2026-09-14 00:04 JST。
raw bagや圧縮データはGitへ追加していない。
