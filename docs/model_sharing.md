# 学習環境・モデルの配布

学習コード、設定、ROS 2 package、検証結果はGitで管理する。
モデル本体はGoogle Driveで共有し、Gitには重みの識別情報だけを置く。
dataset、rosbag、動画、`.venv`、ROS build/installはGitに含めない。

## 現在のAWSIM実行モデル

機械可読の情報は [current_time_path.json](model_distribution/current_time_path.json)。
Google Driveへのアップロードはユーザーが実施する。共有URLは受領後に同JSONへ登録する。

|項目|値|
|---|---|
|モデル|提案モデルのTimePathV1、`launch_balanced` epoch 3|
|共有ファイル名|`epoch_03.pt`|
|ファイルサイズ|147,358,503 bytes（約147 MB）|
|SHA-256|`1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`|
|出力|0.1秒間隔の30点、将来3秒のXY waypoint|
|指令履歴|モデル入力として使用しない|
|モデル設定|checkpoint内に格納。配布JSONにも同じ設定を記録|

Windowsのファイル選択画面から指定する元ファイル:

```text
\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs\time_launch_protection_v2_20260916\launch_balanced\epoch_03.pt
```

AWSIM deploymentでは同一内容を `command_off_best.pt` という名前で配置する。
ファイル名を変更しても内容のSHAが一致していれば、現在のROS設定で読み込める。
checkpointはoptimizer等を含む学習checkpointであり、raw datasetは含まない。
学習の再開には同じデータ・split・教師manifestも別途必要。

ダウンロード後はチェックサムを確認する:

```powershell
Get-FileHash -Algorithm SHA256 -LiteralPath 'C:\Downloads\epoch_03.pt'
```

Linuxでは `sha256sum /path/to/epoch_03.pt` を使用する。
推論loaderも設定されたSHAを検査する。起動手順は [TimePath make dev](time_path_make_dev.md)。

これは現在AWSIM試験に使用しているcheckpointの配布である。
直近の20/10 km/h上限条件では1周133.67秒を確認した。
[学習時のオフライン採用判定](time_launch_protection_20260916.md)と、
[限定AWSIM試験](time_path_make_dev.md)の結果は別々に記録する。
過去の非劣化条件が未達だった事実を、共有によって合格へ変更しない。

## 学習環境の記録

2026-09-17、native WSLの稼働環境から56種類のdistributionの版を取得した。
自身のeditable packageを除く55件を
[requirements-training-wsl-20260917.txt](../requirements-training-wsl-20260917.txt)
へ記録した。詳細は [環境スナップショット](environment/training-wsl-20260917.json)。

- Ubuntu 22.04、Python 3.10.12、PyTorch 2.7.1+cu128、CUDA build 12.8。
- GPUはNVIDIA GeForce RTX 4080。学習・Linux検証はnative WSLで行う。
- Torch/torchvisionのCUDA wheelを含む、Linux向けの**観測バージョン一覧**。
  既存環境の更新や、新規環境への全依存再インストール試験は行っていない。
- 通常の依存宣言は `pyproject.toml` / `requirements.txt`、学習実行条件は
  [発進重点比較](time_launch_protection_20260916.md)と関連config/runnerを参照する。
- [Windows/WSL運用](windows_codex_wsl_training_workflow.md)に従い、Windowsで編集・commit・push、
  WSLでは `tools/with_wsl_training_lock.sh` を通して学習・評価する。

モデル配布準備時に、元checkpointのSHA照合と `TimeRuntimeModel` によるCPUロードを確認した。
実行コードは直近の全pytest **2935 passed / 4 skipped** とROS/AWSIM検証から変更していない。
