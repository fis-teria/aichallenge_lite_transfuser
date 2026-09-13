# AWSIM実行先の容量整理候補 — 2026-09-13

対象は `graneple@192.168.3.10`。復帰教師の収集に先立つ容量確認。
初回調査では削除せず、具体的なキャッシュ5対象を提示して確認した。
その後、ユーザーの「データ移した後の学習データとかも…消してよい」という追加指示を受け、
提示済みキャッシュと、WSLへの移行が確認できた旧学習データの重複分を清掃した。

## 実施結果

一般ユーザー向け空きは **約8.84 GiB → 約22.17 GiB**、使用率は **97% → 91%**。
合計で約13.33 GiBを解放した。以下は実測で、他processの微小な書込みを含む。

| 処理 | 実測解放bytes | 約GiB |
|---|---:|---:|
| 提示済み5対象のキャッシュ清掃 | 7,172,104,192 | 6.68 |
| WSL移行済みの旧復帰データ2コピー削除 | 7,140,356,096 | 6.65 |

最終確認時の空きは23,802,335,232 bytes。`df -h` は切上げ表示で23Gとなる。
単位換算した値は22.17 GiB。

削除した旧データは、次の2ディレクトリの中身。各15周・210ファイルで、
削除前に全ファイルの相対パス、サイズ、SHA-256をWSLの保存先と照合した。

```text
/home/graneple/v3_runtime/m3_epoch3_44670a4/teacher_recovery_v3/38f5c67/recovery_remaining_20260904/accepted
/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/38f5c67/recovery_remaining_20260904/accepted
```

保全した原本は次のWSL native directory。

```text
/home/thistle/e2e_autonomous/raw/teacher_recovery_v3_20260904/accepted
```

- 削除前：210ファイル × 2コピーで、欠損・サイズ不一致・SHA-256不一致は0件。
- 削除後：420ファイルが`.10`から除去されたことを確認。
- 保全先：WSLの15周分210ファイルと付随report6ファイル、計216ファイルを再度SHA-256照合して一致。
- `.10`に残したその他555ファイルも全SHA-256一致、4 symlinkも変更なし。
- Dockerは既存114コンテナを保持、起動中0件。既存image/containerの清掃は行っていない。

元の各`accepted` directoryは空のまま残し、その親に
`accepted.moved_to_wsl_20260913.json`を保存した。保存先、全ファイルのhash、完了状態を記録。
未転送の試行データ、モデル、現在の時間教師、AWSIM実行物は今回の削除対象ではない。

root所有のbag内45ファイルは、既存`aichallenge-2025-dev:latest`を使い、当該`accepted`
1ディレクトリだけを書込み可能にマウントした使い捨てhelperで除去した。
`--network=none --read-only --cap-drop=ALL --security-opt=no-new-privileges`、GPUなし、
PID/memory/CPU制限付き。ホストの権限・所有者は変更していない。
残り375ファイルはホストユーザーで、許可済みの個別パスと直前のinode/size/mtimeを
照合してunlinkした。いずれも一括pruneや無条件の再帰削除は使っていない。

キャッシュはpipの管理コマンドで105ファイル、旧pipが扱えない`http-v2`を含む
残り117,664ファイルを検査済みの個別パスで除去した。5対象は清掃直後に空であることを確認。

実行・確認結果は `docs/evidence/pc10_storage_cleanup_20260913/` に保存。
保全確認のWSL処理は `tools/with_wsl_training_lock.sh` を通した。
source/config/modelは変更せず、学習・AWSIM試行・新たなpytestは実施していない。

## 実測容量と最初の削除候補

root `/dev/nvme0n1p8` は総容量242,469,183,488 bytes（225.82 GiB）。
候補精査時の一般ユーザー向け空きは9,490,022,400 bytes（約8.84 GiB）、使用率97%。
root以外の通常の保存用ドライブは未マウント。inode使用率16%で、問題は保存容量。

以下の5ディレクトリ内の再生成・再取得できるキャッシュを第1段階の候補とする。
GiBは2^30 bytes、容量はファイルの論理サイズではなく割当済みブロックで集計。

| 削除候補の絶対パス | 割当済みbytes | 約GiB | 影響 |
|---|---:|---:|---|
| `/home/graneple/.cache/pip` | 792,449,024 | 0.74 | 次回の依存導入で再ダウンロードが必要になる |
| `/home/graneple/.cache/vscode-cpptools` | 2,314,072,064 | 2.16 | C/C++解析キャッシュの再生成に時間がかかる |
| `/home/graneple/.config/Code/CachedExtensionVSIXs` | 1,634,013,184 | 1.52 | 拡張機能の更新・再導入で再ダウンロードが必要になる |
| `/home/graneple/.cache/google-chrome` | 1,215,197,184 | 1.13 | Web閲覧時にキャッシュを再取得する |
| `/home/graneple/.cache/BraveSoftware` | 1,216,393,216 | 1.13 | Web閲覧時にキャッシュを再取得する |
| **合計** | **7,172,124,672** | **6.68** | 空きは約15.5 GiBになる見込み |

全5対象について、resolve後のパス一致、対象内symlinkなし、外部hardlinkなし、
special fileなし、読み取りエラーなしを確認した。
調査時に閲覧可能な同一ユーザーの `/proc/*/{cwd,exe,fd,maps}` には、これらの対象への
使用中参照は見つからなかった。一部の保護されたprocessは参照検査できていない。
実行時にはアプリの起動状態と対象パスを再確認する。

`python3 -m pip` は22.0.2で、`pip cache info` の約149.8 MBという表示は
約640.5 MB割当の `http-v2` を含まない。古いpipの表示だけで清掃完了と判定しない。
実際に清掃するときは対応する管理コマンドを優先し、管理コマンドが扱わない部分を
削除する場合も上記で許可された厳密な対象内に限定し、完了後に `du` と `df` を測る。
`rm -rf`、Docker prune、ホーム全体や `.cache` 全体の一括削除は使わない。

## 初回のキャッシュ候補から除外したもの

- 教師bag、学習データ、重み、実験結果、source/build/install、AWSIM本体、Docker。
  後続指示で追加したWSL移行済みの旧データ2コピーについては、上の実施結果を参照。
- ブラウザの `.config` 側profile、VS Codeの `User`、`WebStorage`、導入済み拡張機能。
- `.cache/huggingface`、`.cache/ms-playwright`：モデル・実行用browserを含む。
- `.cache/uv`：表示約2.30 GiBのうち約2.01 GiBは対象外へのhardlinkで共有。
  このディレクトリだけを消しても大半の容量は解放されない。uvの実行も確認した。
- Downloads、ごみ箱、別プロジェクト、会話記録：個別の用途・保全確認が必要。

Dockerは起動中0件。114個の既存コンテナ、12個のimageが残っている。
`docker system df` のimage回収可能量に130%という共有計算上の不整合があり、
表示値の単純合計を解放見込みには用いない。既存image/containerの個別保全監査は未実施。
`aichallenge-2025-dev:latest` は今後の実行に必要であり削除対象にしない。

## 確認に使ったコマンド

WindowsからSSHで実施。大きい出力はPythonで上位項目に絞った。

```text
ssh -o BatchMode=yes -o ConnectTimeout=8 graneple@192.168.3.10
df -h /
df -B1 /
df -i /
findmnt
du -x -B1 --max-depth=1 /home/graneple
du -x -B1 --max-depth=1 /home/graneple/.cache
du -x -B1 --max-depth=1 /home/graneple/.config/Code
du -x -B1 --max-depth=1 /home/graneple/e2e_autonomous
du -x -B1 --max-depth=1 /home/graneple/git/autononous_ai/aichallenge-racingkart/output
python3 -m pip --version
python3 -m pip cache info
/home/graneple/.local/bin/uv --version
/home/graneple/.local/bin/uv cache dir
docker ps
docker system df
docker system df -v
```

Git配下とシステム領域の一部は権限により `du` で読めないため、
それらの大分類の集計は全量保証ではない。上記の5候補は全量を読めた。

## 削除の確認が必要な根拠

現地 `/home/graneple/git/autononous_ai/AGENTS.md` が参照する
`/home/graneple/git/autononous_ai/docs/full_access_execution_harness.md` の75〜79行：

> Do not execute until the user explicitly authorizes the exact target and
> action:
> - Delete, purge, prune, clean, overwrite, or bulk-move user files,
>   artifacts, bags, branches, tags, volumes, images, or databases.

このため、初回は上記5対象のキャッシュ削除を具体的に提示して確認した。
これは現地の明示ルールに基づく確認であり、一般的な容量整理への追加の推測条件ではない。
後続のユーザー指示を受け、同じ確認を繰り返さず、清掃と空き容量の再計測を完了した。

この文書以外にsource/configを変更していない。学習・AWSIM試行・新たなpytestは未実施。
