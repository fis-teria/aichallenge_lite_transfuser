"""Verify an archive or materialize source-verified time train/val/test datasets."""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from aic_transfuser_lite.data.time_archive_v1 import verify_and_extract_time_archive
from aic_transfuser_lite.data.time_corpus_v1 import materialize_time_corpus


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    archive = sub.add_parser("extract")
    archive.add_argument("--archive", type=Path, required=True)
    archive.add_argument("--destination", type=Path, required=True)
    archive.add_argument("--archive-sha256", required=True)
    archive.add_argument("--manifest-sha256", required=True)
    archive.add_argument("--report", type=Path, required=True)
    corpus = sub.add_parser("materialize")
    corpus.add_argument("--raw-root", type=Path, required=True)
    corpus.add_argument("--destination", type=Path, required=True)
    corpus.add_argument("--split", type=Path, required=True)
    corpus.add_argument("--freeze-delay-ns", type=int, default=0)
    corpus.add_argument("--only-run")
    args = parser.parse_args()
    if args.command == "extract":
        if args.report.exists():
            raise FileExistsError(args.report)
        report = verify_and_extract_time_archive(args.archive, args.destination, args.archive_sha256,
                                                 args.manifest_sha256)
        with args.report.open("x", encoding="utf-8") as output:
            json.dump(report, output, indent=2)
        print(json.dumps(report), flush=True)
    else:
        report = materialize_time_corpus(args.raw_root, args.destination, args.split,
                                         freeze_delay_ns=args.freeze_delay_ns, only_run=args.only_run)
        print(json.dumps(report["splits"]), flush=True)


if __name__ == "__main__":
    main()
