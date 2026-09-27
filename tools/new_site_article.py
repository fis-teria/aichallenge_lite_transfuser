"""Scaffold one article and register its metadata without overwriting existing files."""
from __future__ import annotations

import argparse
from datetime import date
import json
from pathlib import Path
import re

from build_project_site import ROOT, CATEGORIES, NAVIGATION


def create_article(root: Path, *, slug: str, title: str, summary: str, category: str,
                   updated: str, status: str, nav: str, topic: str = '') -> Path:
    if not re.fullmatch(r'[a-z0-9]+(?:-[a-z0-9]+)*',slug):
        raise ValueError('slug must use lowercase letters, digits, and hyphens')
    if not re.fullmatch(r'\d{4}-\d{2}-\d{2}',updated):
        raise ValueError('date must be YYYY-MM-DD')
    date.fromisoformat(updated)
    if category not in CATEGORIES or nav not in {row[0] for row in NAVIGATION}:
        raise ValueError('invalid category/navigation')
    if not all(value.strip() for value in (title,summary,status)):
        raise ValueError('title, summary and status must be nonempty')
    if category=='paper' and topic not in {'fusion','learning','control'}:
        raise ValueError('paper articles require --topic fusion, learning, or control')
    source=root/'docs/site_src'
    manifest=source/'articles.json'
    records=json.loads(manifest.read_text(encoding='utf-8'))
    path=source/'articles'/f'{slug}.html'
    if path.exists() or any(item['slug']==slug for item in records):
        raise ValueError('article already exists; edit it instead of overwriting')
    records.append(dict(slug=slug,title=title,category=category,nav=nav,summary=summary,
        published=updated,updated=updated,status=status,topic=topic,related=[]))
    body='''<!-- ARTICLE_DRAFT: Replace the prompts below with verified content, then remove this marker. -->
<h2>変更・調査の内容</h2><p>対象と、以前から何が変わったかを記載。</p>
<h2>確認したこと</h2><p>実行日、コード・モデル・設定、検証手順、観測した結果を記載。</p>
<h2>限界と次の確認</h2><p>未実施・未確認の範囲、残る課題を記載。</p>
<h2>根拠</h2><p>確認したcommitへ固定したコード・報告書、または一次資料へのリンクを追加。</p>
'''
    path.write_text(body,encoding='utf-8')
    manifest.write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return path


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('slug','title','summary','status'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--category',choices=list(CATEGORIES),required=True)
    parser.add_argument('--date',default=date.today().isoformat())
    parser.add_argument('--nav',choices=[row[0] for row in NAVIGATION])
    parser.add_argument('--topic',default='')
    args=parser.parse_args()
    nav=args.nav or {'guide':'architecture','report':'results','paper':'papers','development':'updates'}[args.category]
    path=create_article(ROOT,slug=args.slug,title=args.title,summary=args.summary,category=args.category,
        updated=args.date,status=args.status,nav=nav,topic=args.topic)
    print(f'ARTICLE_DRAFT_CREATED: {path}\nComplete the body and evidence before building the site.')


if __name__=='__main__':
    main()
