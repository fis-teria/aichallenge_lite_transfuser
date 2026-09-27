"""Build the article site with the Python standard library; --check detects drift."""
from __future__ import annotations

import argparse
from datetime import date
from html import escape
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CATEGORIES = {"guide": "継続資料", "report": "検証記録", "paper": "論文ノート", "development": "開発・運用"}
NAVIGATION = [
    ("overview", "プロジェクト概要", "index.html"),
    ("architecture", "システムの仕組み", "articles/system-architecture.html"),
    ("results", "検証結果", "results.html"),
    ("stack", "技術スタック", "articles/technology-stack.html"),
    ("papers", "参考論文", "papers.html"),
    ("next", "今後の課題", "articles/roadmap.html"),
    ("updates", "記事一覧・更新履歴", "updates.html"),
]


def load_articles(source: Path) -> list[dict]:
    """Validate unique article identities, dates, relationships and source bodies."""
    records = json.loads((source / "articles.json").read_text(encoding="utf-8"))
    if not isinstance(records, list) or not records:
        raise ValueError("articles.json must be a nonempty list")
    slugs: set[str] = set()
    for item in records:
        if not isinstance(item, dict):
            raise ValueError("article metadata must be an object")
        for field in ("slug", "title", "summary", "category", "nav", "status", "published", "updated"):
            if not isinstance(item.get(field), str) or not item[field].strip():
                raise ValueError(f"article field missing: {field}")
        slug = item["slug"]
        if not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", slug) or slug in slugs:
            raise ValueError(f"invalid or duplicate slug: {slug}")
        slugs.add(slug)
        if item["category"] not in CATEGORIES or item["nav"] not in {row[0] for row in NAVIGATION}:
            raise ValueError(f"invalid category/navigation: {slug}")
        for field in ("published", "updated"):
            if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", item[field]):
                raise ValueError(f"invalid {field}: {slug}")
        if date.fromisoformat(item["updated"]) < date.fromisoformat(item["published"]):
            raise ValueError(f"updated before published: {slug}")
        if not isinstance(item.get("related"), list) or any(not isinstance(value, str) for value in item["related"]):
            raise ValueError(f"invalid related list: {slug}")
        if item["category"] == "paper" and item.get("topic") not in {"fusion", "learning", "control"}:
            raise ValueError(f"invalid paper topic: {slug}")
        body_path = source / "articles" / f"{slug}.html"
        if not body_path.is_file():
            raise ValueError(f"missing article body: {slug}")
        body = body_path.read_text(encoding="utf-8")
        if not body.strip() or "ARTICLE_DRAFT" in body:
            raise ValueError(f"unfinished article: {slug}")
        if re.search(r"<(?:h1|script|iframe)\b", body, re.I):
            raise ValueError(f"article body must not contain h1/script/iframe: {slug}")
    for item in records:
        if any(slug not in slugs or slug == item["slug"] for slug in item["related"]):
            raise ValueError(f"unknown/self related article: {item['slug']}")
    actual = {path.stem for path in (source / "articles").glob("*.html")}
    if actual != slugs:
        raise ValueError(f"unregistered article bodies: {sorted(actual - slugs)}")
    return records


def substitute(template: str, values: dict[str, str]) -> str:
    """Substitute explicit template markers; CSS/HTML braces are left untouched."""
    return re.sub(r"\{\{([a-z_]+)\}\}", lambda match: values[match[1]], template)


def card(article: dict, prefix: str = "", filter_by_topic: bool = False) -> str:
    category = article.get("topic") if filter_by_topic else article["category"]
    return f'''<article class="article-card" data-category="{escape(category)}">
<div class="card-meta"><span class="tag">{CATEGORIES[article['category']]}</span>
<time datetime="{article['updated']}">{article['updated']}</time></div>
<h3><a href="{prefix}articles/{article['slug']}.html">{escape(article['title'])}</a></h3>
<p>{escape(article['summary'])}</p><div class="card-bottom"><span>{escape(article['status'])}</span>
<a class="article-read" href="{prefix}articles/{article['slug']}.html" aria-label="{escape(article['title'])}を読む">記事を読む →</a></div></article>'''


def search_controls(papers: bool = False) -> str:
    categories = {"fusion": "センサ融合", "learning": "学習", "control": "制御・SLAM"} if papers else CATEGORIES
    buttons = '<button type="button" data-filter="all" aria-pressed="true">すべて</button>'
    buttons += ''.join(f'<button type="button" data-filter="{key}" aria-pressed="false">{label}</button>' for key, label in categories.items())
    return f'''<div class="paper-tools" hidden><label class="search-label" for="article-search">記事を検索
<input id="article-search" type="search" placeholder="例：復帰、DINOv3、LiDAR" autocomplete="off"></label>
<div class="filters" role="group" aria-label="記事の種類">{buttons}</div></div>
<p class="search-status small" aria-live="polite" hidden></p>'''


def build_outputs(root: Path = ROOT) -> dict[str, bytes]:
    source = root / "docs/site_src"
    records = load_articles(source)
    by_slug = {item["slug"]: item for item in records}
    priority = {"development": 3, "report": 2, "guide": 1, "paper": 0}
    ordered = sorted(records, key=lambda item: (item["updated"], priority[item["category"]], records.index(item)), reverse=True)
    layout = (source / "layout.html").read_text(encoding="utf-8")
    outputs: dict[str, bytes] = {}

    def render(path: str, title: str, description: str, nav: str, content: str, page: str) -> None:
        prefix = "../" if path.startswith("articles/") else ""
        navigation = '<nav aria-label="サイト内ナビゲーション">'
        for number, (key, label, target) in enumerate(NAVIGATION, 1):
            current = ' aria-current="page"' if key == nav else ''
            navigation += f'<a href="{prefix}{target}"{current}><span>{number:02d}</span>{label}</a>'
        navigation += '</nav>'
        result = substitute(layout, dict(title=escape(title), description=escape(description),
            root=prefix, navigation=navigation, content=content, page=page))
        result = substitute(result, {"root": prefix})
        outputs[path] = (result.rstrip() + "\n").encode("utf-8")

    topics = ''.join(f'<a id="{key}" href="{target}"><span>{number:02d} / EXPLORE</span><strong>{label} →</strong></a>'
        for number, (key, label, target) in enumerate(NAVIGATION[1:], 1))
    home = substitute((source / "home.html").read_text(encoding="utf-8"), {
        "topic_cards": '<div class="resource-grid">'+topics+'</div>',
        "latest_articles": '<div class="article-grid">'+''.join(card(item) for item in ordered[:4])+'</div>',
    })
    last_updated = ordered[0]["updated"]
    home = re.sub(r'<time datetime="[^"]+">[^<]+</time>', f'<time datetime="{last_updated}">{last_updated.replace("-", ".")}</time>', home, count=1)
    render('index.html', 'Research Notes', '現状、構成、検証記録、技術スタック、論文ノートへの入口。', 'overview', home, 'home')

    for name, nav, title, description, subset, searchable in [
        ('updates.html', 'updates', '記事一覧・更新履歴', '記事の公開・更新日順に掲載しています。試験の実施日は各記事に記録。継続資料、検証記録、論文ノート、開発・運用の記事を横断して探せます。', ordered, True),
        ('papers.html', 'papers', '参考論文ノート', '1本の論文を1つの記事に。一次資料の要約と、本プロジェクトへの対応づけ・限界を分けて読めます。', [item for item in records if item['category']=='paper'], True),
        ('results.html', 'results', '検証結果と記録', '異なるコード・設定・試験日の結果を個別に記録。単発の成功、課題の確認、実装テスト、未検証を区別します。', [item for item in records if item['category']=='report'], False),
    ]:
        content = f'<section class="section first listing"><div class="section-heading"><span class="section-number">PROJECT LIBRARY</span><h1>{title}</h1><p>{description}</p></div>'
        if searchable:
            content += search_controls(papers=name=='papers.html')
        content += '<div class="article-grid">'+''.join(card(item, filter_by_topic=name=='papers.html') for item in subset)+'</div>'
        content += '<p id="article-empty" class="note" hidden>一致する記事はありません。検索語や分類を変更してください。</p></section>'
        render(name,title,description,nav,content,'listing')

    for item in records:
        body = (source / 'articles' / f"{item['slug']}.html").read_text(encoding='utf-8')
        parent = 'papers.html' if item['category']=='paper' else 'results.html' if item['category']=='report' else 'updates.html'
        content = f'''<article class="article-page"><header class="article-heading">
<div class="breadcrumbs"><a href="../index.html">ホーム</a><span>/</span><a href="../{parent}">{CATEGORIES[item['category']]}</a></div>
<span class="section-number">{CATEGORIES[item['category']]}</span><h1>{escape(item['title'])}</h1>
<p class="article-lead">{escape(item['summary'])}</p><div class="article-meta">
<span>公開 <time datetime="{item['published']}">{item['published']}</time></span>
<span>更新 <time datetime="{item['updated']}">{item['updated']}</time></span>
<span class="tag">{escape(item['status'])}</span></div></header><div class="article-body">{body}</div>'''
        if item['related']:
            content += '<section class="related-articles"><h2>関連記事</h2><div class="article-grid">'+''.join(card(by_slug[slug],prefix='../') for slug in item['related'])+'</div></section>'
        content += '</article>'
        render(f"articles/{item['slug']}.html",item['title'],item['summary'],item['nav'],content,'article')
    for asset in (source/'assets').iterdir():
        if asset.is_file():
            outputs[f'assets/{asset.name}'] = asset.read_bytes()
    return outputs


def build(root: Path = ROOT, check: bool = False) -> list[str]:
    outputs = build_outputs(root)
    destination = root/'docs/site'
    extra = [str(path.relative_to(destination)) for path in destination.rglob('*.html') if str(path.relative_to(destination)).replace('\\','/') not in outputs]
    if extra:
        raise ValueError(f"Unregistered published pages; preserve or explicitly migrate their URLs: {extra}")
    changed = []
    for relative, data in outputs.items():
        target = destination/relative
        if not target.is_file() or target.read_bytes() != data:
            changed.append(relative)
            if not check:
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(data)
    return changed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    changed = build(check=args.check)
    if args.check and changed:
        raise SystemExit('SITE_BUILD_STALE: run python tools/build_project_site.py\n'+'\n'.join(changed))
    print(f"SITE_BUILD_OK: {'verified' if args.check else 'generated'}, {len(changed)} changed files")


if __name__ == '__main__':
    main()
