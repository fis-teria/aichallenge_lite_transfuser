"""Check responsive article navigation, catalog search, drawer focus and legacy URLs."""
from __future__ import annotations
import argparse
from pathlib import Path
from unicodedata import normalize
from urllib.parse import urljoin
from playwright.sync_api import sync_playwright
from build_project_site import load_articles

ROOT=Path(__file__).resolve().parents[1]


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default=(ROOT/'docs/site/index.html').as_uri())
    parser.add_argument('--channel',default=None)
    parser.add_argument('--screenshots',type=Path)
    args=parser.parse_args()
    base=args.url
    articles=load_articles(ROOT/'docs/site_src')
    papers=[article for article in articles if article['category']=='paper']
    control_count=sum(article['topic']=='control' for article in papers)
    report_count=sum(article['category']=='report' for article in articles)
    dino_count=sum('dinov3' in normalize('NFKC', ' '.join(article[key] for key in ('title','summary','status'))).lower() for article in papers)
    errors:list[str]=[]
    requests:list[str]=[]
    if args.screenshots: args.screenshots.mkdir(parents=True,exist_ok=True)
    pages=sorted(path.relative_to(ROOT/'docs/site').as_posix() for path in (ROOT/'docs/site').rglob('*.html'))
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(channel=args.channel,headless=True)
        context=browser.new_context(reduced_motion='reduce')
        page=context.new_page()
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.on('console',lambda message:errors.append(message.text) if message.type=='error' else None)
        page.on('request',lambda request:requests.append(request.url))
        page.on('requestfailed',lambda request:errors.append(f'Request failed: {request.url}'))
        page.on('response',lambda response:errors.append(f'HTTP {response.status}: {response.url}') if response.status>=400 else None)

        def open_menu(width: int) -> None:
            if width<=900: page.locator('.menu-toggle').click()

        for width,height in ((1440,1000),(1024,900),(768,1024),(390,844),(320,740)):
            page.set_viewport_size(dict(width=width,height=height))
            page.goto(base,wait_until='networkidle')
            assert page.locator('h1').count()==1
            assert page.locator('nav a[aria-current=page]').count()==1
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 1'),width
            if width<=900:
                assert page.locator('.sidebar').is_hidden()
                open_menu(width)
                assert page.locator('.sidebar[role=dialog]').is_visible()
                assert page.locator('main').evaluate('element => element.inert')
                positions=page.locator('nav a').evaluate_all('links => links.map(link => ({x:link.getBoundingClientRect().x,y:link.getBoundingClientRect().y}))')
                assert len({position['x'] for position in positions})==1
                assert positions[0]['y']<positions[-1]['y']
                page.keyboard.press('Shift+Tab')
                assert page.locator('.sidebar a').last.evaluate('element => element === document.activeElement')
                page.keyboard.press('Tab')
                assert page.locator('.menu-close').evaluate('element => element === document.activeElement')
                page.keyboard.press('Escape')
                assert page.locator('.menu-toggle').evaluate('element => element === document.activeElement')
                open_menu(width)
                page.locator('.menu-backdrop').click(position=dict(x=width-10,y=100))
                assert page.locator('.sidebar').is_hidden()
                open_menu(width)
                page.locator('.menu-close').click()
                assert not page.locator('main').evaluate('element => element.inert')
                open_menu(width)
                page.set_viewport_size(dict(width=1200,height=height))
                assert page.locator('.sidebar').is_visible()
                assert page.locator('.menu-backdrop').is_hidden()
                assert not page.locator('main').evaluate('element => element.inert')
                page.set_viewport_size(dict(width=width,height=height))
                assert page.locator('.sidebar').is_hidden()
            else:
                assert page.locator('.sidebar').is_visible()
                assert page.locator('.menu-toggle').is_hidden()
            if args.screenshots: page.screenshot(path=str(args.screenshots/f'home-{width}.png'))
            open_menu(width)
            page.locator('nav a[href="papers.html"]').click()
            page.wait_for_url('**/papers.html')
            assert page.locator('.article-card:visible').count()==len(papers)
            page.locator('[data-filter="control"]').click()
            assert page.locator('.article-card:visible').count()==control_count
            page.locator('[data-filter="all"]').click()
            page.locator('#article-search').fill('ＤＩＮＯｖ３')
            assert page.locator('.article-card:visible').count()==dino_count
            page.locator('#article-search').fill('no_such_article_123')
            assert page.locator('.article-card:visible').count()==0
            assert page.locator('#article-empty').is_visible()
            page.locator('#article-search').fill('DINOv3')
            page.locator('.article-card:visible .article-read[href="articles/dinov3.html"]').click()
            page.wait_for_url('**/articles/dinov3.html')
            assert 'DINOv3' in page.locator('h1').inner_text()
            summary=page.locator('.article-body summary')
            summary.focus()
            page.keyboard.press('Enter')
            assert page.locator('.article-body details').get_attribute('open') is None
            page.keyboard.press('Enter')
            assert page.locator('.article-body details').get_attribute('open') is not None
            assert page.locator('.related-articles a.article-read').count()>0
            open_menu(width)
            page.locator('nav a[href="../updates.html"]').click()
            page.wait_for_url('**/updates.html')
            assert page.locator('.article-card:visible').count()==len(articles)
            page.locator('[data-filter="report"]').click()
            assert page.locator('.article-card:visible').count()==report_count
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 1'),width
            print(f'BROWSER_OK: {width}px, sidebar/keyboard/resize/catalog/article navigation',flush=True)

        for width,height in ((1440,1000),(390,844)):
            page.set_viewport_size(dict(width=width,height=height))
            for relative in pages:
                page.goto(urljoin(base,relative),wait_until='networkidle')
                assert page.locator('h1').count()==1,relative
                assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 1'),(width,relative)
                assert page.locator('nav a[aria-current=page]').count()==1,relative
                if args.screenshots and relative in {'updates.html','articles/system-architecture.html','articles/dinov3.html'}:
                    page.screenshot(path=str(args.screenshots/f'{Path(relative).stem}-{width}.png'),full_page=True)
            print(f'BROWSER_OK: every published page at {width}px',flush=True)

        for fragment,target in {'stack':'articles/technology-stack.html','architecture':'articles/system-architecture.html','results':'results.html','papers':'papers.html','next':'articles/roadmap.html'}.items():
            page.goto(base.split('#')[0]+'#'+fragment,wait_until='networkidle')
            page.wait_for_url('**/'+target)
        fallback_context=browser.new_context(java_script_enabled=False,viewport=dict(width=390,height=844))
        fallback=fallback_context.new_page()
        fallback.goto(urljoin(base,'papers.html'),wait_until='networkidle')
        assert fallback.locator('.article-card:visible').count()==len(papers)
        assert fallback.locator('.paper-tools').is_hidden()
        fallback.locator('.article-read').first.click()
        assert fallback.locator('.article-body').is_visible()
        assert fallback.locator('h1').count()==1
        if base.startswith('file:'): assert all(request.startswith('file:') for request in requests),requests
        assert not errors,errors
        browser.close()
    print('BROWSER_OK: legacy links, JS-disabled article navigation, no missing resources or script errors')


if __name__=='__main__': main()
