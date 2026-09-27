"""Browser smoke checks; install Playwright separately from model dependencies."""
from __future__ import annotations

import argparse
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=(ROOT / "docs/site/index.html").as_uri())
    parser.add_argument("--channel", default=None, help="For example: msedge")
    parser.add_argument("--screenshots", type=Path)
    args = parser.parse_args()
    errors: list[str] = []
    requests: list[str] = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(channel=args.channel, headless=True)
        context = browser.new_context(reduced_motion="reduce")
        page = context.new_page()
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.on("console", lambda message: errors.append(message.text) if message.type == "error" else None)
        page.on("request", lambda request: requests.append(request.url))
        page.on("requestfailed", lambda request: errors.append(f"Request failed: {request.url}"))
        page.on("response", lambda response: errors.append(f"HTTP {response.status}: {response.url}") if response.status >= 400 else None)
        for width, height in ((1440, 1000), (1024, 900), (768, 1024), (390, 844), (320, 740)):
            page.set_viewport_size({"width": width, "height": height})
            page.goto(args.url, wait_until="networkidle")
            assert page.title().startswith("AIC TransFuser Lite")
            assert page.locator(".paper:visible").count() == 6
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth + 1"), f"Horizontal overflow at {width}px"
            page.locator('[data-filter="control"]').click()
            assert page.locator(".paper:visible").count() == 2
            page.locator('[data-filter="all"]').click()
            page.locator("#paper-search").fill("ＤＩＮＯｖ３")
            assert page.locator(".paper:visible").count() == 1
            assert page.locator(".paper:visible h3").inner_text() == "DINOv3"
            page.locator("#paper-search").fill("no_such_paper_123")
            assert page.locator(".paper:visible").count() == 0
            assert page.locator("#paper-empty").is_visible()
            page.locator("#paper-search").fill("")
            assert page.locator(".paper:visible").count() == 6
            summary = page.locator(".paper summary").first
            summary.focus()
            page.keyboard.press("Enter")
            assert page.locator(".paper details").first.get_attribute("open") is not None
            page.keyboard.press("Enter")
            for target in ("#architecture", "#results", "#stack", "#papers", "#next", "#overview"):
                page.locator(f'nav a[href="{target}"]').click()
                page.wait_for_function("hash => location.hash === hash", arg=target)
                page.wait_for_function("hash => document.querySelector('nav a[aria-current]')?.hash === hash", arg=target)
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth + 1")
            if args.screenshots:
                args.screenshots.mkdir(parents=True, exist_ok=True)
                page.screenshot(path=str(args.screenshots / f"site-{width}.png"), full_page=True)
            print(f"BROWSER_OK: {width}x{height}, filters/search/empty state/keyboard/navigation")
        offline = browser.new_context(java_script_enabled=False, viewport={"width": 390, "height": 844})
        fallback = offline.new_page()
        fallback.goto(args.url, wait_until="networkidle")
        assert fallback.locator(".paper:visible").count() == 6
        assert fallback.locator(".paper-tools").is_hidden()
        fallback.locator(".paper summary").first.click()
        assert fallback.locator(".paper details").first.get_attribute("open") is not None
        if args.url.startswith("file:"):
            assert all(request.startswith("file:") for request in requests), requests
        assert not errors, errors
        browser.close()
    print("BROWSER_OK: no JavaScript errors, no missing resources, readable without JavaScript")


if __name__ == "__main__":
    main()
