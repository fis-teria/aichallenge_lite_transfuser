"""Validate the standalone project site without third-party dependencies."""
from __future__ import annotations

from html.parser import HTMLParser
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "docs" / "site"
REPOSITORY = "https://github.com/fis-teria/aichallenge_lite_transfuser"


class SiteParser(HTMLParser):
    """Collect IDs, references, and explicit evidence links from HTML."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.ids: set[str] = set()
        self.references: list[tuple[str, str]] = []
        self.sources: list[tuple[str, str]] = []
        self.errors: list[str] = []
        self.headings = 0

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        values = dict(attrs)
        identifier = values.get("id")
        if identifier:
            if identifier in self.ids:
                self.errors.append(f"Duplicate ID: {identifier}")
            self.ids.add(identifier)
        for key in ("href", "src"):
            if values.get(key):
                self.references.append((tag, str(values[key])))
        if values.get("data-source"):
            self.sources.append((str(values["data-source"]), str(values.get("href", ""))))
        if tag == "img" and "alt" not in values:
            self.errors.append("Image missing alt attribute")
        if tag == "h1":
            self.headings += 1


def main() -> None:
    parser = SiteParser()
    parser.feed((SITE / "index.html").read_text(encoding="utf-8"))
    errors = parser.errors
    if parser.headings != 1:
        errors.append(f"Expected one h1, got {parser.headings}")
    for tag, reference in parser.references:
        url = urlsplit(reference)
        if url.scheme or url.netloc:
            if url.scheme != "https":
                errors.append(f"Non-HTTPS external link: {reference}")
            if tag in {"script", "link", "img", "iframe"}:
                errors.append(f"External runtime dependency: {reference}")
            continue
        if not url.path:
            if unquote(url.fragment) not in parser.ids:
                errors.append(f"Unknown anchor: {reference}")
        else:
            target = (SITE / unquote(url.path)).resolve()
            if not target.is_relative_to(SITE.resolve()) or not target.is_file():
                errors.append(f"Missing or out-of-site asset: {reference}")
    for path, href in set(parser.sources):
        match = re.fullmatch(re.escape(REPOSITORY) + r"/blob/([0-9a-f]{40})/(.+)", href)
        if match is None or unquote(match[2]) != path:
            errors.append(f"Evidence link is not pinned to its path and full commit: {href}")
            continue
        result = subprocess.run(
            ["git", "cat-file", "-e", f"{match[1]}:{path}"],
            cwd=ROOT, capture_output=True, check=False,
        )
        if result.returncode:
            errors.append(f"Missing evidence at pinned commit: {path}")
    for file in SITE.rglob("*"):
        if file.is_file() and file.suffix not in {".html", ".css", ".js", ".svg", ".md"}:
            errors.append(f"Unexpected published asset: {file.relative_to(SITE)}")
    if errors:
        raise SystemExit("\n".join(errors))
    total_bytes = sum(file.stat().st_size for file in SITE.rglob("*") if file.is_file())
    print(f"SITE_OK: {len(parser.ids)} IDs, {len(parser.references)} links/assets, "
          f"{len(set(parser.sources))} pinned sources, {total_bytes:,} bytes")


if __name__ == "__main__":
    main()
