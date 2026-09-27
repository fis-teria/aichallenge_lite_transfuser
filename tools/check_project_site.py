"""Validate every page, local fragment and pinned evidence link in the article site."""
from __future__ import annotations
from html.parser import HTMLParser
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'https://github.com/fis-teria/aichallenge_lite_transfuser'


class SiteParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.ids: set[str] = set()
        self.references: list[tuple[str,str]] = []
        self.sources: list[tuple[str,str]] = []
        self.errors: list[str] = []
        self.headings = 0

    def handle_starttag(self, tag: str, attrs: list[tuple[str,str | None]]) -> None:
        values=dict(attrs)
        identifier=values.get('id')
        if identifier:
            if identifier in self.ids: self.errors.append(f'Duplicate ID: {identifier}')
            self.ids.add(identifier)
        for key in ('href','src'):
            if values.get(key): self.references.append((tag,str(values[key])))
        if values.get('data-source'):
            self.sources.append((str(values['data-source']),str(values.get('href',''))))
        if tag=='img' and 'alt' not in values: self.errors.append('Image missing alt')
        if tag=='h1': self.headings+=1


def validate_site(root: Path = ROOT, verify_sources: bool = True) -> tuple[list[str], dict[str,int]]:
    site=(root/'docs/site').resolve()
    pages: dict[Path,SiteParser] = {}
    errors=[]
    sources:set[tuple[str,str]]=set()
    for path in site.rglob('*.html'):
        parser=SiteParser()
        parser.feed(path.read_text(encoding='utf-8'))
        pages[path.resolve()]=parser
        errors.extend(f'{path.name}: {error}' for error in parser.errors)
        if parser.headings!=1: errors.append(f'{path.name}: expected one h1, got {parser.headings}')
        sources.update(parser.sources)
    if not pages: errors.append('No HTML pages found')
    for path,parser in pages.items():
        for tag,reference in parser.references:
            url=urlsplit(reference)
            if url.scheme or url.netloc:
                if url.scheme!='https': errors.append(f'Non-HTTPS link: {reference}')
                if tag in {'script','link','img','iframe'}: errors.append(f'External runtime dependency: {reference}')
                continue
            target=(path.parent/unquote(url.path)).resolve() if url.path else path
            if not target.is_relative_to(site) or not target.is_file():
                errors.append(f'{path.name}: missing/out-of-site reference: {reference}')
                continue
            if url.fragment and (target not in pages or unquote(url.fragment) not in pages[target].ids):
                errors.append(f'{path.name}: unknown fragment: {reference}')
    for source,href in sources:
        match=re.fullmatch(re.escape(REPOSITORY)+r'/blob/([0-9a-f]{40})/(.+)',href)
        if match is None or unquote(match[2])!=source:
            errors.append(f'Evidence link must use a full commit and matching path: {href}')
        elif verify_sources:
            result=subprocess.run(['git','cat-file','-e',f'{match[1]}:{source}'],cwd=root,capture_output=True,check=False)
            if result.returncode: errors.append(f'Missing pinned evidence: {source}')
    for path in site.rglob('*'):
        if path.is_file() and path.suffix not in {'.html','.css','.js','.svg','.md'}:
            errors.append(f'Unexpected published asset: {path.relative_to(site)}')
    return errors,dict(pages=len(pages),references=sum(len(parser.references) for parser in pages.values()),sources=len(sources))


def main() -> None:
    errors,stats=validate_site()
    if errors: raise SystemExit('\n'.join(errors))
    print(f"SITE_OK: {stats['pages']} pages, {stats['references']} links/assets, {stats['sources']} pinned sources")


if __name__=='__main__': main()
