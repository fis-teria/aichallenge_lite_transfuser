"""Require an article update or explicit no-public-impact review for project changes."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
NOTE = '.github/site-update-note.json'


def relevant(path: str) -> bool:
    if path.startswith(('docs/site/', 'docs/site_src/')) or path == NOTE:
        return False
    return path.startswith(('src/', 'configs/', 'schemas/', 'ros2_ws/', 'tools/', 'docs/', 'tests/', '.github/workflows/')) or path in {
        'README.md', 'AGENTS.md', 'Makefile', 'pyproject.toml', 'requirements.txt',
    } or path.startswith('requirements-')


def validate_coverage(changed: list[str], bodies: dict[str, tuple[str, str]], note: dict | None) -> str:
    """Check meaningful body changes, not a generated page or timestamp-only edit."""
    affected = {path for path in changed if relevant(path)}
    if not affected:
        return 'no project-content change requiring an article review'
    for path, (before, after) in bodies.items():
        if (path in changed and path.startswith('docs/site_src/articles/') and path.endswith('.html')
                and after.strip() and re.sub(r'\s+', '', before) != re.sub(r'\s+', '', after)):
            return f'article updated: {path}'
    if NOTE in changed and isinstance(note, dict):
        paths = note.get('reviewed_paths')
        reason = note.get('reason')
        if (note.get('schema_version') == 1 and isinstance(paths, list)
                and all(isinstance(path, str) for path in paths) and set(paths) == affected
                and isinstance(reason, str) and len(reason.strip()) >= 20):
            return 'explicit no-public-impact review covers all affected paths'
    raise ValueError('SITE_UPDATE_REQUIRED: update a related article body, or change '+NOTE+
        ' with schema_version=1, exact reviewed_paths, and a specific reason (20+ characters).\n'+
        '\n'.join(sorted(affected)))


def git(root: Path, *args: str, allow_missing: bool = False) -> str:
    result = subprocess.run(['git', *args],cwd=root,capture_output=True,encoding='utf-8',check=False)
    if result.returncode and not allow_missing:
        raise ValueError(result.stderr.strip())
    return result.stdout if result.returncode == 0 else ''


def check_changes(root: Path, base: str, head: str) -> str:
    for revision in (base, head):
        git(root,'rev-parse','--verify','--end-of-options',revision+'^{commit}')
    changed = [path for path in git(root,'diff','--name-only','-z',base,head,'--').split('\0') if path]
    bodies = {path:(git(root,'show',f'{base}:{path}',allow_missing=True),git(root,'show',f'{head}:{path}',allow_missing=True))
        for path in changed if path.startswith('docs/site_src/articles/') and path.endswith('.html')}
    raw_note = git(root,'show',f'{head}:{NOTE}',allow_missing=True) if NOTE in changed else ''
    note = json.loads(raw_note) if raw_note else None
    return validate_coverage(changed,bodies,note)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', required=True)
    parser.add_argument('--head', default='HEAD')
    args = parser.parse_args()
    print('SITE_UPDATE_OK: '+check_changes(ROOT,args.base,args.head))


if __name__ == '__main__':
    main()
