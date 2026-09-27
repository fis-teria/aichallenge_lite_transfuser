"""Regression tests for publication drift, broken article links and update omissions."""
from __future__ import annotations
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from build_project_site import build, build_outputs, load_articles
from check_project_site import validate_site
from check_site_update import NOTE, validate_coverage
from new_site_article import create_article


class ArticleBuildTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.source=self.root/'docs/site_src'
        shutil.copytree(ROOT/'docs/site_src',self.source)

    def metadata(self, change) -> None:
        path=self.source/'articles.json'
        records=json.loads(path.read_text(encoding='utf-8'))
        change(records)
        path.write_text(json.dumps(records,ensure_ascii=False),encoding='utf-8')

    def test_build_is_deterministic_and_matches_all_links(self) -> None:
        first=build_outputs(self.root)
        self.assertEqual(first,build_outputs(self.root))
        build(self.root)
        self.assertEqual([],build(self.root,check=True))
        errors,stats=validate_site(self.root,verify_sources=False)
        self.assertEqual([],errors)
        self.assertEqual(len(load_articles(self.source))+4,stats['pages'])

    def test_source_edit_marks_its_article_stale(self) -> None:
        build(self.root)
        path=self.source/'articles/system-architecture.html'
        path.write_text(path.read_text(encoding='utf-8')+'<p>追加の検証記録。</p>',encoding='utf-8')
        self.assertIn('articles/system-architecture.html',build(self.root,check=True))

    def test_generated_edit_is_detected(self) -> None:
        build(self.root)
        (self.root/'docs/site/index.html').write_text('edited output',encoding='utf-8')
        self.assertIn('index.html',build(self.root,check=True))

    def test_published_unregistered_url_is_preserved_and_rejected(self) -> None:
        build(self.root)
        path=self.root/'docs/site/old-report.html'
        path.write_text('<h1>Historical report</h1>',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'Unregistered published'):
            build(self.root)
        self.assertTrue(path.is_file())

    def test_duplicate_slug_rejected(self) -> None:
        self.metadata(lambda records: records.append(dict(records[0])))
        with self.assertRaisesRegex(ValueError,'duplicate slug'): load_articles(self.source)

    def test_path_traversal_slug_rejected(self) -> None:
        self.metadata(lambda records: records[0].update(slug='../outside'))
        with self.assertRaisesRegex(ValueError,'invalid or duplicate slug'): load_articles(self.source)

    def test_reversed_dates_rejected(self) -> None:
        self.metadata(lambda records: records[0].update(updated='2020-01-01'))
        with self.assertRaisesRegex(ValueError,'updated before published'): load_articles(self.source)

    def test_invalid_calendar_date_rejected(self) -> None:
        self.metadata(lambda records: records[0].update(updated='2026-02-30'))
        with self.assertRaises(ValueError): load_articles(self.source)

    def test_missing_related_article_rejected(self) -> None:
        self.metadata(lambda records: records[0].update(related=['missing-article']))
        with self.assertRaisesRegex(ValueError,'related article'): load_articles(self.source)

    def test_missing_body_rejected(self) -> None:
        (self.source/'articles/system-architecture.html').unlink()
        with self.assertRaisesRegex(ValueError,'missing article body'): load_articles(self.source)

    def test_unregistered_source_rejected(self) -> None:
        (self.source/'articles/unregistered.html').write_text('<p>text</p>',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'unregistered article bodies'): load_articles(self.source)

    def test_metadata_is_html_escaped(self) -> None:
        self.metadata(lambda records: records[0].update(title='<unsafe> & "title"'))
        html=build_outputs(self.root)['articles/system-architecture.html'].decode()
        self.assertIn('&lt;unsafe&gt; &amp; &quot;title&quot;',html)
        self.assertNotIn('<unsafe>',html)

    def test_scaffold_cannot_publish_unfinished_article(self) -> None:
        path=create_article(self.root,slug='new-report',title='Report',summary='New test',
            category='report',updated='2026-09-27',status='未検証',nav='results')
        self.assertTrue(path.is_file())
        with self.assertRaisesRegex(ValueError,'unfinished article'): build_outputs(self.root)

    def test_scaffold_does_not_overwrite_existing_article(self) -> None:
        path=self.source/'articles/system-architecture.html'
        before=path.read_bytes()
        with self.assertRaisesRegex(ValueError,'already exists'):
            create_article(self.root,slug='system-architecture',title='x',summary='x',category='guide',
                updated='2026-09-27',status='x',nav='architecture')
        self.assertEqual(before,path.read_bytes())

    def test_completed_new_article_enters_catalog_and_latest_links(self) -> None:
        path=create_article(self.root,slug='new-report',title='New verification',summary='New test',
            category='report',updated='2099-01-01',status='確認済み',nav='results')
        path.write_text('<h2>Conditions</h2><p>A completed verification record.</p>',encoding='utf-8')
        build(self.root)
        for name in ('index.html','updates.html','results.html'):
            self.assertIn('articles/new-report.html',(self.root/'docs/site'/name).read_text(encoding='utf-8'))
        self.assertEqual([],validate_site(self.root,verify_sources=False)[0])


class UpdateCoverageTests(unittest.TestCase):
    path='docs/site_src/articles/test-report.html'

    def test_code_without_article_fails(self) -> None:
        with self.assertRaisesRegex(ValueError,'SITE_UPDATE_REQUIRED'):
            validate_coverage(['src/model.py'],{},None)

    def test_timestamp_or_generated_html_only_is_not_article_update(self) -> None:
        with self.assertRaises(ValueError):
            validate_coverage(['src/model.py','docs/site_src/articles.json','docs/site/articles/model.html'],{},None)

    def test_article_content_update_passes(self) -> None:
        self.assertIn('article updated',validate_coverage(['src/model.py',self.path],{self.path:('<p>old</p>','<p>new evidence</p>')},None))

    def test_article_whitespace_only_fails(self) -> None:
        with self.assertRaises(ValueError):
            validate_coverage(['src/model.py',self.path],{self.path:('<p>old</p>','<p> old </p>')},None)

    def test_deleted_article_does_not_pass(self) -> None:
        with self.assertRaises(ValueError):
            validate_coverage(['src/model.py',self.path],{self.path:('<p>old</p>','')},None)

    def test_complete_no_impact_review_passes(self) -> None:
        note=dict(schema_version=1,reviewed_paths=['tests/test_example.py'],reason='Only rename a local test variable; no behavior or public evidence changed.')
        self.assertIn('explicit no-public-impact',validate_coverage(['tests/test_example.py',NOTE],{},note))

    def test_stale_review_cannot_be_reused(self) -> None:
        note=dict(schema_version=1,reviewed_paths=['tests/test_example.py'],reason='Only rename a local test variable; no public behavior changed.')
        with self.assertRaises(ValueError): validate_coverage(['tests/test_example.py'],{},note)

    def test_incomplete_review_fails(self) -> None:
        note=dict(schema_version=1,reviewed_paths=['src/one.py'],reason='No public impact after reviewing the implementation and documentation.')
        with self.assertRaises(ValueError): validate_coverage(['src/one.py','configs/two.yaml',NOTE],{},note)

    def test_empty_reason_fails(self) -> None:
        note=dict(schema_version=1,reviewed_paths=['src/one.py'],reason='')
        with self.assertRaises(ValueError): validate_coverage(['src/one.py',NOTE],{},note)

    def test_editorial_changes_do_not_require_second_article(self) -> None:
        self.assertIn('no project-content',validate_coverage([self.path,'docs/site/index.html'],{},None))


class PageLinkTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.site=self.root/'docs/site'
        self.site.mkdir(parents=True)

    def page(self, name: str, content: str) -> None:
        (self.site/name).write_text(content,encoding='utf-8')

    def test_cross_page_fragment_valid(self) -> None:
        self.page('index.html','<h1>Home</h1><a href="article.html#evidence">Read</a>')
        self.page('article.html','<h1>Article</h1><h2 id="evidence">Evidence</h2>')
        self.assertEqual([],validate_site(self.root,False)[0])

    def test_missing_cross_page_fragment_rejected(self) -> None:
        self.page('index.html','<h1>Home</h1><a href="article.html#missing">Read</a>')
        self.page('article.html','<h1>Article</h1>')
        self.assertTrue(any('unknown fragment' in error for error in validate_site(self.root,False)[0]))

    def test_missing_page_rejected(self) -> None:
        self.page('index.html','<h1>Home</h1><a href="missing.html">Read</a>')
        self.assertTrue(any('missing/out-of-site' in error for error in validate_site(self.root,False)[0]))

    def test_external_script_rejected(self) -> None:
        self.page('index.html','<h1>Home</h1><script src="https://example.com/site.js"></script>')
        self.assertTrue(any('External runtime' in error for error in validate_site(self.root,False)[0]))


if __name__=='__main__': unittest.main()
