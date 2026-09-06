"""Validate the used keyword subset with existing system jsonschema 3.x.

This is NOT certification of all draft2020-12 semantics. Refuse any keyword not
present in the explicitly supported subset; no install, network or remote ref.
"""
import argparse
import json
from pathlib import Path
import sys

from jsonschema import Draft7Validator, RefResolver

KEYWORDS={'$schema','$id','$ref','$defs','title','description','type','additionalProperties','required','properties',
          'const','enum','allOf','oneOf','if','then','else','items','minItems','maxItems','minLength','maxLength',
          'pattern','minimum','maximum','exclusiveMinimum'}


class LocalResolver(RefResolver):
    """No URL resolution: every payload reference must target the pinned design."""
    def resolve(self, ref):
        document,fragment=ref.split('#',1)
        if document not in ('',self.design['$id']) or not fragment.startswith('/$defs/'):
            raise ValueError('nonlocal or unexpected schema reference')
        value=self.design
        for part in fragment.strip('/').split('/'):
            value=value[part]
        return self.design['$id']+'#'+fragment,value

    def resolve_remote(self, uri):
        raise ValueError('remote schema retrieval forbidden')


def check_schema(s):
    if set(s)-KEYWORDS:
        raise ValueError('unsupported keywords '+str(set(s)-KEYWORDS))
    for k in ('properties','$defs'):
        for v in s.get(k,{}).values(): check_schema(v)
    for k in ('items','if','then','else'):
        if isinstance(s.get(k),dict): check_schema(s[k])
    for k in ('allOf','oneOf'):
        for v in s.get(k,[]): check_schema(v)


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--events',type=Path,required=True); args=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    design=json.loads((root/'schemas/spatial_path_v4_shadow_record_v1.schema.json').read_text())
    runtime=json.loads((root/'schemas/spatial_path_v4_runtime_record_v1.schema.json').read_text())
    check_schema(design); check_schema(runtime)
    resolver=LocalResolver.from_schema(runtime)
    resolver.design=design
    validator=Draft7Validator(runtime,resolver=resolver)
    count=0
    for line in args.events.read_text().splitlines():
        validator.validate(json.loads(line)); count+=1
    print(json.dumps(dict(status='PASS_USED_KEYWORD_SUBSET',records=count,validator='system jsonschema Draft7Validator with local refs',full_draft2020_12='NOT_EXECUTED',python=sys.version)))
