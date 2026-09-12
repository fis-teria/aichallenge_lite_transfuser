"""Read IL from the frozen simulator; never patch simulator assemblies."""
from __future__ import annotations
import json
from pathlib import Path
import sys

ROOT = Path('/home/si26-pc008/cma_mppi_20260912')
sys.path.insert(0, str(ROOT / 'inspect_deps'))
import dnfile
from dncil.cil.body import CilMethodBody
from dncil.cil.body.reader import CilMethodBodyReaderBytes


def main() -> None:
    pe = dnfile.dnPE(str(ROOT / 'snapshot/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll'))
    owners = {id(m.row): str(t.TypeName) for t in pe.net.mdtables.TypeDef for m in t.MethodList}
    owners.update({id(f.row): str(t.TypeName) for t in pe.net.mdtables.TypeDef for f in t.FieldList})
    types = ['VehicleHandicapController', 'AdminSummaryRos2Publisher', 'ROS2Utility',
             'OvertakingLaneJudge', 'VehicleHandicapInstaller']
    lines = []
    for t in pe.net.mdtables.TypeDef:
        if str(t.TypeName) not in types:
            continue
        for method in t.MethodList:
            m = method.row
            lines.append(str(t.TypeName) + '.' + str(m.Name))
            if not m.Rva:
                continue
            body = CilMethodBody(CilMethodBodyReaderBytes(pe.get_data(m.Rva, 65536)))
            for instruction in body.instructions:
                line = str(instruction)
                token = getattr(instruction.operand, 'value', 0)
                table_id, index = token >> 24, (token & 0xffffff) - 1
                if table_id == 0x70:
                    line += ' # ' + repr(str(pe.net.user_strings.get(token & 0xffffff)))
                elif table_id in {0x0a, 0x06, 0x04, 0x02, 0x01}:
                    table = getattr(pe.net.mdtables, {0x0a:'MemberRef',0x06:'MethodDef',0x04:'Field',0x02:'TypeDef',0x01:'TypeRef'}[table_id])
                    row = table[index]
                    line += ' # ' + owners.get(id(row), '') + '.' + str(getattr(row, 'Name', getattr(row, 'TypeName', '')))
                lines.append(line)
    output = ROOT / 'awsim_inspection.il.txt'
    output.write_text('\n'.join(lines))
    print(json.dumps({'output': str(output), 'lines': len(lines)}))


if __name__ == '__main__':
    main()
