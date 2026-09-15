import sys
import manage as m

pair=int(sys.argv[1])
m.remote("import sys\nsys.argv=['assess_native.py','--pair',"+repr(str(pair))+"]\n"+(m.HERE/'assess_native.py').read_text(),
    native=True,lock=True,timeout=180)
if pair==1:
    m.copy_to_host([m.UNC/'pilot_gate.json'])
