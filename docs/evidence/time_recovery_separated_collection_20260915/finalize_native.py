import manage as m

for script in ('verify_completion_native.py','explain_hold_evidence.py'):
    m.remote((m.HERE/script).read_text(),native=True,lock=True,timeout=240)
