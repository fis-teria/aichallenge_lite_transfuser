# Repeatedly validated MPPI refinement

Use each exact reference CSV with its paired MPPI YAML and the `reference_execution_speed_cap_mps` in the corresponding execution JSON. The original CSV speed column alone is not the execution-speed contract.

Normal candidates were compared as four ghosts sharing one AWSIM from the same D1 pose, with both planner and recovery other-vehicle inputs empty. Leader candidates were evaluated in isolated single-car simulations so native rank-1 handicap remained active throughout.

Selection requires all four repeats and dense OT checks to pass, at least 0.05 seconds median improvement, and three wins in four ordered comparisons; otherwise the incumbent is retained. See summary.json for the actual decision, not just the fastest search sample. This is a finite local search and does not establish a global optimum.
