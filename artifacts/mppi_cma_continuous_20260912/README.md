# Automatic CMA continuation launch evidence

This is a running-campaign launch check, not a completed optimization result.
The original CMA distribution and RNG continue from generation 3.
New batches are admitted for three hours, at most six rounds / 384 vehicle evaluations.
Each round adds three generations per condition and four new/old comparison runs per route.
Completed results are exported on SI26 under continuous/results/round-NNN.
The dashboard follows continuous/state.json at http://127.0.0.1:8876 on SI26.
See docs/mppi_cma_si26_20260912.md for stop/resume commands and limits.
