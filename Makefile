.PHONY: test-v3 bag-scan dataset-audit dev

# Linux/SSH simulator host only. Separate, finite V4 profile; never starts the
# existing classic controller or changes the AWSIM binary/assets.
V4_PHASE ?= stationary
V4_WALL_SECONDS ?= 60
dev:
	python3 tools/spatial_dev_runner.py --sim-repo "$(SIM_REPO)" --checkpoint "$(V4_CHECKPOINT)" --xvfb-root "$(XVFB_ROOT)" --output "$(V4_OUTPUT)" --commit "$(V4_COMMIT)" --phase "$(V4_PHASE)" --wall-seconds "$(V4_WALL_SECONDS)" --budget "$(V4_BUDGET)" --binding "$(V4_BINDING)"

test-v3:
	python -m pytest -q

bag-scan:
	python -m aic_transfuser_lite.cli bag scan --input-root "$(INPUT)" --output "$(OUTPUT)"

dataset-audit:
	python -m aic_transfuser_lite.cli dataset audit --dataset-root "$(DATASET)" --output "$(OUTPUT)"
