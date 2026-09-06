.PHONY: test-v3 bag-scan dataset-audit dev

# Linux/SSH simulator host only. Separate, finite V4 profile; never starts the
# existing classic controller or changes the AWSIM binary/assets.
V4_PHASE ?= stationary
V4_WALL_SECONDS ?= 60
DEV_CONTROLLER ?= v4
TINY_PHASE ?= stationary
TINY_WALL_SECONDS ?= 120
dev:
ifeq ($(CONTROL_METHOD),tiny_lidar_net_guarded)
	python3 tools/tiny_dev_runner.py --control-method tiny_lidar_net_guarded $(if $(filter 1,$(TINY_GUI_RETRY)),--gui-retry,) $(if $(filter 2,$(TINY_GUI_RETRY)),--gui-retry2,) --sim-repo "$(SIM_REPO)" --official-package "$(TINY_PACKAGE)" --install-root "$(TINY_INSTALL)" --display "$(DISPLAY)" --xauthority "$(XAUTHORITY)" --output "$(TINY_OUTPUT)" --commit "$(TINY_COMMIT)" --phase short --wall-seconds 120 --budget "$(TINY_BUDGET)"
else
ifeq ($(DEV_CONTROLLER),tiny)
	python3 tools/tiny_dev_runner.py --sim-repo "$(SIM_REPO)" --official-package "$(TINY_PACKAGE)" --xvfb-root "$(XVFB_ROOT)" --output "$(TINY_OUTPUT)" --commit "$(TINY_COMMIT)" --phase "$(TINY_PHASE)" --wall-seconds "$(TINY_WALL_SECONDS)" --budget "$(TINY_BUDGET)"
else
	python3 tools/spatial_dev_runner.py --sim-repo "$(SIM_REPO)" --checkpoint "$(V4_CHECKPOINT)" --xvfb-root "$(XVFB_ROOT)" --output "$(V4_OUTPUT)" --commit "$(V4_COMMIT)" --phase "$(V4_PHASE)" --wall-seconds "$(V4_WALL_SECONDS)" --budget "$(V4_BUDGET)" --binding "$(V4_BINDING)" --forward-limit "$(if $(V4_FORWARD_LIMIT),$(V4_FORWARD_LIMIT),0)"
endif
endif

test-v3:
	python -m pytest -q

bag-scan:
	python -m aic_transfuser_lite.cli bag scan --input-root "$(INPUT)" --output "$(OUTPUT)"

dataset-audit:
	python -m aic_transfuser_lite.cli dataset audit --dataset-root "$(DATASET)" --output "$(OUTPUT)"
