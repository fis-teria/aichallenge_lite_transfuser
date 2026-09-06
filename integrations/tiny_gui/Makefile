# Include the existing dev recipe without editing the dirty racing-kart checkout.
TINY_GUI_INCLUDE := $(abspath $(lastword $(MAKEFILE_LIST)))
ifeq ($(strip $(RACINGKART_REPO)),)
$(error RACINGKART_REPO is required)
endif
include $(RACINGKART_REPO)/Makefile

ifeq ($(CONTROL_METHOD),tiny_lidar_net_guarded)
# GNU make does not propagate -f to recursive $(MAKE). Keep child invocations
# on this same include rather than the source checkout's unrelated Makefile.
MAKE := $(MAKE) -f $(TINY_GUI_INCLUDE)
# Only this new SIM-only method uses the narrow evidence recorder. Existing
# methods retain their original capture-run-fingerprint recipe and launch.
capture-run-fingerprint:
	python3 "$(TINY_SOURCE_ROOT)/tools/tiny_gui_integration.py" --fingerprint "$(TINY_TRIAL_OUTPUT)" --source "$(TINY_SOURCE_ROOT)" --racing-repo "$(RACINGKART_REPO)"
dev: AWSIM_START_MODE := off
dev: DEV_AUTO_START := false
endif
