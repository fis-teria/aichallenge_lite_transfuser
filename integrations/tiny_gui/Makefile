# Include the existing dev recipe without editing the dirty racing-kart checkout.
ifeq ($(strip $(RACINGKART_REPO)),)
$(error RACINGKART_REPO is required)
endif
include $(RACINGKART_REPO)/Makefile

ifeq ($(CONTROL_METHOD),tiny_lidar_net_guarded)
# Only this new SIM-only method uses the narrow evidence recorder. Existing
# methods retain their original capture-run-fingerprint recipe and launch.
capture-run-fingerprint:
	python3 "$(TINY_SOURCE_ROOT)/tools/tiny_gui_integration.py" --fingerprint "$(TINY_TRIAL_OUTPUT)" --source "$(TINY_SOURCE_ROOT)" --racing-repo "$(RACINGKART_REPO)"
dev: AWSIM_START_MODE := off
dev: DEV_AUTO_START := false
endif
