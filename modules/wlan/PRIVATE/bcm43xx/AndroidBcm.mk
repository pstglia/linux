# This makefile is included from device/intel/common/wifi/WifiRules.mk.

include $(call my-dir)/external_AndroidBcm.mk
$(info *** Using sources for bcm$(BCM43xx_CHIP) ...)

# The following is to generate the intel_prebuilts
ifeq ($(GENERATE_INTEL_PREBUILTS),true)

$(info *** Generating prebuilt for bcm$(BCM43xx_CHIP) ...)

define built-bcm-driver-$(BCM43xx_CHIP)

.PHONY: bcm$(1)_prebuilt

KMODULE_NAME_BCM$(1) := bcm$(1)
KMODULE_PREBUILT_BCM$(1) := $(PRODUCT_OUT)/$(call intel-prebuilts-path,$(2))
KMODULE_PREBUILD_SRC_BCM$(1) := $$(KMODULE_PREBUILT_BCM$(1))/src_$$(KMODULE_NAME_BCM$(1))

$$(KMODULE_PREBUILD_SRC_BCM$(1))/Makefile: $(2)/Makefile.in
	$(hide) mkdir -p $$(@D)
	$(hide) sed -e 's?%BCM_DRIVER%?bcm$(1)?g' $$< >$$@

bcm$(1)_prebuilt: bcm$(1)_install | $(ACP)
bcm$(1)_prebuilt: $$(KMODULE_PREBUILD_SRC_BCM$(1))/Makefile
	$(hide) echo Copying $$(KMODULE_NAME_BCM$(1))_bin.o as prebuilt
	$(hide) $(ACP) -fp $(PRODUCT_OUT)/$(2)/src_$$(KMODULE_NAME_BCM$(1))/$$(KMODULE_NAME_BCM$(1)).o \
			   $$(KMODULE_PREBUILD_SRC_BCM$(1))/$$(KMODULE_NAME_BCM$(1))_bin.o_shipped
	$(hide) $(ACP) -fp $(2)/external_AndroidBcm.mk \
			   $$(KMODULE_PREBUILT_BCM$(1))/AndroidBcm.mk

# Make sure we install the prebuilt module for intel_prebuilts.
intel_prebuilts: bcm$(1)_prebuilt

endef

$(eval $(call built-bcm-driver-$(BCM43xx_CHIP),$(BCM43xx_CHIP),$(BCM43xx_PATH)))

endif # GENERATE_INTEL_PREBUILTS

