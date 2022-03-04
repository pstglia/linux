BCM43xx_PATH := $(call my-dir)

# Sanity check
ifeq ($(BCM43xx_CHIP),$(empty))
$(error Variable BCM43xx_CHIP is not set)
endif # BCM43xx_CHIP

ifneq ($(wildcard $(BCM43xx_PATH)/src/Makefile),$(empty))
$(eval $(call build_kernel_module,$(BCM43xx_PATH)/src,bcm$(BCM43xx_CHIP),CONFIG_BCMDHD=m CONFIG_BCM$(BCM43xx_CHIP)=y DRIVER=bcm$(BCM43xx_CHIP) CONFIG_DHD_USE_SCHED_SCAN=y))
else
$(warning bcm$(BCM43xx_CHIP) specified in device configuration but not found)
endif
