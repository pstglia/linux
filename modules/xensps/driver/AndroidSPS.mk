# This file is included from vendor/intel/*/board/*/Android.mk

.PHONY: secplatsvc
secplatsvc: build_kernel
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" \
	TARGET_DEVICE="$(TARGET_DEVICE)" \
	TARGET_BOARD_PLATFORM="$(TARGET_BOARD_PLATFORM)" \
	KERNEL_SRC_DIR="$(KERNEL_SRC_DIR)" \
	vendor/intel/support/kernel-build.sh $(KERNEL_BUILD_FLAGS) \
	-M linux/modules/xensps/driver

$(PRODUCT_OUT)/ramdisk.img:	secplatsvc
