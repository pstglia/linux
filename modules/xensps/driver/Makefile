INSTALL_MOD_PATH ?= /lib/modules/$(shell uname -r)/
SPS_SRC_DIR      ?= $(shell pwd)
ARCH             ?= i386

ccflags-y += -I$(ANDROID_BUILD_TOP)/linux/modules
obj-m := secplatsvc.o
secplatsvc-objs := spsvc_module.o

export CROSS_COMPILE:=$(CROSS_COMPILE)

.PHONY=kernel_check

all: compile

kernel_check:
ifndef KDIR
	echo The KDIR variable needs to be defined
	exit 255
endif

compile: kernel_check
	$(MAKE) ARCH=$(ARCH) M=$(SPS_SRC_DIR) -C $(KDIR) modules

install:
	$(MAKE) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) ARCH=$(ARCH) M=$(SPS_SRC_DIR) -C $(KDIR) modules_install

clean:
	rm -rf *.[oas] .*.flags *.ko *.ko.* .*.cmd .*.d .*.tmp *.mod.c .tmp_versions Module*.symvers
	rm -rf Module.markers modules.order
