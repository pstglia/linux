obj-$(CONFIG_X86_INTEL_MID) += intel-mid.o intel_mid_vrtc.o mfld.o mrfl.o
obj-y += spid.o

# SFI specific code
ifdef CONFIG_X86_INTEL_MID
obj-$(CONFIG_SFI) += sfi.o device_libs/
endif

obj-y += platform_sst_audio.o
obj-$(subst m,y,$(CONFIG_MFD_ARIZONA_SPI)) += platform_byt_audio_spi.o
obj-$(subst m,y,$(CONFIG_MFD_ARIZONA_SPI)) += platform_wm5102_spi.o
obj-$(subst m,y,$(CONFIG_VLV2_PLAT_CLK)) += platform_vlv2_plat_clk.o intel_mid_pcihelpers.o
