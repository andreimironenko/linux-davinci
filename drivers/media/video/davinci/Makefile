#
# Makefile for the davinci video device drivers.
#
# Capture: DM6446 and DM355
obj-$(CONFIG_VIDEO_VPSS_SYSTEM) += vpss.o
obj-$(CONFIG_VIDEO_VPFE_CAPTURE) += vpfe_capture.o
obj-$(CONFIG_VIDEO_DM6446_CCDC) += dm644x_ccdc.o
obj-$(CONFIG_VIDEO_DM355_CCDC) += dm355_ccdc.o
dm355_af_driver-objs		+= dm355_af.o dm355_af_hw.o
obj-$(CONFIG_VIDEO_DM355_AF)	+= dm355_af_driver.o
dm355_aew_driver-objs		+= dm355_aew.o dm355_aew_hw.o
obj-$(CONFIG_VIDEO_DM355_AEW)	+= dm355_aew_driver.o
