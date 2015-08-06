#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

define CUST_FILTER_PROJECT_DRV
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/imgsensor/$(1)),)
project_drv += $(1)
endif
endef

$(foreach drv,$(subst $\",,$(CONFIG_CUSTOM_KERNEL_IMGSENSOR)),$(eval $(call CUST_FILTER_PROJECT_DRV,$(drv))))
custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(project_drv)))
obj-y += $(custom_kernel_imgsensor)
$(info imgsensor drv by project $(project_drv))
$(info CONFIG_CUSTOM_KERNEL_IMGSENSOR is $(CONFIG_CUSTOM_KERNEL_IMGSENSOR), drv is $(drv), and $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/imgsensor)