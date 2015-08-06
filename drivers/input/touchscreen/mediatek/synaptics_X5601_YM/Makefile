include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/synaptics_X5601_YM/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_X5601_YM/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  synaptics_dsx_fw_update.o
obj-y	+=  synaptics_dsx_i2c.o
obj-y	+=  synaptics_dsx_rmi_dev.o
