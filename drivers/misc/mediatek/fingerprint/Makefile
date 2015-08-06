include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

ifeq ($(CONFIG_RGK_DRIVER_FINGERPRINT_GF66XX),y)
obj-y   +=  gf66xx/
endif

ifeq ($(CONFIG_RGK_DRIVER_FINGERPRINT_MADEV),y)
obj-y   +=  madev/
endif
