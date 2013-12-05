subdir-ccflags-$(CONFIG_ARCH_MSM) += -Iarch/arm/mach-msm

-include $(src)/Makefile.board

obj-y += boot_mode.o
obj-y += pcb_version.o
obj-y += oppo_attributes.o
obj-y += oppo_hs_detect.o
obj-$(CONFIG_OPPO_DEVICE_INFO) += device_info.o
obj-$(CONFIG_OPPO_TRACE) += oppo_trace.o
