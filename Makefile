ifneq (${KERNELRELEASE},)
	obj-m := kthread.o
else
	KERNEL_SOURCE := /home/yliu41/LFC/test_color
	PWD := $(shell pwd)
default:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} modules

clean:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} clean
endif
