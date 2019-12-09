#
# Makefile for industrial I/O Magnetometer sensors
#

obj-$(CONFIG_SENSORS_HMC5843)	+= hmc5843.o
obj-$(CONFIG_SENSORS_YAS537) += yas_mag_drv-yas537.o yas_mag_kernel.o
CFLAGS_yas_mag_kernel.o += -Idrivers/staging/iio
obj-$(CONFIG_SENSORS_STK8313) += yas_acc_drv-stk8313.o yas_acc_kernel.o
CFLAGS_yas_acc_kernel.o += -Idrivers/staging/iio
