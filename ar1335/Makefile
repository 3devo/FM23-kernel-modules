# SPDX-License-Identifier: GPL-2.0
ar1335-objs += ar1335_mipi_v3.o
obj-m += ar1335.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
