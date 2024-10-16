# SPDX-License-Identifier: GPL-2.0
obj-m += 3devo_wdt.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
