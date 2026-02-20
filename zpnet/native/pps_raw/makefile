obj-m += pps_raw.o

# Default: build against running kernel
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	sudo cp pps_raw.ko /lib/modules/$(shell uname -r)/extra/
	sudo depmod -a

load: all
	sudo insmod pps_raw.ko gpio_pin=18

unload:
	sudo rmmod pps_raw

reload: unload load

.PHONY: all clean install load unload reload