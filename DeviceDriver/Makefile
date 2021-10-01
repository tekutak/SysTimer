CFILES = systimer.c

obj-m := systimer.o
SystimerModule-objs := $(CFILES:.c=.o)

ccflags-y += -std=gnu11 -Wall -Wno-declaration-after-statement

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean