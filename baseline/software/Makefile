ALT_DEVICE_FAMILY ?= soc_cv_av
SOCEDS_ROOT ?= c:/intelFPGA_lite/17.0/embedded
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
CROSS_COMPILE = arm-linux-gnueabihf-
CROSS_CFLAGS = -g -Wall  \
		-D$(ALT_DEVICE_FAMILY) \
		-I../conversion		\
		-I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) \
		-I$(HWLIBS_ROOT)/include/ \
                -pg
CROSS_LDFLAGS = -g -Wall -lm
CROSS_CC = $(CROSS_COMPILE)gcc
CROSS_OBJDUMP= $(CROSS_COMPILE)objdump
CROSS_SIZE= $(CROSS_COMPILE)size
FUNC_CC = gcc -lm -DNODE1SOC -pg

all: access dmademo sumdemo

access: access.c
	$(CROSS_CC) $(CROSS_CFLAGS) $(CROSS_LDFLAGS) $^ -o $@
	$(CROSS_OBJDUMP) -S -d -l $@ > access.lst

sumdemo: sumdemo.c
	$(CROSS_CC) $(CROSS_CFLAGS) $(CROSS_LDFLAGS) $^ -o $@
	$(CROSS_OBJDUMP) -S -d -l $@ > sumdemo.lst

dmademo: dmademo.c
	$(CROSS_CC) $(CROSS_CFLAGS) $(CROSS_LDFLAGS) $^ -o $@
	$(CROSS_OBJDUMP) -S -d -l $@ > dmademo.lst

%.o : %.c
	$(CROSS_CC) $(CROSS_CFLAGS) -c $< -o $@

.PHONY: clean

clean:
	rm -f $(TARGET) $(TARGET)_x86 *.lst \
		wave*.txt *.a *.o *~ \
		access access.lst gmon.out \
		dmademo sumdemo
