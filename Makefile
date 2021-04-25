obj-m += xioparser.o
xioparser-objs := OSC99/OscAddress.o OSC99/OscError.o OSC99/OscPacket.o OSC99/OscSlip.o \
OSC99/OscBundle.o OSC99/OscMessage.o OSC99/OscCommon.o xioparser_main.o
include /home/cps/machinekit/src/Makefile.modinc

.PHONY: clean

clean:
	rm -f *.so *.ko *.o
	rm -f *.sym *.tmp *.ver
	rm -f *.mod.c .*.cmd
	rm -f modules.order Module.symvers
	rm -rf .tmp_versions 