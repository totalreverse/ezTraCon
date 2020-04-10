#   EzTraCon - ezUSB Trainer Controller
#   Copyright (C) 2019 by Michael Hipp                                    


SDCC = sdcc
ASM = sdas8051
ASMFLAGS = -xlogs

# D52 = echo 
D52 = d52
D52FLAGS = -dxnp

IHXFILE_ANTPLUS    = ezTraCon_Ant_Motor.hex
IHXFILE_ANTPLUS_EC = ezTraCon_Ant_Eddy.hex

IHXFILE_CLASSIC    = ezTraCon_Classic.hex
IHXFILE_CLASSIC_MB = ezTraCon_Classic_Motor.hex
IHXFILE_CLASSIC_EC = ezTraCon_Classic_Eddy.hex

IHXFILE_FAT        = ezTraCon_Fat.hex

# Choose ONE of the following USB communication options:
#
# 1. Emulate an ANTPLUS FE-C Device with (default) Dynastream-USB ID "0fcf:1008"
#   -DWITH_EMULATE_USB_ANTFE_C
#
#   This switchs from classic 24/48/64-byte Tacx T1904,T1932,T1942 protocol to emulated ANTPLUS 'usb stick'
#   This switch is highly experimental and adds lot of additional code, so you cannot compile a
#   multi-function firmware with eddy current and motorbrake
#
# or:
#
# 2. Use FORTIUS 24/48/64 Bytes Protocol with (default) Taxc-USB ID "3561:1942"
#   -DWITH_FORTIUS_PROTOCOL
#
# or: (TODO)
#
# 3. The good old iMagic (solid green) protocol  (not yet supported) with (default) Tacx-USB ID  "3561:1902"
#   -DWITH_IMAGIC_PROTOCOLL
#

#########################################
# Additional options
#########################################
#
# Forces USB-ID
#
# -DWITH_USB_ID_T1942_SOLID_BLUE
# -DWITH_USB_ID_T1902_SOLID_GREEEN
# -DWITH_USB_VENDOR_ID=
# -DWITH_USB_DEVICE_ID=
#
#########################################
#
# -DWITH_EDDYCURRENT
#
#  => HIGHLY EXPERIMENTAL: enables eddy current brake handling via reverse phase control ("PAC/PWM/RPC")
#     !! "60Hz/110V"-Implementation not tested !!
#
###################
#
# -DWITH_MOTORBRAKE
#
# => enables motorbrake handling via serial communication. 
#
# Allows additional options:
#
#   -DWITH_BITBANG
#      or:
#   -DWITH_SERIAL0
#   => how to handle the serial communication (BITBANG works with both: the solid green and the solid blue head unit)
#
#   -DWITH_SERIAL_8N2
#   => switch from 8N1 to 8N2 (experimental and not necessary)
#
###########################################
# -DWITH_USB_POLL 
#
# enables simple NON IRQ USB communication (recommended)
#
###########################################
# Addition (development) options:
#
# -DWITH_DEVELOPMENT
# -DWITH_DEVELOPMENT_LOG_ANT
# -DWITH_DEVELOPMENT_SERIAL
# -DWITH_STATISTICS_GENERAL
# -DWITH_STATISTICS_IRQ
# -DWITH_STATISTICS_SERIAL
# -DWITH_ANALYZER
# -DWITH_STATISTICS_GENERAL
# -DWITH_STATISTICS_IRQ
# -DWITH_STATISTICS_SERIAL
# -DWITH_ANTPLUS_SEARCH_TIMEOUT_EMULATION
# -DWITH_LOAD_DATA_TO_UPPER_MEM
#  => needs special fxload - allows to load static data to 0x1b40 and higher
#  => needs -DWITH_USB_POLL
# 

# --acall-ajmp --stack-loc 0x80 --nogcse --peep-asm
CFLAGS  = --std-sdcc99 --opt-code-size --model-small 
LDFLAGS = --code-loc 0x0000 --model-small

#--callee-saves myFunction1,myFunction2,....

$(IHXFILE_ANTPLUS):	CFLAGS += -DWITH_EMULATE_USB_ANTFE_C -DWITH_MOTORBRAKE -DWITH_BITBANG -DWITH_USB_POLL  #-DWITH_ANALYZER 
$(IHXFILE_ANTPLUS_EC):	CFLAGS += -DWITH_EMULATE_USB_ANTFE_C -DWITH_EDDYCURRENT -DWITH_USB_POLL 
$(IHXFILE_CLASSIC):	CFLAGS += -DWITH_MOTORBRAKE -DWITH_BITBANG -DWITH_USB_POLL -DWITH_STATISTICS_GENERAL -DWITH_STATISTICS_IRQ  -DWITH_ANALYZER  \
				-DWITH_EDDYCURRENT -DWITH_EXTENDED_PROTOCOL -DWITH_USB_ID_T1942_SOLID_BLUE
$(IHXFILE_CLASSIC_MB):	CFLAGS += -DWITH_MOTORBRAKE -DWITH_BITBANG -DWITH_USB_POLL -DWITH_DEVELOPMENT -DWITH_STATISTICS_GENERAL -DWITH_STATISTICS_IRQ  -DWITH_ANALYZER -DWITH_USB_ID_T1942_SOLID_BLUE
$(IHXFILE_CLASSIC_EC):	CFLAGS += -DWITH_EDDYCURRENT -DWITH_EXTENDED_PROTOCOL  -DWITH_USB_POLL -DWITH_DEVELOPMENT -DWITH_STATISTICS_GENERAL -DWITH_STATISTICS_IRQ  -DWITH_ANALYZER -DWITH_USB_ID_T1942_SOLID_BLUE
$(IHXFILE_FAT):		CFLAGS += -DWITH_BITBANG -DWITH_MOTORBRAKE -DWITH_EMULATE_USB_ANTFE_C -DWITH_EDDYCURRENT -DWITH_USB_POLL 

#$(IHXFILE_CLASSIC):	CFLAGS += -DWITH_MOTORBRAKE -DWITH_BITBANG -DWITH_EDDYCURRENT -DWITH_USB_POLL \
#			          -DWITH_DEVELOPMENT -DWITH_STATISTICS_GENERAL -DWITH_STATISTICS_IRQ  -DWITH_STATISTICS_SERIAL -DWITH_ANALYZER \
#				  -DWITH_EXTENDED_PROTOCOL

# 0x0000 to 0x1b00 is classical code space
# 0x1b00 to 0x1b40 is USB-ISR jump-table space, that must start at a 0xXX00 adress
#                  not used when -DWITH_USB_POLL - can be acessed as (initialized) xdata but not as code
# 0x1b40 to 0x1dc0 IN/OUTBUF 3-7 is currently unused direct usable memory (we do not use these IO buffers)
#                  can be acessed/used as (initialized) xdata but not as code
# 0x1dc0 to 0x1e40 IN/OUT #2 In antplus mode we only use IN/OUTBUF 0,1 for communication 
#                  can be acessed/used as (initialized) xdata but not as code (only for ANTPLUS emulation)
# 0x1dc0 to 0x1f00 (0x7dc0-0x7f00) is EZ-USB memory for the communication buffers (OUTBUF/INBUF 0..2)
# 0x1f00 to 0x2000 (0x7f00-0x8000) are EZ-USB registers
# 0x2000 to 0x2800 is directly adressable __xdata space - because we do not use the isochronous memory 
#                  probably not zeroed if you use if for xdata ??

# -DWITH_LOAD_DATA_TO_UPPER_MEM 
#  0x1b00..0x1c00 for antplus protocol static initialized data
#  0x1b80..0x1c00 for classic protocol static initialized USB decriptor data
#
# THIS WORKS ONLY WITH A SPECIAL FXLOAD TOOL!

$(IHXFILE_ANTPLUS):	LDFLAGS += --code-size 0x1b40 --xram-loc 0x1c00 --xram-size 0x240 --iram-size 0x100
$(IHXFILE_ANTPLUS_EC):	LDFLAGS += --code-size 0x1b40 --xram-loc 0x1c00 --xram-size 0x240 --iram-size 0x100 
$(IHXFILE_CLASSIC):	LDFLAGS += --code-size 0x1b40 --xram-loc 0x1c00 --xram-size 0x1c0 --iram-size 0x100 
$(IHXFILE_CLASSIC_MB):	LDFLAGS += --code-size 0x1b40 --xram-loc 0x1c00 --xram-size 0x1c0 --iram-size 0x100 
$(IHXFILE_CLASSIC_EC):	LDFLAGS += --code-size 0x1b40 --xram-loc 0x1c00 --xram-size 0x1c0 --iram-size 0x100 
# DO not use the FAT file. It will not fit into EZUSB code space. It is just to see how large a full featured version would be
$(IHXFILE_FAT):		LDFLAGS = --code-size 0x2400 --xram-loc 0x2400 --xram-size 0x200 --iram-size 0x100 

SRCDIR   = src
OBJDIR   = build
SOURCES  = $(SRCDIR)/analyzer.c  $(SRCDIR)/antplus.c  $(SRCDIR)/eddycurrent.c  $(SRCDIR)/i2c.c  $(SRCDIR)/main.c  $(SRCDIR)/serial.c  $(SRCDIR)/usb.c $(SRCDIR)/math.c $(SRCDIR)/blinkcode.c $(SRCDIR)/models.c
#PSOURCES = 
#SOURCES  := $(wildcard $(SRCDIR)/*.c)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.rel) 
#POBJECTS := $(PSOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.rel) 
# $(IHXFILE_FAT)
TARGETS = $(IHXFILE_ANTPLUS) $(IHXFILE_CLASSIC) $(IHXFILE_ANTPLUS_EC) $(IHXFILE_CLASSIC_EC) $(IHXFILE_CLASSIC_MB) 


BUILD = build
# for non USB_POLL mode add the usb Jump Table: $(BUILD)/usbJT.rel
# OBJECTS = $(BUILD)/usb.rel $(BUILD)/i2c.rel $(BUILD)/blickcode.rel  $(BUILD)/main.rel  $(BUILD)/serial.rel $(BUILD)/eddycurrent.rel  $(BUILD)/analyzer.rel $(BUILD)/antplus.rel

DEPS = usb.h i2c.h reg_ezusb.h main.h tacx.h antplus.h timestamp.h trainer.h eddycurrent.h math.h

#########################################

.PHONY: all, cleanall, build, clean

all:	cleanall antplus antplusec classic classicec classicmb 
	tail -n 8 $(BUILD)x/*.mem

#########################################

#$(POBJECTS): $(OBJDIR)/%.rel : $(SRCDIR)/%.c $(OBJDIR)/timestamp.h
#	$(SDCC) -c $(CFLAGS) --acall-ajmp -mmcs51 -I. -I$(OBJDIR) -o $@ $<

$(OBJECTS): $(OBJDIR)/%.rel : $(SRCDIR)/%.c $(OBJDIR)/timestamp.h
	$(SDCC) -c $(CFLAGS) -mmcs51 -I. -I$(OBJDIR) -o $@ $<

$(BUILD)/%.rel: %.a51 $(DEPS) $(OBJECTS) 
	$(ASM) $(ASMFLAGS) -o $@ $<

$(BUILD)/%.rel: %.asm $(DEPS) $(OBJECTS)
	$(ASM) $(ASMFLAGS) -o $@ $<

$(OBJDIR)/timestamp.h:
	mkdir -p build
	echo "// timestampC is days since 2018/02/xx" > $@
	echo "#define timestampC 0x"`expr $$(date +%s) / 86400 - 17600` >> $@
	echo "#define timestampStr  STR_DESCR_0(14) `date +%y/%m/%d\ %H:%M | sed "s/\(.\)/,'\1'/g"`" >> $@
	# echo "__code uint16_t strSerialNumber[] = STR_DESCR2(19"`date +%Y/%m/%d\ %H:%M:%S | sed "s/\(.\)/,'\1'/g"`");" >> timestamp.h

# Always clean before build - so different target do not interfere
$(TARGETS): $(OBJECTS) $(POBJECTS)
	$(SDCC) -mmcs51 $(LDFLAGS) -o $@ $^
	d52 -dxnp $@ 
	cat $(@:.hex=.mem)
	mkdir -p $(BUILD)x
	mv -f *.lk *.map *.mem *.d52 $(BUILD)x


########################################

classic:
	@echo START: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}
	rm -f $(IHXFILE_CLASSIC)
	$(MAKE) clean  $(IHXFILE_CLASSIC)
	@echo FINISHED: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}

antplus:
	@echo START: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}
	rm -f $(IHXFILE_ANTPLUS)
	$(MAKE) clean  $(IHXFILE_ANTPLUS)
	@echo FINISHED: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}

classicec:
	@echo START: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}
	rm -f $(IHXFILE_CLASSIC_EC)
	$(MAKE) clean  $(IHXFILE_CLASSIC_EC)
	@echo FINISHED: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}

classicmb:
	@echo START: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}
	rm -f $(IHXFILE_CLASSIC_MB)
	$(MAKE) clean  $(IHXFILE_CLASSIC_MB)
	@echo FINISHED: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}

antplusec:
	@echo START: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}
	rm -f $(IHXFILE_ANTPLUS_EC)
	$(MAKE) clean  $(IHXFILE_ANTPLUS_EC)
	@echo FINISHED: target=$@ CFLAGS=${CFLAGS} LDFLAGS=${LDFLAGS}


#########################################

cleanall: clean
	rm -rf *.ihx *.hex $(BUILD)x

clean:
	@echo  clean $(BUILD)
	rm -rf $(BUILD)





