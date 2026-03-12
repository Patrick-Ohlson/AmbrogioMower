#/****************************************************************
#KPIT Cummins Infosystems Limited Pune, India. - 01-Sept-2003.
#
#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
#*****************************************************************/

PREFIX=h8300-elf
#FIX FOR ABSOLUTE ADDRESS

ADDRESS = 0x00000000

# Specify compiler to be used
CC = $(PREFIX)-gcc

# Specify Assembler to be used
AS = $(PREFIX)-as

# Specify linker to be used
LD = $(PREFIX)-ld

# Specify CPU flag
CPU = -ms -ms2600
#add flag "-mn" for normal mode

# Build output directory
BUILDDIR = .build

# Common compiler flags
#   -save-temps removed to avoid cluttering base directory
#   (KPIT make.exe cannot do separate compile to .build/ — use build.sh for that)
CFLAGS = -nostartfiles -Os  -ggdb
#-O2

ALL_FLAGS = $(CFLAGS) $(CPU) -DINTEGRATED

# Application name
APPNAME = main

# Source files
#
#Monitor testbed
#SOURCES = monitor.s
#
#L200 Testbed
#SOURCES = _1_StartHere_line200_300-v2010-20130724-r17157.zip_l200-v2010-basic_IDA7_BASE4DECOMP_V2.s
#
#AM2000 Testbed
#SOURCES = 1_StartHere_testam2000-20140417-r20335.zip_testam2000_a-with-boot-2144.BIN.s 1_testam_patch_C.c 1_testam_patch_end.s

#Main testbed
SOURCES = start.S vects.c sci.c hwinit.c monitor.S monitor.c \
          timer/timer.c lcd/lcd.c misc/misc.c i2c/i2c.c rtc/rtc.c motor/motor.c \
          steering/steering.c \
          utils/utils.c \
          eeprom/eeprom.c eeprom/config.c \
          accel/accel.c \
          keyboard/keyboard.c \
          battery/battery.c \
          wire/wire.c \
          bump/bump.c \
          active_object/smallheap.c active_object/queue.c active_object/hsm.c \
          active_object/schedulerrt.c active_object/error.c active_object/ao.c \
          tests/test_motor.c tests/test_drive.c tests/test_i2c.c tests/test_rtc.c \
          tests/test_memory.c tests/test_misc.c tests/test_lcd.c \
          tests/test_timer.c tests/test_sci.c tests/test_statics.c \
          tests/test_steering.c tests/test_eeprom.c tests/test_accel.c \
          tests/test_keyboard.c tests/test_battery.c \
          tests/test_wire.c tests/test_bump.c tests/test_ao.c \
          tests/demo.c tests/remote.c \
          $(APPNAME).c


# Link + compile in single pass (KPIT make.exe is too old for separate compile)
$(BUILDDIR)/$(APPNAME).out : $(SOURCES) $(APPNAME).ld makefile
	$(CC) $(ALL_FLAGS) -DROMSTART  -T$(APPNAME).ld -Xlinker  -Map -Xlinker $(BUILDDIR)/$(APPNAME).map -o $@ $(SOURCES)

$(APPNAME).bin : $(BUILDDIR)/$(APPNAME).out
	$(PREFIX)-objcopy -O binary $< $@

$(APPNAME).mot : $(BUILDDIR)/$(APPNAME).out
	$(PREFIX)-objcopy -O srec $< $@

rom: $(APPNAME).bin

clean:
	rm -rf $(BUILDDIR) $(APPNAME).bin $(APPNAME).mot

