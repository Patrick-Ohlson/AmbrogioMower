#!/bin/bash
#============================================================================
# build.sh — Build TestFirmware with all intermediates in .build/
#
# Usage:
#   ./build.sh          Build main.bin (separate compile + link)
#   ./build.sh clean    Remove .build/ and main.bin
#   ./build.sh temps    Build with -save-temps (puts .i/.s in .build/)
#
# This script provides separate compile→link with .build/ output that
# the KPIT make.exe (3.81) cannot do reliably on Windows.
#============================================================================

set -e

BUILDDIR=".build"
PREFIX="h8300-elf"
CC="${PREFIX}-gcc"
CPU="-ms -ms2600"
CFLAGS="-O0 -ggdb"
APPNAME="main"

# Add toolchain to PATH if not already there
export PATH="/F/Dropbox/Robotic/GNUH8/h8300-elf/bin:$PATH"

# Source files (same list as Makefile)
SOURCES=(
    start.S vects.c sci.c hwinit.c
    timer/timer.c lcd/lcd.c misc/misc.c i2c/i2c.c rtc/rtc.c motor/motor.c
    steering/steering.c
    utils/utils.c
    eeprom/eeprom.c eeprom/config.c
    accel/accel.c
    keyboard/keyboard.c
    battery/battery.c
    wire/wire.c
    bump/bump.c
    active_object/smallheap.c active_object/queue.c active_object/hsm.c
    active_object/schedulerrt.c active_object/error.c active_object/ao.c
    tests/test_motor.c tests/test_drive.c tests/test_i2c.c tests/test_rtc.c
    tests/test_memory.c tests/test_misc.c tests/test_lcd.c
    tests/test_timer.c tests/test_sci.c tests/test_statics.c
    tests/test_steering.c tests/test_eeprom.c tests/test_accel.c
    tests/test_keyboard.c tests/test_battery.c
    tests/test_wire.c tests/test_bump.c tests/test_ao.c
    tests/demo.c
    ${APPNAME}.c
)

# Handle arguments
case "$1" in
    clean)
        echo "Cleaning..."
        rm -rf "${BUILDDIR}" ${APPNAME}.bin ${APPNAME}.mot
        echo "Done."
        exit 0
        ;;
    temps)
        SAVE_TEMPS="-save-temps=obj"
        echo "Building with -save-temps (intermediates in ${BUILDDIR}/)"
        ;;
    *)
        SAVE_TEMPS=""
        ;;
esac

# Create build directory structure
echo "=== Build: creating directories ==="
for src in "${SOURCES[@]}"; do
    dir=$(dirname "${BUILDDIR}/${src}")
    mkdir -p "$dir"
done

# Compile each source to .build/path/file.o
echo "=== Build: compiling ${#SOURCES[@]} files ==="
OBJS=()
for src in "${SOURCES[@]}"; do
    obj="${BUILDDIR}/${src%.*}.o"
    OBJS+=("$obj")

    # Skip if object is newer than source
    if [ "$obj" -nt "$src" ] && [ "$obj" -nt "${APPNAME}.ld" ]; then
        continue
    fi

    echo "  CC  $src"
    $CC $SAVE_TEMPS $CFLAGS $CPU -DROMSTART -c -o "$obj" "$src"
done

# Link
echo "=== Build: linking ==="
$CC -nostartfiles $CPU -DROMSTART \
    -T${APPNAME}.ld \
    -Xlinker -Map -Xlinker ${BUILDDIR}/${APPNAME}.map \
    -o ${BUILDDIR}/${APPNAME}.out \
    "${OBJS[@]}"

# Binary
echo "=== Build: objcopy ==="
${PREFIX}-objcopy -O binary ${BUILDDIR}/${APPNAME}.out ${APPNAME}.bin

echo "=== Build complete: ${APPNAME}.bin ==="
ls -la ${APPNAME}.bin
