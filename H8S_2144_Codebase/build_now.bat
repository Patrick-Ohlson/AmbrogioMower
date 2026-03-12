@Echo On
set location=F:\Dropbox\Robotic\GNUH8
set workdir=F:\Dropbox\Robotic\GNUH8\2021_GNUH8_Mower\BASE_L200_2013_REMOTE - Claude\STATES\active_object
Set PATH=%location%\h8300-elf\bin;%location%\h8300-elf\h8300-elf\bin;%location%\Other;%location%\h8300-elf\libexec\gcc\h8300-elf\4.7-GNUH8_v12.02;%PATH%
cd /d "%workdir%"

echo Assembling start.S...
h8300-elf-gcc -ms -ms2600 -DROMSTART -c -o start.o start.S
if errorlevel 1 (echo FAILED start.S && goto :done)

echo Compiling vects.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -c -o vects.o vects.c
if errorlevel 1 (echo FAILED vects.c && goto :done)

echo Compiling test_framework.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -DROMSTART -c -o test_framework.o test_framework.c
if errorlevel 1 (echo FAILED test_framework.c && goto :done)

echo Compiling hsm.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -DROMSTART -c -o hsm.o hsm.c
if errorlevel 1 (echo FAILED hsm.c && goto :done)

echo Compiling schedulerrt.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -DROMSTART -c -o schedulerrt.o schedulerrt.c
if errorlevel 1 (echo FAILED schedulerrt.c && goto :done)

echo Compiling smallheap.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -DROMSTART -c -o smallheap.o smallheap.c
if errorlevel 1 (echo FAILED smallheap.c && goto :done)

echo Compiling queue.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -DROMSTART -c -o queue.o queue.c
if errorlevel 1 (echo FAILED queue.c && goto :done)

echo Compiling error.c...
h8300-elf-gcc -O0 -ggdb -femit-class-debug-always -DNDEBUG -ms -ms2600 -DROMSTART -c -o error.o error.c
if errorlevel 1 (echo FAILED error.c && goto :done)

echo Linking main.out...
h8300-elf-gcc -nostartfiles -ms -ms2600 -Tmain.ld -Xlinker -Map -Xlinker main.map -o main.out start.o vects.o test_framework.o hsm.o schedulerrt.o smallheap.o queue.o error.o
if errorlevel 1 (echo FAILED linking && goto :done)

echo Creating main.bin...
h8300-elf-objcopy -O binary main.out main.bin
F:\Dropbox\Robotic\GNUH8\Other\srec_cat main.bin -binary -o main.mot

echo BUILD OK

:done
