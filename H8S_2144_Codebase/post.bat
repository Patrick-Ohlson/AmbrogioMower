@Echo Off
echo POST
set location=F:\Dropbox\Robotic\GNUH8

rem Set PATH=%CD%\h8300-elf\bin;%CD%\h8300-elf\h8300-elf\bin;%CD%\Other;%CD%\h8300-elf\libexec\gcc\h8300-elf\4.7-GNUH8_v12.02;%PATH%
if defined H8_PATH_SET goto :path_done
Set PATH=%location%\h8300-elf\bin;%location%\h8300-elf\h8300-elf\bin;%location%\Other;%location%\h8300-elf\libexec\gcc\h8300-elf\4.7-GNUH8_v12.02;%PATH%
set H8_PATH_SET=1
:path_done
Set MAKE_MODE=unix
python version.py
if exist *.o del /Q *.o
if exist *.mot del /Q *.mot
if exist main.bin del /Q main.bin
if exist .build\main.out del /Q .build\main.out
make rom
if errorlevel 1 goto :error

rem Generate .mot from freshly-built main.bin (objcopy output)
F:\Dropbox\Robotic\GNUH8\Other\srec_cat main.bin -binary -o main.mot
copy .build\main.out ..\build\main.out

echo. && echo === Symbols ===
nm .build\main.out | findstr /i "EndCode SetupHW patched_boot endchar byte_180E8"
echo.
echo --- Signals ---
nm .build\main.out | findstr /i "sig_motor sig_wire sig_timeout"
echo.
echo --- AO Framework ---
nm .build\main.out | findstr /i "ao_motor ao_wire pool_Resolve pool_Alloc hsm_Dispatch ao_Scheduler ao_ProcessQueue"
echo.
echo --- ROM usage ---
h8300-elf-size .build\main.out
nm .build\main.out | findstr EndCode > endcode.txt

echo.
goto :done

:error
echo.
echo *** BUILD FAILED ***
echo.

:done
rem pause
