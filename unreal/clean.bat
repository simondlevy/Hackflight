::
:: clean.bat Windows cleanup script for MulticopterSim
::
:: Copyright (C) 2020 Simon D. Levy
::
:: MIT License

::rm -rf WindowsNoEditor *.txt Makefile *.sln *.pri *.kdev4 *.pro *.*workspace .vs/ Build/ Binaries/ DerivedDataCache/ Intermediate/ Saved/

del /f /q *.txt Makefile *.sln *.pri *.kdev4 *.pro *.*workspace > NUL

del /f /q Binaries\Win64\*.target > NUL
del /f /q Binaries\Win64\UE4* > NUL

rmdir /q /s .vs Build DerivedDataCache Intermediate Saved > NUL:q
