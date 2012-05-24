set MAKE="mingw32-make"
rem set MAKE="nmake"

set BUILD_DIR="_build_"
::-----

%MAKE% distclean 2> nul

set DIR="."

rd /S /Q "%DIR%\%BUILD_DIR%" 2> nul
rd /S /Q "%DIR%\lib" 2> nul
rd /S /Q "%DIR%\bin" 2> nul

del /F /S /Q "Makefile*" 2> nul
del /F /S /Q "*.o" 2> nul
del /F /S /Q "*.moc" 2> nul
del /F /S /Q "moc_*.cpp" 2> nul
del /F /S /Q "ui_*.h" 2> nul
del /F /S /Q "*qrc_*.h" 2> nul
