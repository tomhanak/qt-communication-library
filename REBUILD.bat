set MAKE="mingw32-make"
rem set MAKE="nmake"

rem set MAKE_ARGS=
set MAKE_ARGS="-j5"

set BUILD_DIR="_build_"
::-----

call .\CLEAN.bat

if not exist "%BUILD_DIR%" md "%BUILD_DIR%"
pushd "%BUILD_DIR%"

qmake ../QtCommunication.pro -o Makefile.QtCommunication && (
    %MAKE% %MAKE_ARGS% -f Makefile.QtCommunication && (
    qmake ../tests/console/test.pro -o Makefile.test && (
    %MAKE% %MAKE_ARGS% -f Makefile.test
)))

popd "%OLD_DIR%"
