### Library settings
TARGET = test_serialport

### General settings
TEMPLATE = app
QT -= gui
win32:CONFIG += console
macx:CONFIG -= app_bundle

DESTDIR = bin

### Dependencies
include(../../QtCommunication.pri)
macx {
    QMAKE_LFLAGS_SONAME += -Wl,-install_name,@executable_path/../lib/
    QMAKE_LFLAGS_SONAME += -Wl,-install_name,@executable_path/../../../lib/
} else:linux-* {
    QMAKE_LFLAGS += -Wl,-z,origin \'-Wl,-rpath,\$\$ORIGIN/../lib\'
    QMAKE_LFLAGS += -Wl,-z,origin \'-Wl,-rpath,\$\$ORIGIN/../../../lib\'
}

### Sources

SOURCES += main.cpp
