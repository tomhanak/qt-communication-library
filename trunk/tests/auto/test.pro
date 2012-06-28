### Library settings
TARGET = test_auto

### General settings
TEMPLATE = app
QT -= gui
QT += testlib
CONFIG += console
CONFIG -= app_bundle

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

HEADERS += test_serialport.h
SOURCES += test_serialport.cpp

SOURCES += main_auto.cpp
