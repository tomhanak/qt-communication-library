### Library settings
TARGET = QtCommunication
unix:VERSION = 1.0.0
DEFINES *= QTCOMMUNICATION_LIBRARY

### Available switches
#DEFINES *= QTCOMMUNICATION_SERIALPORT_NO_WARNINGS
#DEFINES *= QTCOMMUNICATION_SERIALPORT_NO_PORTABILTY_WARNINGS
#DEFINES *= QTCOMMUNICATION_SERIALPORT_MULTIPLATFORM_BAUD_RATES_ONLY
#DEFINES *= QTCOMMUNICATION_SERIALPORT_BAUD_RATES_UP_TO_256KBPS_ONLY

### General settings
TEMPLATE = lib
CONFIG -= debug_and_release
CONFIG += qt release warn_on thread
QT -= gui

DESTDIR = lib

### Directory with sources
SRC_DIR = QtCommunication


### Sources

PUBLIC_HEADERS += $${SRC_DIR}/serialport.h
PUBLIC_HEADERS += $${SRC_DIR}/QtCommunication_public.h
HEADERS += $${SRC_DIR}/serialport_p.h
SOURCES += $${SRC_DIR}/serialport.cpp

### Add public headers with other headers
HEADERS += $${PUBLIC_HEADERS}


### Installation
target.path = $$[QT_INSTALL_LIBS]
sources.files = $${PUBLIC_HEADERS}
sources.path = $$[QT_INSTALL_HEADERS]/QtCommunication
INSTALLS += target sources
