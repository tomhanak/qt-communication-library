### Library settings
TARGET = QtCommunication
unix:VERSION = 1.0.0

### Available switches
#DEFINES += QTCOMMUNICATION_SERIALPORT_NO_WARNINGS
#DEFINES += QTCOMMUNICATION_SERIALPORT_NO_PORTABILTY_WARNINGS
#DEFINES += QTCOMMUNICATION_SERIALPORT_MULTIPLATFORM_BAUD_RATES_ONLY
#DEFINES += QTCOMMUNICATION_SERIALPORT_BAUD_RATES_UP_TO_256KBPS_ONLY

### General settings
TEMPLATE = lib
CONFIG -= debug_and_release
CONFIG += qt release warn_on thread
QT -= gui

DESTDIR = lib
OBJECTS_DIR = _build_
MOC_DIR = _build_

### Directory with sources
SRC_DIR = QtCommunication


### Sources

PUBLIC_HEADERS += $${SRC_DIR}/serialport.h
HEADERS += $${SRC_DIR}/serialport_p.h
SOURCES += $${SRC_DIR}/serialport.cpp

### Add public headers with other headers
HEADERS += $$PWD/QtCommunication_public.h
HEADERS += $${PUBLIC_HEADERS}


### Installation
target.path = $$[QT_INSTALL_LIBS]
sources.files = $${PUBLIC_HEADERS}
sources.path = $$[QT_INSTALL_HEADERS]/communication
INSTALLS += target sources