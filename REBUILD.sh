#!/bin/bash

./CLEAN.sh

BUILD_DIR="_build_"
MAKE_ARGS="-j 5"

OLD_DIR=`pwd`
mkdir -p $BUILD_DIR
cd $BUILD_DIR

qmake ../QtCommunication.pro -o Makefile.QtCommunication && \
    make $MAKE_ARGS -f Makefile.QtCommunication && \
    qmake ../tests/auto/test.pro -o Makefile.test_auto && \
    make $MAKE_ARGS -f Makefile.test_auto && \
    qmake ../tests/console/test.pro -o Makefile.test_console && \
    make $MAKE_ARGS -f Makefile.test_console

cd $OLD_DIR
