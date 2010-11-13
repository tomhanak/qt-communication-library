#!/bin/bash

./CLEAN.sh

BUILD_DIR="_build_"
MAKE_ARGS="-j 5"

OLD_DIR=`pwd`
mkdir -p $BUILD_DIR
cd $BUILD_DIR

qmake ../QtCommunication.pro -o Makefile.QtCommunication && \
    make $MAKE_ARGS -f Makefile.QtCommunication && \
    qmake ../tests/console/test.pro -o Makefile.test && \
    make $MAKE_ARGS -f Makefile.test

cd $OLD_DIR
