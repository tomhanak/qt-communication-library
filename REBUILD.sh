#!/bin/bash

make distclean 2> /dev/null

qmake && time make
