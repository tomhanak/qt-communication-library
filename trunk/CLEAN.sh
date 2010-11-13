#!/bin/bash

make distclean 2> /dev/null

DIR="."

rm -r "$DIR/_build_" 2> /dev/null
rm -r "$DIR/lib" 2> /dev/null

find $DIR -name "Makefile*" | xargs rm 2> /dev/null
find $DIR -name "*.o" | xargs rm 2> /dev/null
find $DIR -name "*.moc" | xargs rm 2> /dev/null
find $DIR -name "moc_*.cpp" | xargs rm 2> /dev/null
find $DIR -name "ui_*.h" | xargs rm 2> /dev/null
find $DIR -name "qrc_*.h" | xargs rm 2> /dev/null
