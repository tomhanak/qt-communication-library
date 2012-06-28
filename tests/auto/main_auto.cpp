#include <QtCore/QDebug>
#include <QtTest/QTest>

#include "test_serialport.h"

int main(int argc, char *argv[])
{
    int retVal = 0;

    qDebug() << "Starting automated tests";
    qDebug();

    TestSerialPort testSerialPort;
    retVal += QTest::qExec(&testSerialPort, argc, argv);

    qDebug();
    qDebug() << "Automated tests finished";

    return retVal;
}
