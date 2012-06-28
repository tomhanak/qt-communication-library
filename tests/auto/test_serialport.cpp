#include "test_serialport.h"

#include <QtCore/QDebug>
#include <QtTest/QTest>

#include <QtCommunication/serialport.h>

using namespace qcl;

/*******************************************************************************
    TestSerialPort - constructor/destructor
*******************************************************************************/

TestSerialPort::TestSerialPort(QObject *parent)
    : QObject(parent),
    m_serial(0)
{
}

TestSerialPort::~TestSerialPort()
{
    delete m_serial;
    m_serial = 0;
}

/*******************************************************************************
    TestSerialPort - predefined test methods
*******************************************************************************/

void TestSerialPort::init()
{
    m_serial = new SerialPort;
}

void TestSerialPort::cleanup()
{
    delete m_serial;
    m_serial = 0;
}

void TestSerialPort::initTestCase()
{
}

void TestSerialPort::cleanupTestCase()
{
}

/*******************************************************************************
    TestSerialPort - custom test methods
*******************************************************************************/

void TestSerialPort::portName()
{
#ifdef Q_WS_WIN
    m_serial->setPortName("COM1");
    QVERIFY2(m_serial->portName() == "COM1", "Entered port name: COM1");
    m_serial->setPortName("\\\\.\\COM1");
    QVERIFY2(m_serial->portName() == "COM1", "Entered port name: \\\\.\\COM1");
#else
    m_serial->setPortName("/ttyS0");
    QVERIFY2(m_serial->portName() == "/ttyS0", "Entered port name: /ttyS0");
    m_serial->setPortName("ttyS0");
    QVERIFY2(m_serial->portName() == "/dev/ttyS0", "Entered port name: ttyS0");
    m_serial->setPortName("/dev/ttyS0");
    QVERIFY2(m_serial->portName() == "/dev/ttyS0",
            "Entered port name: /dev/ttyS0");
#endif
}
