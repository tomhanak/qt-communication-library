#include <QtCore/QDebug>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QTime>

#include <QtCommunication/serialport.h>

int main(int /*argc*/, char **/*argv*/)
{
    using namespace qcl;

    QTextStream cout(stdout);
    qDebug() << "START";
    qDebug();

#ifdef Q_WS_WIN
    QString portName = "COM1";
#else
#ifdef Q_WS_MAC
    //QString portName = "/dev/tty.Bluetooth-Modem";
    QString portName = "/dev/cu.Bluetooth-Modem";
#else
    //QString strPort = "/dev/ttyUSB0";
    QString portName = "/dev/ttyS0";
#endif
#endif

    SerialPort port(portName);

    if (!port.open(QIODevice::ReadWrite | QIODevice::Text)) {
        qDebug() << "!!! Port" << port.portName() << "nebyl otevren !!!";
        qDebug() << "Error:" << port.lastError();
        return -1;
    }
    port.setBaudRateFromNumber(9600);
    //port.setBaudRate(SerialPort::BaudRate_9600);
    port.setDataBits(SerialPort::EightDataBits);
    port.setParity(SerialPort::NoParity);
    port.setStopBits(SerialPort::OneStopBit);
    port.setFlowControl(SerialPort::NoFlowControl);
    port.setAutoFlushOnWrite(true);

    qDebug() << "SerialPort Settings:";
    qDebug() << "  Port name          :" << port.portName();
    qDebug() << "  Baud rate          :" << port.baudRate();
    qDebug() << "  Data bits          :" << port.dataBits();
    qDebug() << "  Parity             :" << port.parity();
    qDebug() << "  Stop bits          :" << port.stopBits();
    qDebug() << "  Flow control       :" << port.flowControl();
    qDebug() << "  Auto flush on write:" << port.autoFlushOnWrite();
    qDebug() << "  Status             :" << port.lineStatus();
    qDebug();

    qDebug() << "TEST SerialPort";
    qDebug();

//    qDebug() << qPrintable(QTime::currentTime().toString("hh:mm:ss.zzz"))
//            << "- Zapsano:" << port.write("Testovaci retezec") << "bajtu";

//    for (int n = 0; n < 20; ++n) {
//        if (port.read(1).count() == 1) {
//            qDebug() << qPrintable(QTime::currentTime().toString("hh:mm:ss.zzz"))
//                    << "  - precten" << n + 1 << "bajt";
//        }
//    }

//    qDebug() << qPrintable(QTime::currentTime().toString("hh:mm:ss.zzz"))
//            << "- Precteno:" << port.readAll().count() << "bajtu";


    const int count = 960;
    QByteArray data;
    data.fill(0x5A, count); // 0101 1010 bin
    qDebug() << count << "bajtu k prenosu";

    QTime start;
    QTime end;


    start = QTime::currentTime();
    do {

        if (port.write(data) != count) {
            qDebug() << "Problem pri zapisu";
            qDebug() << "Error:" << port.lastError();
            port.clearLastError();
            break;
        }
//        if (!port.waitForBytesWritten(count * 10)) {
//            qDebug() << "Timeout pri zapisu";
//            break;
//        }

        if (port.read(count).count() != count) {
            qDebug() << "Problem pri cteni";
            qDebug() << "Error:" << port.lastError();
            port.clearLastError();
            break;
        }
//        if (!port.waitForReadyRead(count * 10)) {
//            qDebug() << "Timeout pri cteni";
//            break;
//        }

    } while (false);
    end = QTime::currentTime();
    qDebug() << "Doba prenosu bloku dat:" << start.msecsTo(end) << "ms";


    start = QTime::currentTime();
    for (int n = 0; n < count; ++n) {

        char c = n % 10 + 'A';
        if (port.write(&c, 1) != 1) {
            qDebug() << "Problem pri zapisu, bajt" << n + 1;
            qDebug() << "Error:" << port.lastError();
            port.clearLastError();
            break;
        }
//        if (!port.waitForBytesWritten(100)) {
//            qDebug() << "Timeout pri zapisu";
//            break;
//        }

//        if (!port.waitForReadyRead(100)) {
//            qDebug() << "Timeout pri cteni";
//            break;
//        }
        if (port.read(1).count() != 1) {
            qDebug() << "Problem pri cteni, bajt" << n + 1;
            qDebug() << "Error:" << port.lastError();
            port.clearLastError();
            break;
        }

    }
    end = QTime::currentTime();
    qDebug() << "Doba prenosu bajt po bajtu:" << start.msecsTo(end) << "ms";


    port.close();
    qDebug() << "Error:" << port.lastError();

    qDebug();
    qDebug() << "KONEC";
    return 0;
}
