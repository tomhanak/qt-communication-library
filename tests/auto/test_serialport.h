#include <QtCore/QObject>

namespace qcl {
    class SerialPort;
}

class TestSerialPort : public QObject
{
    Q_OBJECT

public:
    TestSerialPort(QObject *parent = 0);
    virtual ~TestSerialPort();

private Q_SLOTS: // Predefined methods
    void init();
    void cleanup();
    void initTestCase();
    void cleanupTestCase();

private Q_SLOTS: // Test methods
    void portName();

private:
    qcl::SerialPort *m_serial;
};
