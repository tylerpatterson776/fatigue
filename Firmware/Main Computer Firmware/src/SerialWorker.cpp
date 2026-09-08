#include "SerialWorker.h"

#include <utility>

SerialWorker::SerialWorker(QObject* parent) : QObject(parent), m_silenceTimer(this)
{
    m_silenceTimer.setSingleShot(true);
    m_silenceTimer.setInterval(5000);
    connect(&m_silenceTimer, &QTimer::timeout, this, [this] {
        if (!m_port)
            return;
        m_stalled = true;
        emit streamStalled();
        emit errorOccurred("No valid samples have been received for 5 seconds. Check the Pi and serial connection.");
    });
}

SerialWorker::~SerialWorker()
{
    close();
}

void SerialWorker::open(const QString& portName, qint32 baudRate)
{
    close();

    m_port = new QSerialPort(this);
    m_port->setPortName(portName);
    m_port->setBaudRate(baudRate);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);
    m_port->setReadBufferSize(65536);

    if (m_port->open(QIODevice::ReadOnly)) {
        connect(m_port, &QSerialPort::readyRead, this, &SerialWorker::onReadyRead);
        connect(m_port, &QSerialPort::errorOccurred, this, [this, port = m_port](QSerialPort::SerialPortError e) {
            if (m_port != port || e == QSerialPort::NoError)
                return;
            const auto message = port->errorString();
            if (e != QSerialPort::TimeoutError)
                close();
            emit errorOccurred(message);
        });
        m_silenceTimer.start();
        emit connectionChanged(true);
    } else {
        const auto message = m_port->errorString();
        close();
        emit errorOccurred(message);
    }
}

void SerialWorker::close()
{
    m_silenceTimer.stop();
    m_lineBuf.clear();
    m_stalled = false;
    m_invalidReported = false;
    m_discardingLine = false;
    auto* port = std::exchange(m_port, nullptr);
    if (!port)
        return;
    disconnect(port, nullptr, this, nullptr);
    port->close();
    port->deleteLater();
    emit connectionChanged(false);
}

void SerialWorker::onReadyRead()
{
    if (!m_port)
        return;
    auto* port = m_port;
    const auto bytes = port->readAll();
    if (m_port != port)
        return;
    m_lineBuf += bytes;

    int idx;
    while ((idx = m_lineBuf.indexOf('\n')) != -1) {
        const QByteArray line = m_lineBuf.left(idx).trimmed();
        m_lineBuf.remove(0, idx + 1);
        if (m_discardingLine || line.size() > 65536) {
            m_discardingLine = false;
            reportInvalidSample();
            continue;
        }
        if (line.isEmpty())
            continue;
        if (auto s = Sample::parse(line.toStdString())) {
            m_silenceTimer.start();
            if (m_stalled) {
                m_stalled = false;
                emit streamResumed();
                if (m_port != port)
                    return;
            }
            emit sampleReceived(*s);
            if (m_port != port)
                return;
        } else {
            reportInvalidSample();
        }
    }
    if (m_lineBuf.size() > 65536) {
        m_lineBuf.clear();
        m_discardingLine = true;
        reportInvalidSample();
    }
}

void SerialWorker::reportInvalidSample()
{
    if (m_invalidReported)
        return;
    m_invalidReported = true;
    emit errorOccurred("Received an invalid sample. Check that the Pi firmware matches this application.");
}
