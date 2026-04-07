#include "SerialWorker.h"

SerialWorker::SerialWorker(QObject* parent) : QObject(parent) {}

SerialWorker::~SerialWorker() { close(); }

void SerialWorker::open(const QString& portName)
{
    close();

    m_port = new QSerialPort(this);
    m_port->setPortName(portName);
    m_port->setBaudRate(921600);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);

    connect(m_port, &QSerialPort::readyRead, this, &SerialWorker::onReadyRead);
    connect(m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
        if (e != QSerialPort::NoError)
            emit errorOccurred(m_port->errorString());
    });

    if (m_port->open(QIODevice::ReadOnly)) {
        emit connectionChanged(true);
    } else {
        emit errorOccurred(m_port->errorString());
        m_port->deleteLater();
        m_port = nullptr;
    }
}

void SerialWorker::close()
{
    if (!m_port) return;
    m_port->close();
    m_port->deleteLater();
    m_port = nullptr;
    m_lineBuf.clear();
    emit connectionChanged(false);
}

void SerialWorker::onReadyRead()
{
    m_lineBuf += m_port->readAll();

    int idx;
    while ((idx = m_lineBuf.indexOf('\n')) != -1) {
        const QByteArray line = m_lineBuf.left(idx).trimmed();
        m_lineBuf.remove(0, idx + 1);
        if (line.isEmpty()) continue;
        if (auto s = Sample::parse(line.toStdString()))
            emit sampleReceived(*s);
    }
}
