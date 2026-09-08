#pragma once

#include <QByteArray>
#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include "Sample.h"

class SerialWorker : public QObject
{
    Q_OBJECT

public:
    explicit SerialWorker(QObject* parent = nullptr);
    ~SerialWorker() override;

public slots:
    void open(const QString& portName, qint32 baudRate = 921600);
    void close();

signals:
    void sampleReceived(Sample s);
    void connectionChanged(bool connected);
    void errorOccurred(const QString& msg);
    void streamStalled();
    void streamResumed();

private slots:
    void onReadyRead();

private:
    void reportInvalidSample();

    QSerialPort* m_port = nullptr;
    QByteArray m_lineBuf;
    QTimer m_silenceTimer;
    bool m_stalled = false;
    bool m_invalidReported = false;
    bool m_discardingLine = false;
};
