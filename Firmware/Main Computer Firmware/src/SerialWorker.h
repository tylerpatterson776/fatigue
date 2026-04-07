#pragma once

#include <QByteArray>
#include <QObject>
#include <QSerialPort>
#include "Sample.h"

class SerialWorker : public QObject {
    Q_OBJECT

public:
    explicit SerialWorker(QObject* parent = nullptr);
    ~SerialWorker() override;

public slots:
    void open(const QString& portName);
    void close();

signals:
    void sampleReceived(Sample s);
    void connectionChanged(bool connected);
    void errorOccurred(const QString& msg);

private slots:
    void onReadyRead();

private:
    QSerialPort* m_port    = nullptr;
    QByteArray   m_lineBuf;
};
