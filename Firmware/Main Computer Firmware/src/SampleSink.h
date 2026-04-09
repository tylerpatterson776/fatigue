#pragma once

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <utility>

#include "Sample.h"

class SampleSink
{
public:
    virtual ~SampleSink() = default;
    virtual void handle(Sample s) = 0;
    virtual void flush() {}
};

class CSVFileSink: public SampleSink
{
public:
    CSVFileSink(QString filename) : m_filename(std::move(filename))
    {
        open();
    }
    ~CSVFileSink() override
    {
        close();
    }

    void handle(Sample s) override
    {
        if (!m_file.isOpen()) return;

        m_stream << s.timestamp_ms << "," << s.load_n << "," << s.distance_mm << "\n";
    }

    void flush() override
    {
        m_stream.flush();
    }

    [[nodiscard]] QString filename() const { return m_filename; }
private:
    void close()
    {
        m_file.close();
    }

    void open()
    {
        if (!QDir().mkpath("logs"))
        {
            throw std::runtime_error("Could not create logs directory");
        }
        m_file.setFileName("logs/" + m_filename);
        m_file.open(QIODevice::WriteOnly);
        if (!m_file.isOpen()) return;
        m_stream.setDevice(&m_file);
        m_stream << "time_ms,load_n,distance_mm\n";
    }

    QString m_filename;
    QFile m_file;
    QTextStream m_stream;

};