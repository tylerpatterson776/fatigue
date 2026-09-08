#pragma once

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QFileInfo>
#include <QLocale>
#include <limits>
#include <stdexcept>
#include <utility>

#include "Sample.h"

class SampleSink
{
public:
    virtual ~SampleSink() = default;
    virtual void handle(Sample s) = 0;
    virtual void flush() {}
};

class CSVFileSink : public SampleSink
{
public:
    CSVFileSink(QString filename) : m_filename(std::move(filename))
    {
        open();
    }
    ~CSVFileSink() override
    {
        try {
            flush();
        } catch (...) {
        }
        m_file.close();
    }

    void handle(Sample s) override
    {
        m_stream.setRealNumberPrecision(std::numeric_limits<float>::max_digits10);
        m_stream << s.timestamp_ms << "," << s.load_n << "," << s.distance_mm << ",";
        m_stream.setRealNumberPrecision(std::numeric_limits<double>::max_digits10);
        m_stream << s.cycle << "\n";
        checkWrite();
    }

    void flush() override
    {
        m_stream.flush();
        checkWrite();
        if (!m_file.flush())
            throw std::runtime_error(
                QString("Could not flush %1: %2").arg(m_filename, m_file.errorString()).toStdString());
    }

    [[nodiscard]] QString filename() const
    {
        return m_filename;
    }

private:
    void checkWrite()
    {
        if (!m_file.isOpen() || m_stream.status() != QTextStream::Ok || m_file.error() != QFileDevice::NoError)
            throw std::runtime_error(
                QString("Could not write %1: %2").arg(m_filename, m_file.errorString()).toStdString());
    }

    void open()
    {
        if (!QDir().mkpath("logs")) {
            throw std::runtime_error("Could not create logs directory");
        }
        const auto requested = m_filename;
        const auto dot = requested.lastIndexOf('.');
        const auto base = dot < 0 ? requested : requested.left(dot);
        const auto extension = dot < 0 ? QString() : requested.mid(dot);
        for (unsigned int suffix = 0;; ++suffix) {
            m_filename = suffix == 0 ? requested : base + "_" + QString::number(suffix) + extension;
            m_file.setFileName("logs/" + m_filename);
            if (m_file.open(QIODevice::WriteOnly | QIODevice::NewOnly))
                break;
            if (!QFileInfo::exists(m_file.fileName()))
                throw std::runtime_error(
                    QString("Could not open %1: %2").arg(m_filename, m_file.errorString()).toStdString());
        }
        m_stream.setDevice(&m_file);
        m_stream.setLocale(QLocale::c());
        m_stream.setRealNumberPrecision(std::numeric_limits<double>::max_digits10);
        m_stream << "time_ms,load_n,distance_mm,cycle\n";
        flush();
    }

    QString m_filename;
    QFile m_file;
    QTextStream m_stream;
};
