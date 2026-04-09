#pragma once

#include <limits>
#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QTimer>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "Sample.h"
#include "SampleSink.h"

class QComboBox;
class QPushButton;
class SerialWorker;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow() override;

    static MainWindow* instance();

private slots:
    void onConnectClicked();
    void onSample(Sample s);
    void onConnectionChanged(bool connected);
    void onWorkerError(const QString& msg);
    void refreshPorts();
    void onLogToggled();
    void onLogTimer() const;

private:
    void        construct_ui();
    QChartView* makeChart(const QString& title, const QString& yLabel,
                          QLineSeries*& series, QValueAxis*& axisX, QValueAxis*& axisY);

    static MainWindow* m_instance;

    // Logging
    QTimer m_logTimer;
    std::vector<std::unique_ptr<SampleSink>> m_sinks;

    // Toolbar
    QComboBox*   m_portCombo  = nullptr;
    QPushButton* m_connectBtn = nullptr;
    QPushButton* m_logBtn     = nullptr;
    bool         m_logging    = false;

    // Load chart
    QLineSeries* m_loadSeries = nullptr;
    QValueAxis*  m_loadAxisX  = nullptr;
    QValueAxis*  m_loadAxisY  = nullptr;

    // Distance chart
    QLineSeries* m_distSeries = nullptr;
    QValueAxis*  m_distAxisX  = nullptr;
    QValueAxis*  m_distAxisY  = nullptr;

    // Serial
    SerialWorker* m_worker    = nullptr;
    bool          m_connected = false;

    double m_t0       = -1.0;
    double m_lastLoad = std::numeric_limits<double>::quiet_NaN();
    double m_lastDist = std::numeric_limits<double>::quiet_NaN();
};
