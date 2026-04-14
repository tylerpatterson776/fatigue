#pragma once

#include <limits>
#include <QMainWindow>
#include <QTimer>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "Sample.h"
#include "SampleSink.h"
#include "FrequencyEstimator.h"
#include "HoverChartView.h"

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
    void             construct_ui();
    HoverChartView*  makeChart(const QString& title, const QString& yLabel,
                               QLineSeries*& series, QValueAxis*& axisX, QValueAxis*& axisY,
                               QLineSeries*& minLine, QLineSeries*& maxLine);

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
    HoverChartView* m_loadView    = nullptr;
    QLineSeries*    m_loadSeries  = nullptr;
    QLineSeries*    m_loadMinLine = nullptr;
    QLineSeries*    m_loadMaxLine = nullptr;
    QValueAxis*     m_loadAxisX   = nullptr;
    QValueAxis*     m_loadAxisY   = nullptr;
    FrequencyEstimator m_loadEstimator;

    // Distance chart
    HoverChartView* m_distView    = nullptr;
    QLineSeries*    m_distSeries  = nullptr;
    QLineSeries*    m_distMinLine = nullptr;
    QLineSeries*    m_distMaxLine = nullptr;
    QValueAxis*     m_distAxisX   = nullptr;
    QValueAxis*     m_distAxisY   = nullptr;
    FrequencyEstimator m_distEstimator;

    // Serial
    SerialWorker* m_worker    = nullptr;
    bool          m_connected = false;

    double m_t0     = -1.0;
    double m_tStart = -1.0;
    double m_lastLoad = std::numeric_limits<double>::quiet_NaN();
    double m_lastDist = std::numeric_limits<double>::quiet_NaN();
};
