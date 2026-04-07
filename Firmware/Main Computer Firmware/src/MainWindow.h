#pragma once

#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "Sample.h"

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

private:
    void        construct_ui();
    QChartView* makeChart(const QString& title, const QString& yLabel,
                          QLineSeries*& series, QValueAxis*& axisX, QValueAxis*& axisY);

    static MainWindow* m_instance;

    // Toolbar
    QComboBox*   m_portCombo  = nullptr;
    QPushButton* m_connectBtn = nullptr;

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

    double m_t0 = -1.0;  // time of first sample in seconds
};
