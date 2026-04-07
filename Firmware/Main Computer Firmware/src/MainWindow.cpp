#include "MainWindow.h"
#include "SerialWorker.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSerialPortInfo>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>
#include <QtCharts/QChart>

#include <algorithm>
#include <cmath>
#include <stdexcept>

MainWindow* MainWindow::m_instance = nullptr;

MainWindow::MainWindow()
{
    m_instance = this;
    setWindowTitle("Fatigue");
    resize(1100, 750);

    m_worker = new SerialWorker(this);
    connect(m_worker, &SerialWorker::sampleReceived,    this, &MainWindow::onSample);
    connect(m_worker, &SerialWorker::connectionChanged, this, &MainWindow::onConnectionChanged);
    connect(m_worker, &SerialWorker::errorOccurred,     this, &MainWindow::onWorkerError);

    construct_ui();
}

MainWindow::~MainWindow() = default;

MainWindow* MainWindow::instance()
{
    if (!m_instance)
        throw std::runtime_error("MainWindow::instance() called before MainWindow()");
    return m_instance;
}


// ReSharper disable once CppMemberFunctionMayBeStatic
QChartView* MainWindow::makeChart(const QString& title, const QString& yLabel,
                                  QLineSeries*& series, QValueAxis*& axisX, QValueAxis*& axisY)
{
    series = new QLineSeries;

    auto* chart = new QChart;
    chart->setTitle(title);
    chart->legend()->hide();
    chart->setAnimationOptions(QChart::NoAnimation); // Might change this
    chart->addSeries(series);

    axisX = new QValueAxis;
    axisX->setTitleText("Time (s)");
    axisX->setRange(0.0, 30.0);
    axisX->setLabelFormat("%.0f");
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    axisY = new QValueAxis;
    axisY->setTitleText(yLabel);
    axisY->setRange(-1.0, 1.0);   // will be auto-scaled on first sample
    axisY->setLabelFormat("%.2f");
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);

    auto* view = new QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);
    return view;
}

void MainWindow::construct_ui()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(6);

    // --- Connection bar ---
    auto* bar = new QHBoxLayout;
    bar->addWidget(new QLabel("Port:"));

    m_portCombo = new QComboBox;
    m_portCombo->setMinimumWidth(180);
    bar->addWidget(m_portCombo);

    auto* refreshBtn = new QPushButton("Refresh");
    connect(refreshBtn, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    bar->addWidget(refreshBtn);

    m_connectBtn = new QPushButton("Connect");
    connect(m_connectBtn, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
    bar->addWidget(m_connectBtn);
    bar->addStretch();
    root->addLayout(bar);

    // --- Charts ---
    auto* splitter = new QSplitter(Qt::Vertical);

    auto* loadView = makeChart("Load", "Load (N)",
                               m_loadSeries, m_loadAxisX, m_loadAxisY);
    auto* distView = makeChart("Distance", "Distance (mm)",
                               m_distSeries, m_distAxisX, m_distAxisY);

    splitter->addWidget(loadView);
    splitter->addWidget(distView);
    splitter->setSizes({450, 300});
    root->addWidget(splitter, 1);

    refreshPorts();
}

// Slots
void MainWindow::refreshPorts()
{
    const QString current = m_portCombo->currentText();
    m_portCombo->clear();
    for (const auto& info : QSerialPortInfo::availablePorts())
        m_portCombo->addItem(info.portName());
    const int idx = m_portCombo->findText(current);
    if (idx >= 0) m_portCombo->setCurrentIndex(idx);
}

void MainWindow::onConnectClicked()
{
    if (m_connected) {
        m_worker->close();
    } else {
        const QString port = m_portCombo->currentText();
        if (port.isEmpty()) return;
        m_worker->open(port);
    }
}

void MainWindow::onConnectionChanged(bool connected)
{
    m_connected = connected;
    m_connectBtn->setText(connected ? "Disconnect" : "Connect");
    if (!connected) m_t0 = -1.0;
}

void MainWindow::onWorkerError(const QString& msg)
{
    QMessageBox::critical(this, "Serial Error", msg);
}

void MainWindow::onSample(Sample s)
{
    const double t = s.timestamp_ms / 1000.0;
    if (m_t0 < 0.0) m_t0 = t;
    const double x = t - m_t0;

    m_loadSeries->append(x, s.load_n);
    m_distSeries->append(x, s.distance_mm);

    constexpr double kWindow = 30.0;
    const double xMin = std::max(0.0, x - kWindow);
    const double xMax = xMin + kWindow;

    auto pruneOld = [&](QLineSeries* ser) {
        const auto& pts = ser->points();
        int n = 0;
        for (const auto& p : pts) { if (p.x() < xMin) ++n; else break; }
        if (n > 0) ser->removePoints(0, n);
    };
    pruneOld(m_loadSeries);
    pruneOld(m_distSeries);

    m_loadAxisX->setRange(xMin, xMax);
    m_distAxisX->setRange(xMin, xMax);

    // Auto-scale Y to the visible window with a small margin
    auto scaleY = [](QLineSeries* ser, QValueAxis* axis) {
        const auto pts = ser->points();
        if (pts.isEmpty()) return;
        double lo = pts[0].y(), hi = pts[0].y();
        for (const auto& p : pts) {
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
        }
        const double margin = std::max((hi - lo) * 0.1, 0.5);
        axis->setRange(lo - margin, hi + margin);
    };
    scaleY(m_loadSeries, m_loadAxisY);
    scaleY(m_distSeries, m_distAxisY);
}
