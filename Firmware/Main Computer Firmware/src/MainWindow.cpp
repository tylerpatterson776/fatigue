#include "MainWindow.h"
#include "SerialWorker.h"

#include <QComboBox>
#include <QDateTime>
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
#include <limits>
#include <stdexcept>

static constexpr double kWindow = 5.0;

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

    m_logTimer.setSingleShot(false);
    connect(&m_logTimer, &QTimer::timeout, this, &MainWindow::onLogTimer);
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
    chart->setAnimationOptions(QChart::NoAnimation);
    chart->addSeries(series);

    axisX = new QValueAxis;
    axisX->setTitleText("Time (s)");
    axisX->setRange(0.0, kWindow);
    axisX->setLabelFormat("%.0f");
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    axisY = new QValueAxis;
    axisY->setTitleText(yLabel);
    axisY->setRange(-1.0, 1.0);
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

    m_logBtn = new QPushButton("Start Log");
    connect(m_logBtn, &QPushButton::clicked, this, &MainWindow::onLogToggled);
    bar->addWidget(m_logBtn);
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
    if (!connected) {
        m_t0       = -1.0;
        m_lastLoad = std::numeric_limits<double>::quiet_NaN();
        m_lastDist = std::numeric_limits<double>::quiet_NaN();
        m_loadSeries->clear();
        m_distSeries->clear();
    }
}

void MainWindow::onWorkerError(const QString& msg)
{
    QMessageBox::critical(this, "Serial Error", msg);
}

void MainWindow::onLogToggled()
{
    if (!m_logging) {
        const QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
        const QString filename = QString("fatigue_%1.csv").arg(ts);
        m_sinks.push_back(std::make_unique<CSVFileSink>(filename));
        m_logTimer.start(500);
        m_logging = true;
        m_logBtn->setText("Stop Log");
    } else {
        m_logTimer.stop();
        m_sinks.clear();
        m_logging = false;
        m_logBtn->setText("Start Log");
    }
}

void MainWindow::onLogTimer() const
{
    for (const auto& sink : m_sinks)
        sink->flush();
}

void MainWindow::onSample(Sample s)
{
    if (m_logging)
        for (const auto& sink : m_sinks)
            sink->handle(s);

    const double t = s.timestamp_ms / 1000.0;
    if (m_t0 < 0.0) m_t0 = t;
    const double x = t - m_t0;

    if (x >= kWindow) {
        m_t0 = t;
        m_loadSeries->clear();
        m_distSeries->clear();
        m_loadSeries->append(0.0, s.load_n);
        m_distSeries->append(0.0, s.distance_mm);
        return;
    }

    constexpr double kLoadThresh = 1.0;   // N
    constexpr double kDistThresh = 0.001;  // mm

    const bool loadChanged = std::isnan(m_lastLoad) || std::abs(s.load_n      - m_lastLoad) > kLoadThresh;
    const bool distChanged = std::isnan(m_lastDist) || std::abs(s.distance_mm - m_lastDist) > kDistThresh;
    if (!loadChanged && !distChanged) return;

    if (loadChanged) { m_loadSeries->append(x, s.load_n);      m_lastLoad = s.load_n; }
    if (distChanged) { m_distSeries->append(x, s.distance_mm); m_lastDist = s.distance_mm; }

    auto scaleY = [](const QLineSeries* ser, QValueAxis* axis, const double marginFactor) {
        const auto pts = ser->points();
        if (pts.isEmpty()) return;
        double lo = pts[0].y(), hi = pts[0].y();
        for (const auto& p : pts) {
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
        }
        const double margin = std::max((hi - lo) * marginFactor, 0.5);
        axis->setRange(lo - margin, hi + margin);
    };
    scaleY(m_loadSeries, m_loadAxisY, 0.1);
    scaleY(m_distSeries, m_distAxisY, 0);
}
