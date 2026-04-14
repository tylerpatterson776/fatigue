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


static void updateHLine(QLineSeries* line, double y)
{
    line->replace(0, -1e9, y);
    line->replace(1,  1e9, y);
}

// ReSharper disable once CppMemberFunctionMayBeStatic
HoverChartView* MainWindow::makeChart(const QString& title, const QString& yLabel,
                                      QLineSeries*& series, QValueAxis*& axisX, QValueAxis*& axisY,
                                      QLineSeries*& minLine, QLineSeries*& maxLine)
{
    auto* chart = new QChart;
    chart->setTitle(title);
    chart->legend()->hide();
    chart->setAnimationOptions(QChart::NoAnimation);

    axisX = new QValueAxis;
    axisX->setTitleText("Time (s)");
    axisX->setRange(0.0, kWindow);
    axisX->setLabelFormat("%.0f");
    chart->addAxis(axisX, Qt::AlignBottom);

    axisY = new QValueAxis;
    axisY->setTitleText(yLabel);
    axisY->setRange(-1.0, 1.0);
    axisY->setLabelFormat("%.2f");
    chart->addAxis(axisY, Qt::AlignLeft);

    // Min/max lines (added first so data series renders on top)
    QPen redPen(QColor(220, 30, 30, 110));
    redPen.setWidth(1);
    redPen.setStyle(Qt::DashLine);

    minLine = new QLineSeries;
    minLine->setPen(redPen);
    minLine->append(-1e9, 0);
    minLine->append( 1e9, 0);
    chart->addSeries(minLine);
    minLine->attachAxis(axisX);
    minLine->attachAxis(axisY);

    maxLine = new QLineSeries;
    maxLine->setPen(redPen);
    maxLine->append(-1e9, 0);
    maxLine->append( 1e9, 0);
    chart->addSeries(maxLine);
    maxLine->attachAxis(axisX);
    maxLine->attachAxis(axisY);

    // Data series
    series = new QLineSeries;
    chart->addSeries(series);
    series->attachAxis(axisX);
    series->attachAxis(axisY);

    auto* view = new HoverChartView(chart);
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
    m_logBtn->setFixedWidth(100);
    connect(m_logBtn, &QPushButton::clicked, this, &MainWindow::onLogToggled);
    bar->addWidget(m_logBtn);
    bar->addStretch();
    root->addLayout(bar);

    // --- Charts ---
    auto* splitter = new QSplitter(Qt::Vertical);

    m_loadView = makeChart("Load", "Load (N)",
                           m_loadSeries, m_loadAxisX, m_loadAxisY,
                           m_loadMinLine, m_loadMaxLine);
    m_loadView->setSeries(m_loadSeries);

    m_distView = makeChart("Distance", "Distance (mm)",
                           m_distSeries, m_distAxisX, m_distAxisY,
                           m_distMinLine, m_distMaxLine);
    m_distView->setSeries(m_distSeries);

    splitter->addWidget(m_loadView);
    splitter->addWidget(m_distView);
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
        m_tStart   = -1.0;
        m_lastLoad = std::numeric_limits<double>::quiet_NaN();
        m_lastDist = std::numeric_limits<double>::quiet_NaN();
        m_loadEstimator = FrequencyEstimator{};
        m_distEstimator = FrequencyEstimator{};
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
        m_loadEstimator = FrequencyEstimator{};
        m_distEstimator = FrequencyEstimator{};
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
    static uint32_t sample_count = 0;
    ++sample_count;

    constexpr uint32_t sample_division_factor = 3;
    if (sample_count % sample_division_factor != 0)
        return;

    if (m_logging)
        for (const auto& sink : m_sinks)
            sink->handle(s);

    const double t = s.timestamp_ms / 1000.0;
    if (m_t0 < 0.0)     m_t0     = t;
    if (m_tStart < 0.0) m_tStart = t;

    const auto t_rel = static_cast<float>(t - m_tStart);
    const auto loadResult = m_loadEstimator.update(t_rel, static_cast<float>(s.load_n));
    const auto distResult = m_distEstimator.update(t_rel, static_cast<float>(s.distance_mm));

    m_loadView->updateStats(loadResult.frequency_hz, loadResult.avg_max, loadResult.avg_min);
    m_distView->updateStats(distResult.frequency_hz, distResult.avg_max, distResult.avg_min);
    updateHLine(m_loadMinLine, loadResult.avg_min);
    updateHLine(m_loadMaxLine, loadResult.avg_max);
    updateHLine(m_distMinLine, distResult.avg_min);
    updateHLine(m_distMaxLine, distResult.avg_max);

    // Freeze chart data while mouse is over either chart.
    if (m_loadView->isPaused() || m_distView->isPaused()) return;

    const double x = t - m_t0;
    if (x >= kWindow) {
        m_t0 = t;
        m_loadSeries->clear();
        m_distSeries->clear();
        m_loadSeries->append(0.0, s.load_n);
        m_distSeries->append(0.0, s.distance_mm);
        return;
    }

    constexpr double kLoadThresh = 0.1;    // N
    constexpr double kDistThresh = 0.00001; // mm

    const bool loadChanged = std::isnan(m_lastLoad) || std::abs(s.load_n      - m_lastLoad) > kLoadThresh;
    const bool distChanged = std::isnan(m_lastDist) || std::abs(s.distance_mm - m_lastDist) > kDistThresh;
    if (!loadChanged && !distChanged) return;

    if (loadChanged) { m_loadSeries->append(x, s.load_n);      m_lastLoad = s.load_n; }
    if (distChanged) { m_distSeries->append(x, s.distance_mm); m_lastDist = s.distance_mm; }

    auto scaleY = [](const QLineSeries* ser, QValueAxis* axis, const double marginFactor) {
        if (ser->count() == 0) return;
        double lo = std::numeric_limits<double>::max(), hi = -std::numeric_limits<double>::max();
        for (int i = 0; i < ser->count(); ++i) {
            const QPointF p = ser->at(i);
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
        }
        const double margin = std::max((hi - lo) * marginFactor, 0.5);
        axis->setRange(lo - margin, hi + margin);
    };
    scaleY(m_loadSeries, m_loadAxisY, 0.1);
    scaleY(m_distSeries, m_distAxisY, 0);
}
