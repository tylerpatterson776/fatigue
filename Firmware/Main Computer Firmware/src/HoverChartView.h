#pragma once
#include <QLabel>
#include <QtCharts/QChartView>
#include <QtCharts/QXYSeries>

class HoverChartView : public QChartView {
    Q_OBJECT
public:
    explicit HoverChartView(QChart* chart, QWidget* parent = nullptr);
    bool isPaused() const { return m_paused; }
    void setSeries(QXYSeries* series) { m_series = series; }
    void updateStats(float freq_hz, float avg_max, float avg_min);

protected:
    void mouseMoveEvent(QMouseEvent* event) override;
    void enterEvent(QEnterEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    bool       m_paused  = false;
    QXYSeries* m_series  = nullptr;
    QLabel*    m_tooltip = nullptr;
    QLabel*    m_stats   = nullptr;
};
