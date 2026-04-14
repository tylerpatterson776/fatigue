#include "HoverChartView.h"
#include <QMouseEvent>
#include <QtCharts/QChart>
#include <limits>

HoverChartView::HoverChartView(QChart* chart, QWidget* parent)
    : QChartView(chart, parent)
{
    setMouseTracking(true);

    m_tooltip = new QLabel(this);
    m_tooltip->setStyleSheet(
        "background: rgba(0,0,0,160); color: white; "
        "padding: 4px 6px; border-radius: 3px; font-family: monospace;");
    m_tooltip->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_tooltip->hide();

    m_stats = new QLabel(this);
    m_stats->setStyleSheet(
        "color: white; background: rgba(0,0,0,120); "
        "padding: 4px 6px; border-radius: 3px; font-family: monospace;");
    m_stats->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_stats->setText("f: -- Hz\nmax: --\nmin: --");
    m_stats->adjustSize();
    m_stats->move(10, 36); // below chart title
    m_stats->raise();
    m_stats->show();
}

void HoverChartView::enterEvent(QEnterEvent* event)
{
    m_paused = true;
    m_tooltip->show();
    QChartView::enterEvent(event);
}

void HoverChartView::leaveEvent(QEvent* event)
{
    m_paused = false;
    m_tooltip->hide();
    QChartView::leaveEvent(event);
}

void HoverChartView::mouseMoveEvent(QMouseEvent* event)
{
    const QPointF cursorVal = chart()->mapToValue(event->pos());
    const double cursorX = cursorVal.x();

    // Find the series point with the closest x to the cursor
    double closestY = cursorVal.y();
    if (m_series && m_series->count() > 0) {
        double bestDx = std::numeric_limits<double>::max();
        for (int i = 0; i < m_series->count(); ++i) {
            const QPointF p = m_series->at(i);
            const double dx = std::abs(p.x() - cursorX);
            if (dx < bestDx) {
                bestDx = dx;
                closestY = p.y();
            }
        }
    }

    m_tooltip->setText(
        QString("t: %1 s\ny: %2")
            .arg(cursorX, 0, 'f', 3)
            .arg(closestY, 0, 'f', 4));
    m_tooltip->adjustSize();

    int tx = event->pos().x() + 14;
    int ty = event->pos().y() - m_tooltip->height() - 6;
    if (tx + m_tooltip->width() > width())
        tx = event->pos().x() - m_tooltip->width() - 14;
    if (ty < 0)
        ty = event->pos().y() + 14;
    m_tooltip->move(tx, ty);
    m_tooltip->raise();

    QChartView::mouseMoveEvent(event);
}

void HoverChartView::updateStats(float freq_hz, float avg_max, float avg_min)
{
    m_stats->setText(
        QString("f: %1 Hz\nmax: %2\nmin: %3")
            .arg(static_cast<double>(freq_hz), 0, 'f', 2)
            .arg(static_cast<double>(avg_max), 0, 'f', 3)
            .arg(static_cast<double>(avg_min), 0, 'f', 3));
    m_stats->adjustSize();
}
