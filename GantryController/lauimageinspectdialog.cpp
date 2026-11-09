/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauimageinspectdialog.h"
#include "laulastdirectory.h"

#include <cmath>
#include <QDir>
#include <QMenu>
#include <QLabel>
#include <QPainter>
#include <QFileDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QImageWriter>
#include <QStandardPaths>
#include <QContextMenuEvent>

namespace
{
    const double MAX_ZOOM = 20.0;            // screen pixels per image pixel
    const double MIN_ZOOM_OF_FIT = 0.5;      // how far below "whole picture fits" the wheel can go
    const double WHEEL_RATE = 1.0015;        // zoom per unit of wheel (one notch is 120 units)
    const double VELOCITY_SMOOTHING_S = 0.05;// mouse speed is averaged over about this long
    const double RELEASE_STALE_S = 0.08;     // held still this long before letting go: no coasting
    const double FRICTION_PER_S = 3.5;       // coasting speed falls by e over 1/this seconds
    const double MIN_COAST_SPEED = 25.0;     // px per second
    const int COAST_INTERVAL_MS = 16;
}

LAUImageInspectView::LAUImageInspectView(const QImage &image, QWidget *parent) : QWidget(parent), source(image)
{
    picture = QPixmap::fromImage(source);
    setCursor(Qt::OpenHandCursor);
    setFocusPolicy(Qt::WheelFocus);
    setMinimumSize(320, 240);

    coastTimer = new QTimer(this);
    coastTimer->setInterval(COAST_INTERVAL_MS);
    connect(coastTimer, &QTimer::timeout, this, &LAUImageInspectView::onCoast);
}

double LAUImageInspectView::fitScale() const
{
    if (source.isNull() || width() <= 0 || height() <= 0) {
        return 1.0;
    }
    return qMin(width() / (double)source.width(), height() / (double)source.height());
}

void LAUImageInspectView::fit()
{
    scale = fitScale();
    topLeft = QPointF((width() - source.width() * scale) / 2.0, (height() - source.height() * scale) / 2.0);
}

void LAUImageInspectView::keepInView()
{
    // a side smaller than the window stays centred; a larger one can't leave a gap at its edges
    const double w = source.width() * scale, h = source.height() * scale;
    const QPointF before = topLeft;
    topLeft.setX(w <= width() ? (width() - w) / 2.0 : qBound(width() - w, topLeft.x(), 0.0));
    topLeft.setY(h <= height() ? (height() - h) / 2.0 : qBound(height() - h, topLeft.y(), 0.0));

    // coasting into an edge stops that direction
    if (!qFuzzyCompare(1.0 + before.x(), 1.0 + topLeft.x())) {
        velocity.setX(0.0);
    }
    if (!qFuzzyCompare(1.0 + before.y(), 1.0 + topLeft.y())) {
        velocity.setY(0.0);
    }
}

void LAUImageInspectView::stopCoasting()
{
    coastTimer->stop();
    velocity = QPointF();
}

void LAUImageInspectView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.fillRect(rect(), QColor(32, 32, 32));
    // sharp pixels when magnified, smooth when the picture is reduced
    painter.setRenderHint(QPainter::SmoothPixmapTransform, scale < 1.0);
    painter.drawPixmap(QRectF(topLeft, QSizeF(source.width() * scale, source.height() * scale)), picture, QRectF(picture.rect()));
}

void LAUImageInspectView::resizeEvent(QResizeEvent *)
{
    if (fitted) {
        fit();
    } else {
        keepInView();
    }
}

void LAUImageInspectView::wheelEvent(QWheelEvent *event)
{
    stopCoasting();
    const QPointF cursor = event->position();
    const QPointF underCursor = (cursor - topLeft) / scale;      // the image point that stays under the mouse

    const double lowest = fitScale() * MIN_ZOOM_OF_FIT;
    scale = qBound(lowest, scale * std::pow(WHEEL_RATE, event->angleDelta().y()), MAX_ZOOM);
    topLeft = cursor - underCursor * scale;
    fitted = false;
    keepInView();
    update();
    event->accept();
}

void LAUImageInspectView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) {
        return;
    }
    stopCoasting();
    dragging = true;
    lastMouse = event->position();
    moveClock.restart();
    setCursor(Qt::ClosedHandCursor);
}

void LAUImageInspectView::mouseMoveEvent(QMouseEvent *event)
{
    if (!dragging) {
        return;
    }
    const QPointF delta = event->position() - lastMouse;
    const double dt = qMax(moveClock.restart() / 1000.0, 0.001);
    lastMouse = event->position();

    const double blend = qMin(dt / VELOCITY_SMOOTHING_S, 1.0);
    velocity = velocity * (1.0 - blend) + (delta / dt) * blend;

    topLeft += delta;
    fitted = false;
    const QPointF held = velocity;           // reaching an edge mid-drag shouldn't change the throw
    keepInView();
    velocity = held;
    update();
}

void LAUImageInspectView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton || !dragging) {
        return;
    }
    dragging = false;
    setCursor(Qt::OpenHandCursor);

    if (moveClock.elapsed() / 1000.0 > RELEASE_STALE_S || std::hypot(velocity.x(), velocity.y()) < MIN_COAST_SPEED) {
        velocity = QPointF();
        return;
    }
    coastClock.restart();
    coastTimer->start();
}

void LAUImageInspectView::onCoast()
{
    const double dt = qMax(coastClock.restart() / 1000.0, 0.001);
    topLeft += velocity * dt;
    keepInView();                            // zeroes the direction that ran into an edge
    velocity *= std::exp(-FRICTION_PER_S * dt);
    if (std::hypot(velocity.x(), velocity.y()) < MIN_COAST_SPEED) {
        stopCoasting();
    }
    update();
}

void LAUImageInspectView::contextMenuEvent(QContextMenuEvent *event)
{
    stopCoasting();
    QMenu menu(this);
    menu.addAction(QString("Export image to disk..."), this, &LAUImageInspectView::onExport);
    menu.exec(event->globalPos());
}

bool LAUImageInspectView::exportTo(const QString &path) const
{
    return source.save(path);
}

void LAUImageInspectView::onExport()
{
    const QString folder = LAULastDirectory::get();
    const QString path = QFileDialog::getSaveFileName(this, QString("Export Image"), QDir(folder).filePath(QString("platen_mosaic.png")),
                                                      QString("PNG image (*.png);;TIFF image (*.tif *.tiff);;JPEG image (*.jpg *.jpeg)"));
    if (path.isEmpty()) {
        return;
    }
    LAULastDirectory::rememberFile(path);
    if (!exportTo(path)) {
        QMessageBox::warning(this, QString("Export Image"), QString("Couldn't save the image to %1").arg(QDir::toNativeSeparators(path)));
    }
}

LAUImageInspectDialog::LAUImageInspectDialog(const QImage &image, const QString &title, QWidget *parent) : QDialog(parent)
{
    setWindowTitle(title);
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint);
    resize(1200, 1000);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(4);
    layout->addWidget(new LAUImageInspectView(image), 1);

    QLabel *hint = new QLabel(QString("Wheel: zoom.  Left button: drag.  Right button: export."));
    hint->setStyleSheet(QString("color: gray;"));
    layout->addWidget(hint);
}
