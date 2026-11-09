/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUImageInspectDialog -- looks at one large image.  The mouse wheel zooms about the cursor, dragging with the
// left button moves the picture and lets it coast on when released, and the right button has Export Image.
//
#ifndef LAUIMAGEINSPECTDIALOG_H
#define LAUIMAGEINSPECTDIALOG_H

#include <QDialog>
#include <QImage>
#include <QPixmap>
#include <QPointF>
#include <QTimer>
#include <QElapsedTimer>
#include <QWidget>

class LAUImageInspectView : public QWidget
{
    Q_OBJECT

public:
    explicit LAUImageInspectView(const QImage &image, QWidget *parent = nullptr);

    double zoom() const
    {
        return scale;
    }
    QPointF offset() const
    {
        return topLeft;
    }
    bool isCoasting() const
    {
        return coastTimer->isActive();
    }
    bool exportTo(const QString &path) const;

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;

private slots:
    void onCoast();
    void onExport();

private:
    QImage source;
    QPixmap picture;
    double scale = 1.0;
    QPointF topLeft;
    bool fitted = true;                  // still showing the whole picture: follow the window's size

    bool dragging = false;
    QPointF lastMouse;
    QPointF velocity;                    // px per second
    QElapsedTimer moveClock;
    QElapsedTimer coastClock;
    QTimer *coastTimer = nullptr;

    double fitScale() const;
    void fit();
    void keepInView();
    void stopCoasting();
};

class LAUImageInspectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUImageInspectDialog(const QImage &image, const QString &title, QWidget *parent = nullptr);
};

#endif // LAUIMAGEINSPECTDIALOG_H
