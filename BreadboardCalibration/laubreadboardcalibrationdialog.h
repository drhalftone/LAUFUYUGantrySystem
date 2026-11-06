/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUBreadboardCalibrationDialog -- standalone dialog: the live (levelled) camera picture, one Calibrate
// button, and buttons to look at the results.  Every successful calibration is saved automatically to
// Documents/BreadboardCalibration/<date_time>/ in the same format as vision/calibrate_frame.py.
//
#ifndef LAUBREADBOARDCALIBRATIONDIALOG_H
#define LAUBREADBOARDCALIBRATIONDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QImage>
#include <QThread>
#include <QPushButton>
#include <QButtonGroup>

#include "laubaslergrabber.h"
#include "laubreadboardcalibration.h"

// shows a QImage scaled to fit, keeping its shape
class LAUImageView : public QWidget
{
    Q_OBJECT

public:
    explicit LAUImageView(QWidget *parent = nullptr) : QWidget(parent)
    {
        setMinimumSize(640, 400);
    }
    void setImage(const QImage &img)
    {
        image = img;
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override;

private:
    QImage image;
};

class LAUBreadboardCalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUBreadboardCalibrationDialog(QWidget *parent = nullptr);
    ~LAUBreadboardCalibrationDialog() override;

private slots:
    void onPreview(QImage image, double exposureMs);
    void onAveragedFrame(cv::Mat raw16, double exposureMs);
    void onResult(LAUBreadboardCalibrationResult result);
    void onCalibrateClicked();
    void onOpenFolderClicked();
    void onViewChanged(int id);
    void onCameraError(QString message);

signals:
    void emitCalibrate(cv::Mat raw16, double exposureMs);

private:
    enum View { ViewLive, ViewHoles, ViewCorrected, ViewLines, ViewVanishing };

    LAUImageView *view = nullptr;
    QLabel *status = nullptr;
    QLabel *camera = nullptr;
    QPushButton *calibrateButton = nullptr;
    QPushButton *folderButton = nullptr;
    QButtonGroup *viewButtons = nullptr;

    LAUBaslerGrabber *grabber = nullptr;
    QThread calibrationThread;
    LAUBreadboardCalibration *calibration = nullptr;

    QImage liveImage;
    LAUBreadboardCalibrationResult result;
    QString savedFolder;
    bool busy = false;

    void showView();
    void setStatus(const QString &text);
};

#endif // LAUBREADBOARDCALIBRATIONDIALOG_H
