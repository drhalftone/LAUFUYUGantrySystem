/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUBreadboardCalibrationWidget -- the live (levelled) camera picture; Calibrate and the choice of which
// result to look at are on its right-click menu.  Every successful calibration is saved automatically to
// Documents/BreadboardCalibration/<date_time>/ in the same format as vision/calibrate_frame.py.
// LAUBreadboardCalibrationDialog -- the same widget as a standalone dialog with a Close button.
//
#ifndef LAUBREADBOARDCALIBRATIONDIALOG_H
#define LAUBREADBOARDCALIBRATIONDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QImage>
#include <QAction>
#include <QThread>
#include <QActionGroup>
#include <QPushButton>
#include <QVector>
#include <QList>

#include "lauglimageview.h"
#include "laubaslergrabber.h"
#include "laubreadboardcalibration.h"

// Recalibrates on every new frame so the user can watch the parameters settle, then lock one in with OK.
// The widget below feeds it; which picture to show is on its right-click menu.
class LAULiveCalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAULiveCalibrationDialog(QWidget *parent = nullptr);

    // the most recent valid calibration, even if frames since then have failed
    LAUBreadboardCalibrationResult lockedResult() const
    {
        return lastGood;
    }

public slots:
    void onFrame(cv::Mat raw16);
    void onResult(LAUBreadboardCalibrationResult result);

private:
    enum View { ViewLive, ViewHoles, ViewCorrected, ViewLines, ViewVanishing };
    enum { ParameterCount = 5, HistoryLength = 10, SteadyFrames = 5 };

    LAUImageView *view = nullptr;
    QLabel *verdict = nullptr;
    QLabel *nowLabels[ParameterCount] = { nullptr };
    QLabel *changeLabels[ParameterCount] = { nullptr };
    QPushButton *okButton = nullptr;
    QActionGroup *viewActions = nullptr;

    cv::Mat liveFrame;
    LAUBreadboardCalibrationResult lastGood;    // what OK returns and the result views keep showing
    QList<QVector<double>> history;              // the last HistoryLength good frames, in a row

    void showView();
    void setVerdict(const QString &text, const QString &color);
};

class LAUBreadboardCalibrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LAUBreadboardCalibrationWidget(QWidget *parent = nullptr);
    ~LAUBreadboardCalibrationWidget() override;

    // With rails attached, OK on the Calibrate warning asks for them to go home (emitGoHome) and the
    // capture waits for onHomeFinished().  Without, the user has to put them there by hand.
    void enableHomeMove(bool state = true)
    {
        homeMoveFlag = state;
    }

public slots:
    void onHomeFinished(bool ok, QString message);

private slots:
    void onFrame(cv::Mat raw16, double exposureMs);
    void onAveragedFrame(cv::Mat raw16, double exposureMs);
    void onResult(LAUBreadboardCalibrationResult result);
    void onCalibrateClicked();
    void onOpenFolderClicked();
    void onCameraError(QString message);
    void onLiveFinished(int code);

signals:
    void emitCalibrate(cv::Mat raw16, double exposureMs);
    void emitGoHome();

private:
    enum View { ViewLive, ViewLiveCorrected, ViewHoles, ViewCorrected, ViewLines, ViewVanishing };

    LAUImageView *view = nullptr;
    QLabel *status = nullptr;
    QLabel *camera = nullptr;
    QAction *calibrateAction = nullptr;
    QAction *folderAction = nullptr;
    QActionGroup *viewActions = nullptr;

    LAUBaslerGrabber *grabber = nullptr;
    QThread calibrationThread;
    LAUBreadboardCalibration *calibration = nullptr;

    cv::Mat liveFrame;
    LAUBreadboardCalibrationResult result;
    QString savedFolder;
    bool busy = false;
    bool homeMoveFlag = false;
    LAULiveCalibrationDialog *liveDialog = nullptr;
    bool liveInFlight = false;                   // a frame requested for the live dialog hasn't come back yet

    void startCapture();
    void startLive();
    void requestLiveFrame();
    void applyResult(const LAUBreadboardCalibrationResult &r);    void showView();
    void setStatus(const QString &text);
};

class LAUBreadboardCalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUBreadboardCalibrationDialog(QWidget *parent = nullptr);
};

#endif // LAUBREADBOARDCALIBRATIONDIALOG_H
