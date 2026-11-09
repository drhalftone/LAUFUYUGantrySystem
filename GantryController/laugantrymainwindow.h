/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUGantryMainWindow -- one window for the gantry: the carriage camera in a group box on top and the
// rail sliders in a group box below it.  The camera starts by itself; the sliders' group box is checkable
// and the first time it is ticked the user confirms the FMC4030's address and the rails connect.
//
#ifndef LAUGANTRYMAINWINDOW_H
#define LAUGANTRYMAINWINDOW_H

#include <QGroupBox>
#include <QJsonArray>
#include <QMainWindow>
#include <QProgressBar>

#include "laubreadboardcalibrationdialog.h"
#include "lauvelmexwidget.h"

// shown while the rails are moving; the only way out is Abort or the move finishing
class LAUHomeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUHomeDialog(QWidget *parent = nullptr);

    void finish();
    void setMessage(const QString &text);
    void setProgress(int value, int maximum);

public slots:
    void reject() override;                  // Abort, Esc and the title bar's X all stop the rails

signals:
    void emitAbort();

private:
    QLabel *label = nullptr;
    QProgressBar *bar = nullptr;
    QPushButton *abortButton = nullptr;
};

class LAUGantryMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit LAUGantryMainWindow(QWidget *parent = nullptr);

private slots:
    void onRailsEnabled(bool state);
    void onGoHome();
    void onHomeFinished(bool ok, QString message);
    void onScanRequested(bool withTarget);
    void onScanFrame(cv::Mat raw16, double exposureMs);
    void onScanTimer();

private:
    LAUBreadboardCalibrationWidget *cameraWidget = nullptr;
    LAUMultiVelmexWidget *railWidget = nullptr;
    QGroupBox *railBox = nullptr;
    LAUHomeDialog *homeDialog = nullptr;

    // platen scan: stop at each position, let the rails settle, save one averaged frame
    struct ScanStop
    {
        double xMM, yMM;
    };
    QList<ScanStop> scanStops;
    int scanIndex = 0;
    bool scanning = false;
    bool scanWithTarget = false;
    bool scanAborted = false;
    bool waitingForFrame = false;
    QTimer *scanTimer = nullptr;
    QString scanBase;                        // scans go in a time-stamped folder under this
    QString scanFolder;
    QJsonArray scanFrames;

    void moveToNextStop();
    void onScanMoveFinished(bool ok, const QString &message);
    bool saveScanFrame(const cv::Mat &raw16, double exposureMs);
    void finishScan(const QString &message);
    void onAbort();
};

#endif // LAUGANTRYMAINWINDOW_H
