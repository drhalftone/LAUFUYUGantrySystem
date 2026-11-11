/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUGantryMainWindow -- one window for the gantry: the carriage camera in a group box on top and the
// rail sliders in a group box below it.  The camera starts by itself; the sliders' group box is checkable
// and the first time it is ticked the user confirms the FMC4030's address and the rails connect.
//
// Scan target measures a printed target in two passes: the camera scans the platen with the target on it and the
// merged picture gives every patch's place (LAUTargetFinder, LAUPlatenGeometry); then, the i1 Pro calibrated on
// its white tile and seen in its cradle by the camera, the rails stop on each patch for an i1 reading, and the
// spectra go into a .ti3 with XYZ added by Argyll's spec2cie.
//
#ifndef LAUGANTRYMAINWINDOW_H
#define LAUGANTRYMAINWINDOW_H

#include <QGroupBox>
#include <QJsonArray>
#include <QMainWindow>
#include <QProgressBar>
#include <QThread>

#include "lauplatenmosaic.h"
#include "lauplatenoverlay.h"
#include "lauprinttarget.h"
#include "laui1worker.h"
#include "laurailpredictor.h"
#include "lauholeview.h"
#include <QElapsedTimer>
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
    ~LAUGantryMainWindow() override;

signals:
    void emitMerge(LAUPlatenMosaicRequest request);
    void emitI1Start();
    void emitI1Measure(int index);

private slots:
    void onRailCount(int count, int dim);
    void onRailMove(int target, double speed, double accel, int dim);
    void onPrintTargetRequested();
    void onGuideTimer();
    void onMergeProgress(QString text);
    void onMergeFinished(LAUPlatenMosaicResult result);
    void onRailsEnabled(bool state);
    void onGoHome();
    void onHomeFinished(bool ok, QString message);
    void onScanRequested(bool withTarget);
    void onImageScanRequested();
    void onCornerTuneRequested();
    void onScanFrame(cv::Mat raw16, double exposureMs);
    void onScanTimer();
    void onI1Ready(bool ok, QString message, int serial);
    void onI1Reading(int index, QVector<double> reflectance, QString error);
    void onMeasureTimer();
    void onSpec2cieFinished(QString tool, bool success);

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
    // Bare calibrates the platen; Image and Target merge against that calibration, and Target then measures patches
    enum ScanKind { ScanBare, ScanImage, ScanTarget };
    void startScan(ScanKind kind);
    bool scanWithTarget = false;             // something on the platen: merged against the bare calibration
    bool scanMeasures = false;               // and find the target's patches for the i1 after
    bool scanAborted = false;
    bool waitingForFrame = false;
    QTimer *scanTimer = nullptr;
    QString scanBase;                        // scans go in a time-stamped folder under this
    QString scanFolder;
    QJsonArray scanFrames;

    // merging runs in its own thread so the live picture keeps moving
    QThread mergeThread;
    LAUPlatenMosaic *merger = nullptr;

    // after a completed scan the rails go home while it merges; the window unlocks once both are done
    bool returningHome = false;
    bool merging = false;
    QString wrapUpMessage, wrapUpNote, wrapUpImage, wrapUpFolder;

    // measuring a target with the i1, after the camera has found its patches
    struct PatchStop
    {
        int patch;                           // into measureTarget.patches
        double singleMM, pairMM;
    };
    LAUPrintTarget measureTarget;
    QList<PatchStop> patchStops;
    QVector<QVector<double>> readings;      // by patch; empty until measured
    int patchIndex = 0;
    bool measuring = false;
    bool measureAborting = false;
    bool waitingForReading = false;
    int i1Serial = 0;
    QString measureFolder, measureName;
    QTimer *measureTimer = nullptr;
    QThread i1Thread;
    LAUi1Worker *i1 = nullptr;
    class LAUArgyllRunner *argyll = nullptr;

    cv::Mat lastLiveFrame;
    bool cradleIsClear(const QString &title);    // false if the user cancels; warns while the i1 looks attached

    void setUpMeasuring();                   // these in laugantrymeasure.cpp
    void locateTarget(const QString &mosaicFolder);
    void startI1();
    void moveToNextPatch();
    void onPatchMoveFinished(bool ok, const QString &message);
    bool writeSpectral(const QString &path) const;
    void endMeasure(const QString &message);
    void onMeasureAbort();

    // Fine-tune page corners (laugantrytune.cpp): the i1 hole over each corner of the last page found, the user
    // nudging the rails until the corner is centred, then a correction of the rails fitted to the four
    struct TuneCorner
    {
        QString name;
        cv::Point2d board;
        double plannedSingle = 0.0, plannedPair = 0.0;   // from the model, before any correction
        double startSingle = 0.0, startPair = 0.0;       // where the rails first went: the model with the saved correction
        double single = 0.0, pair = 0.0;                 // where the rails are, or were when it was confirmed
        bool reachable = false;
        bool confirmed = false;
        cv::Point2d inset;                               // plain paper 12 mm in from both edges, to see the hole on
        double insetSingle = 0.0, insetPair = 0.0;
        bool insetReachable = false;
        bool automatic = false;                          // centred by the camera, not by hand
        QString note;                                    // why the camera couldn't, if it couldn't
        double missMM = -1.0;                            // the camera's last measured miss before the final step
        int moves = 0;
    };
    enum TuneStage { TuneToInset, TuneToCorner, TuneManual };
    TuneStage tuneStage = TuneManual;
    bool tuneWaitingFrame = false;
    QTimer *tuneTimer = nullptr;
    LAUHoleView holeView;
    QList<cv::Point2d> tuneApertures;        // the hole, rails at zero, each time it was seen this session
    cv::Mat tuneWindow;                      // the paper seen through the hole
    void onTuneTimer();
    void onTuneFrame(const cv::Mat &raw16);
    void tuneByHand(const QString &reason);
    QList<TuneCorner> tuneCorners;
    int tuneIndex = -1;
    bool tuning = false;
    bool tuneCancelling = false;
    QString tuneFolder;                      // the page's <scan>/mosaic
    LAUPlatenGeometry tuneGeometry;
    class LAUCornerTuneDialog *tuneDialog = nullptr;
    void goToTuneCorner();
    void onTuneMoveFinished(bool ok, const QString &message);
    void onTuneNudge(int right, int down);
    void onTuneCentered();
    void onTuneSkip();
    void onTuneCancel();
    void nextTuneCorner();
    void finishTune();
    void endTune(const QString &message);

    void returnHomeAfterScan();
    void wrapUp();
    void startMerge(const QString &bareFolder);

    // what should be on the platen at the rails' current position
    LAUPlatenOverlay platenOverlay;
    LAURailPredictor singleRail, pairRail;   // reports come twice a second; these fill in between, for drawing only
    QElapsedTimer guideClock;
    QTimer *guideTimer = nullptr;
    void loadGuides();
    void updateGuides();
    void moveToNextStop();
    void onScanMoveFinished(bool ok, const QString &message);
    bool saveScanFrame(const cv::Mat &raw16, double exposureMs);
    void finishScan(const QString &message);
    void onAbort();
};

#endif // LAUGANTRYMAINWINDOW_H
