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
    void setZoom(const QImage &image);       // a picture above the message, shown from the first one on

public slots:
    void reject() override;                  // Abort, Esc and the title bar's X all stop the rails

signals:
    void emitAbort();

private:
    QLabel *zoom = nullptr;
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
    void onPagePositionRequested();
    void onCornerCheckRequested();
    void onEdgeSlopRequested();
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
        double nearSingle = 0.0, nearPair = 0.0;         // the same, kept within the i1's travel
        double single = 0.0, pair = 0.0;                 // where the rails are, or were when it was confirmed
        bool reachable = false;
        bool confirmed = false;
        QList<cv::Point2d> insets;                       // plain paper in from both edges, to see the hole on, nearest first
        int insetTry = 0;
        bool automatic = false;                          // centred by the camera, not by hand
        QString note;                                    // why the camera couldn't, if it couldn't
        double missMM = -1.0;                            // the camera's last measured miss before the final step
        int moves = 0;
        // the hole relative to the carriage (board mm, rails at zero), seen on the plain paper beside this corner: the
        // platen calibration's error changes across the platen, so a corner is centred with the hole seen next to it
        bool holeKnown = false;
        cv::Point2d hole;
    };
    enum TuneStage { TuneToInset, TuneToCorner, TuneManual };
    TuneStage tuneStage = TuneManual;
    bool tuneWaitingFrame = false;
    QTimer *tuneTimer = nullptr;
    LAUHoleView holeView;
    QList<cv::Point2d> tuneApertures;        // the hole, rails at zero, each time it was seen this session
    cv::Mat tuneWindow;                      // the paper seen through the hole
    LAUHoleView::Corner tuneSeen;            // what the camera last found of this corner's edges, drawn on the zoom
    bool tuneSeenValid = false;
    QString tuneSeenWhy;                     // and why it wasn't the corner, if it wasn't
    void onTuneTimer();
    void onTuneFrame(const cv::Mat &raw16);
    void tuneByHand(const QString &reason);
    void tuneOutOfReach(bool measured, const QString &why);
    bool savePageMap(const cv::Matx23d &fix, double *widthMM, double *heightMM);
    bool pageMoving = false;                 // Go to page position: the sliders are on their way
    QString pageMoveMessage;
    void showTuneZoom(const cv::Mat &raw16);

    // what the camera found at the last stop, for the slop close-up (laugantryedgeslop.cpp)
    cv::Point2d slopAperture;                // the hole relative to the carriage (rails at zero): where to look for it next
    LAUHoleView::Corner slopSeen;
    bool slopSeenValid = false, slopSeenHole = false;
    cv::Point2d slopSeenAperture;            // the hole found, less the rails then: it moves with the carriage
    double slopSeenRadius = 0.0;
    QString slopSeenWhy;
    void showSlopZoom(const cv::Mat &raw16);
    QString slopZoomTitle, slopZoomNote;     // set: the zoom's header and an extra line, for the edge test

    // Measure slop by tracking the hole (laugantryedgeslop.cpp): the camera rides on the carriage and the i1 holder
    // swings on it, so where the hole lands in the camera's view after a move IS the slop.  Over plain paper the hole
    // is a bright disc; the sliders move 1" right and back, 1" left and back, and the hole's place relative to the
    // carriage is compared between the two directions of arrival, then predicted and refined until it lands still.
    enum EdgeStage { EdgeSpot, EdgeOut, EdgeBack };
    struct EdgeStop
    {
        bool predicting = false;
        int side = 1;                        // +1 arrived from the right, moving left; -1 from the left, moving right
        double commandedX = 0.0, reportedX = 0.0, reportedY = 0.0;
        bool found = false;
        cv::Point2d aperture;                // where the hole was relative to the carriage: board mm, rails at zero
        double missX = 0.0;                  // predicting: X, slider mm, still needed to land where the midpoint puts it
        QString note;
    };
    bool edgeSloping = false, edgeCancelling = false, edgeWaitingFrame = false, edgeLearning = true;
    QTimer *edgeTimer = nullptr;
    EdgeStage edgeStage = EdgeSpot;
    int edgeSide = 1, edgeTries = 3;         // edgeTries: predicted returns in a row within tolerance, from each side, to finish
    int edgeHits[2] = { 0, 0 };              // [0] arriving moving left, [1] moving right
    QList<cv::Point2d> edgeApertures[2];     // the hole relative to the carriage, each arrival from that side
    double edgeAway = 25.4, edgeTolerance = 0.02;
    double edgeSpotX = 0.0, edgeY = 0.0, edgeX = 0.0;
    QList<cv::Point2d> edgeSpots;            // plain-paper spots to try, slider mm (X, Y)
    int edgeSpotTry = 0;
    QString edgeFolder, edgeScan;
    QList<EdgeStop> edgeStops;
    bool edgeWasAborted = false;
    int edgeFailures = 0;                    // frames in a row the camera couldn't find the hole in
    // live tracking: the hole found in the live picture a few times a second, for the close-up
    qint64 edgeTrackedAt = 0;
    bool edgeTracked = false;
    cv::Point2d edgeTrackedAperture;
    cv::Point2d edgeReference;               // where the hole was first seen at the spot, relative to the carriage: the bullseye until both sides are known
    bool edgeReferenceSet = false;
    LAUHoleView::Hole edgeEllipse;           // the last hole found, its ellipse's centre relative to the carriage (rails at zero)
    bool edgeEllipseValid = false;
    void keepEdgeEllipse(const LAUHoleView::Hole &hole, double pairMM, double singleMM);
    QList<cv::Point2d> edgeTrail;            // the hole's board position as tracked, latest last
    cv::Point2d edgeMean(int i) const;
    double edgeCorrection(int side) const;   // X, slider mm, to add arriving from this side so the hole lands at the midpoint
    void edgeMoveTo(double x, EdgeStage stage, const QString &message);
    void edgeNextReturn();
    void onEdgeMoveFinished(bool ok, const QString &message);
    void onEdgeTimer();
    void onEdgeFrame(const cv::Mat &raw16);
    void trackHoleLive(const cv::Mat &raw16);
    void finishEdgeSlop();
    void edgeConverged();
    QString writeEdgeResults(QString *plotOut);
    void edgeAborted();
    void endEdgeSlop(const QString &message);

    // One-way moves (laugantryedgeslop.cpp) for the corner fine-tune, Go to page position and the patches: every move
    // ends X alone, moving right, over its last SLOP_APPROACH_MM, so the i1 holder always hangs the same way at a stop
    static constexpr double SLOP_APPROACH_MM = 5.0;
    void resetSlopTracking();
    void moveRailsSlop(double single, double pair, const QString &what = QString());
    bool continueSlopMove(bool ok);          // from onHomeFinished: true, the last leg has been sent
    bool slopLegPending = false;
    struct SlopMove
    {
        QString what;
        double single = 0.0, pair = 0.0;     // the target
        double approachSingle = 0.0;         // where the last, X-only leg starts
        bool arrived = false;
        double reportedSingle = 0.0, reportedPair = 0.0;
    };
    QList<SlopMove> slopMoves;               // this run's moves, for its log
    bool writeSlopMoves(const QString &path) const;
    // the newest slop test's hole relative to the carriage, arriving each way, to compare the fine-tune's sightings with
    cv::Point2d slopHoleLeft, slopHoleRight;
    bool slopHolesKnown = false;
    QString slopTestSource;
    bool loadSlopTest();
    // each time the fine-tune saw the hole on plain paper beside a corner: the hole relative to the carriage
    struct TuneSighting
    {
        QString corner;
        int inset = 0;
        bool found = false;
        double reportedSingle = 0.0, reportedPair = 0.0;
        cv::Point2d hole;                    // board mm, rails at zero
        QString why;
    };
    QList<TuneSighting> tuneSightings;
    bool writeTuneSightings(const QString &path) const;
    LAUPlatenGeometry slopGeometry;          // the page's scan, its travel replaced by the limit sliders'
    QList<TuneCorner> tuneCorners;
    int tuneIndex = -1;
    bool tuning = false;
    bool tuneCancelling = false;
    QString tuneFolder;                      // the page's <scan>/mosaic
    LAUPlatenGeometry tuneGeometry;          // its travel replaced by the limit sliders' while tuning
    double tuneSavedTravel[4] = { 0, 0, 0, 0 };   // the scan's own travel, put back before it is saved
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

    // Check page corners (laugantrycheck.cpp): the sliders visit the four fine-tuned corners round after round, with
    // the one-way approach; at each visit the hole is seen on plain paper beside the corner, then the camera finds the
    // corner and measures how far it is from the hole's centre.  The spread over the rounds is the repeatability.
    struct CheckCorner
    {
        QString name;
        cv::Point2d board;                   // the corner, board mm
        double single = 0.0, pair = 0.0;     // where the fine-tune put it (page_sliders.json)
        bool tunedHoleKnown = false;
        cv::Point2d tunedHole;               // the hole the fine-tune centred it with, rails at zero
        QList<cv::Point2d> insets;           // plain paper beside it, slider mm, nearest first
        int insetUsed = 0;                   // the one the hole was last seen on, tried first
    };
    struct CheckVisit
    {
        int round = 0, corner = 0, inset = 0;
        bool holeFound = false;
        cv::Point2d hole;                    // seen on plain paper on this visit, rails at zero
        QString holeWhy;
        double single = 0.0, pair = 0.0;     // reported, over the corner
        bool found = false;
        cv::Point2d miss, missTuned;         // the corner less the hole's centre: this visit's hole, and the fine-tune's
        double missSingle = 0.0, missPair = 0.0;   // the move that would close the miss, slider mm
        QString why;
    };
    enum CheckStage { CheckToInset, CheckToCorner };
    bool checking = false, checkCancelling = false, checkWaitingFrame = false;
    CheckStage checkStage = CheckToInset;
    QTimer *checkTimer = nullptr;
    QString checkFolder;
    QList<CheckCorner> checkCorners;
    QList<CheckVisit> checkVisits;
    int checkRounds = 3, checkRound = 0, checkIndex = 0, checkInsetTry = 0;
    CheckVisit checkVisit;                   // the visit under way
    cv::Mat checkWindow;
    void checkNextVisit();
    void onCheckMoveFinished(bool ok, const QString &message);
    void onCheckTimer();
    void onCheckFrame(const cv::Mat &raw16);
    void onCheckAbort();
    void showCheckZoom(const cv::Mat &gray, const cv::Point2d &hole, const LAUHoleView::Corner *corner, const QString &text);
    void finishCheck(const QString &stopped);
    void endCheck(const QString &message);

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
