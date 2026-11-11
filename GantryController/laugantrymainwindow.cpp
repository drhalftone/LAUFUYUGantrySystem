/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laugantrymainwindow.h"

#include <QDir>
#include <QFile>
#include <QTimer>
#include <QSettings>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QStandardPaths>

#include "lauimageinspectdialog.h"
#include "lauprinttargetdialog.h"
#include "laulastdirectory.h"
#include <QFileInfo>
#include <QFileDialog>
#include <QVBoxLayout>
#include <cmath>

namespace
{
    // The number of stops comes from the calibrated view, so a different lens needs no change here: each step
    // is at most this fraction of the view, which leaves room for the i1 cradle hiding part of every frame.
    const double SCAN_MAX_STEP_OF_VIEW = 0.40;
    const int SCAN_SETTLE_MS = 500;
    const int GUIDE_REDRAW_MS = 25;          // how often the guides move while a rail is moving
    const int SCAN_FRAME_TIMEOUT_MS = 15000;
    const double HALF_PITCH_MM = 12.7;

    // Evenly spaced stops.  A step that is a whole number of half hole-pitches would hide a wrong
    // direction from the Python calibration (see vision/scan_map.py), so nudge the step off them.
    int stopsForTravel(double travelMM, double viewMM)
    {
        return qMax(2, (int)std::ceil(travelMM / (SCAN_MAX_STEP_OF_VIEW * viewMM)) + 1);
    }

    QList<double> axisStops(double low, double high, int count)
    {
        double step = (high - low) / (count - 1);
        for (double r = std::fmod(step, HALF_PITCH_MM); r < 1.0 || r > HALF_PITCH_MM - 1.0; r = std::fmod(step, HALF_PITCH_MM)) {
            step -= 0.5;
        }
        QList<double> stops;
        for (int n = 0; n < count; n++) {
            stops << low + n * step;
        }
        return stops;
    }
}

LAUHomeDialog::LAUHomeDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Calibrate"));
    setModal(true);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setSpacing(12);
    label = new QLabel(QString("Moving the sliders to their home position..."));
    label->setAlignment(Qt::AlignHCenter);
    layout->addWidget(label);

    // a range of 0..0 makes the bar sweep left to right over and over until the dialog closes
    bar = new QProgressBar();
    bar->setRange(0, 0);
    bar->setTextVisible(false);
    bar->setMinimumWidth(360);
    layout->addWidget(bar);

    abortButton = new QPushButton(QString("Abort"));
    abortButton->setMinimumHeight(44);
    connect(abortButton, &QPushButton::clicked, this, &LAUHomeDialog::reject);
    layout->addWidget(abortButton);
}

void LAUHomeDialog::reject()
{
    abortButton->setEnabled(false);
    label->setText(QString("Stopping the sliders..."));
    emit emitAbort();
}

void LAUHomeDialog::setMessage(const QString &text)
{
    if (abortButton->isEnabled()) {
        label->setText(text);
    }
}

void LAUHomeDialog::setProgress(int value, int maximum)
{
    bar->setRange(0, maximum);
    bar->setValue(value);
}

void LAUHomeDialog::finish()
{
    QDialog::accept();
    deleteLater();
}

LAUGantryMainWindow::LAUGantryMainWindow(QWidget *parent) : QMainWindow(parent)
{
#ifdef FUYU_SIMULATE
    setWindowTitle(QString("Gantry Controller  —  SIMULATION (no rail hardware)"));
#else
    setWindowTitle(QString("Gantry Controller"));
#endif

    QWidget *central = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(central);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);
    setCentralWidget(central);

    QGroupBox *cameraBox = new QGroupBox(QString("Camera"));
    QVBoxLayout *cameraLayout = new QVBoxLayout(cameraBox);
    cameraWidget = new LAUBreadboardCalibrationWidget();
    cameraLayout->addWidget(cameraWidget);
    layout->addWidget(cameraBox, 1);          // the camera takes all the spare height

    //   FMC4030 SDK axis 0 = X, the single rail of the H
    //   FMC4030 SDK axis 2 = Y, the tied-together rail pair (both rails share one controller axis)
    QList<int> axes;
    axes << 0 << 2;

    railBox = new QGroupBox(QString("Enable Sliders"));
    railBox->setCheckable(true);
    railBox->setChecked(false);
    QVBoxLayout *railLayout = new QVBoxLayout(railBox);
    railWidget = new LAUMultiVelmexWidget(axes, nullptr, false);
    railLayout->addWidget(railWidget);
    connect(railBox, &QGroupBox::toggled, this, &LAUGantryMainWindow::onRailsEnabled);
    layout->addWidget(railBox, 0);

    cameraWidget->enableHomeMove();
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitGoHome, this, &LAUGantryMainWindow::onGoHome);
    connect(railWidget, &LAUMultiVelmexWidget::emitHomeFinished, this, &LAUGantryMainWindow::onHomeFinished);

    cameraWidget->enablePlatenScan();
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitScanRequested, this, &LAUGantryMainWindow::onScanRequested);
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitImageScanRequested, this, &LAUGantryMainWindow::onImageScanRequested);
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitScanFrame, this, &LAUGantryMainWindow::onScanFrame);
    scanBase = QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).filePath(QString("GantryScans"));
    scanTimer = new QTimer(this);
    scanTimer->setSingleShot(true);
    connect(scanTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onScanTimer);

    qRegisterMetaType<LAUPlatenMosaicRequest>("LAUPlatenMosaicRequest");
    qRegisterMetaType<LAUPlatenMosaicResult>("LAUPlatenMosaicResult");
    merger = new LAUPlatenMosaic();
    merger->moveToThread(&mergeThread);
    connect(&mergeThread, &QThread::finished, merger, &QObject::deleteLater);
    connect(this, &LAUGantryMainWindow::emitMerge, merger, &LAUPlatenMosaic::onMerge, Qt::QueuedConnection);
    connect(merger, &LAUPlatenMosaic::emitProgress, this, &LAUGantryMainWindow::onMergeProgress, Qt::QueuedConnection);
    connect(merger, &LAUPlatenMosaic::emitFinished, this, &LAUGantryMainWindow::onMergeFinished, Qt::QueuedConnection);
    mergeThread.start();

    setUpMeasuring();                        // laugantrymeasure.cpp

    connect(railWidget, &LAUMultiVelmexWidget::emitRailCount, this, &LAUGantryMainWindow::onRailCount);
    connect(railWidget, &LAUMultiVelmexWidget::emitRailMove, this, &LAUGantryMainWindow::onRailMove);
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitPrintTargetRequested, this, &LAUGantryMainWindow::onPrintTargetRequested);
    guideClock.start();
    guideTimer = new QTimer(this);
    guideTimer->setInterval(GUIDE_REDRAW_MS);
    connect(guideTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onGuideTimer);
    loadGuides();
}

void LAUGantryMainWindow::loadGuides()
{
    platenOverlay.load(QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString());
    updateGuides();
}

void LAUGantryMainWindow::updateGuides()
{
    double xLow, xHigh, yLow, yHigh;
    if (!platenOverlay.isValid() || !singleRail.isKnown() || !pairRail.isKnown() ||
        !railWidget->scanRange(0, &xLow, &xHigh) || !railWidget->scanRange(2, &yLow, &yHigh)) {
        cameraWidget->setGuides(LAUViewOverlay());      // without the rails' position there is nothing true to draw
        return;
    }
    const double now = guideClock.elapsed() / 1000.0;
    cameraWidget->setGuides(platenOverlay.build(pairRail.estimate(now), singleRail.estimate(now), xLow, xHigh, yLow, yHigh));
}

void LAUGantryMainWindow::onRailCount(int count, int dim)
{
    if (dim != 0 && dim != 2) {
        return;
    }
    LAURailPredictor &rail = (dim == 0) ? singleRail : pairRail;
    const double now = guideClock.elapsed() / 1000.0, mm = count / 1000.0;
    const bool same = rail.isKnown() && !rail.isMoving() && rail.estimate(now) == mm;
    rail.report(mm, now);
    if (!same) {
        updateGuides();
    }
}

void LAUGantryMainWindow::onRailMove(int target, double speed, double accel, int dim)
{
    if (dim != 0 && dim != 2) {
        return;
    }
    ((dim == 0) ? singleRail : pairRail).command(target / 1000.0, speed, accel, guideClock.elapsed() / 1000.0);
    if (singleRail.isMoving() || pairRail.isMoving()) {
        guideTimer->start();
    }
}

void LAUGantryMainWindow::onPrintTargetRequested()
{
    double xLow, xHigh, yLow, yHigh, widthMM, heightMM;
    if (!railWidget->scanRange(0, &xLow, &xHigh) || !railWidget->scanRange(2, &yLow, &yHigh)) {
        cameraWidget->showMessage(QString("The sliders' travel isn't known. Press Calibrate on both sliders first."));
        return;
    }
    if (!platenOverlay.reachSize(xLow, xHigh, yLow, yHigh, &widthMM, &heightMM)) {
        cameraWidget->showMessage(QString("The i1's reach on the platen isn't known yet. Run Calibrate platen first."));
        return;
    }
    const QString documents = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    LAUPrintTargetDialog dialog(widthMM, heightMM, QDir(documents).filePath(QString("GantryTargets")), this);
    dialog.exec();
}

void LAUGantryMainWindow::onGuideTimer()
{
    if (!singleRail.isMoving() && !pairRail.isMoving()) {
        guideTimer->stop();
    }
    updateGuides();
}

LAUGantryMainWindow::~LAUGantryMainWindow()
{
    merger->cancel();
    mergeThread.quit();
    mergeThread.wait();
    i1Thread.quit();                         // waits out a reading in progress, a few seconds at most
    i1Thread.wait();
}

void LAUGantryMainWindow::startMerge(const QString &bareFolder)
{
    LAUPlatenMosaicRequest request;
    request.scanFolder = scanFolder;
    request.bareFolder = bareFolder;
    if (!railWidget->scanRange(0, &request.sliderXLow, &request.sliderXHigh) ||
        !railWidget->scanRange(2, &request.sliderYLow, &request.sliderYHigh)) {
        wrapUpMessage = QString("The scan is saved in %1, but the sliders' travel isn't known, so it wasn't merged.").arg(QDir::toNativeSeparators(scanFolder));
        wrapUp();
        return;
    }

    merging = true;
    cameraWidget->showMessage(QString("Merging the scan..."));
    emit emitMerge(request);
}

void LAUGantryMainWindow::returnHomeAfterScan()
{
    scanning = false;
    waitingForFrame = false;
    scanTimer->stop();
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    wrapUpMessage.clear();
    wrapUpNote.clear();
    wrapUpImage.clear();

    returningHome = true;
    railWidget->goToHome();                  // can report back before it returns, if the move can't start
    if (returningHome && railWidget->isHoming()) {
        homeDialog = new LAUHomeDialog(this);
        homeDialog->setWindowTitle(QString("Scan Complete"));
        homeDialog->setMessage(QString("Scan complete. Returning the sliders to their home position..."));
        connect(homeDialog, &LAUHomeDialog::emitAbort, this, &LAUGantryMainWindow::onAbort);
        homeDialog->open();
    } else if (returningHome) {
        returningHome = false;
        wrapUpNote = QString("\nThe sliders did not return to their home position.");
    }
}

void LAUGantryMainWindow::wrapUp()
{
    if (merging || returningHome) {
        return;
    }
    const QString image = wrapUpImage, folder = wrapUpFolder;
    finishScan(wrapUpMessage + wrapUpNote);
    loadGuides();                            // a new platen calibration may have just been saved
    if (scanMeasures && measureTarget.isValid() && !folder.isEmpty()) {
        locateTarget(folder);                // the camera's pass is done: on to the patches
        return;
    }
    if (!image.isEmpty()) {
        const QImage merged(image);
        if (merged.isNull()) {
            cameraWidget->showMessage(QString("Couldn't open the merged picture %1").arg(QDir::toNativeSeparators(image)));
            return;
        }
        LAUImageInspectDialog *inspect = new LAUImageInspectDialog(merged, QString("Merged Platen"), this);
        inspect->setAttribute(Qt::WA_DeleteOnClose);
        inspect->show();
    }
}

void LAUGantryMainWindow::onMergeProgress(QString text)
{
    cameraWidget->showMessage(text);
}

void LAUGantryMainWindow::onMergeFinished(LAUPlatenMosaicResult result)
{
    merging = false;
    wrapUpMessage = result.ok ? QString("Merged. Results are in %1").arg(QDir::toNativeSeparators(result.folder)) : result.message;
    // a picture-only scan shows the plain picture; the others show it labelled with the holes
    const bool plain = scanWithTarget && !scanMeasures;
    const QString plainImage = result.pageImage.isEmpty() ? QDir(result.folder).filePath(QString("platen_mosaic.png")) : result.pageImage;
    wrapUpImage = result.ok ? (plain ? plainImage : result.gridImage) : QString();
    wrapUpFolder = result.ok ? result.folder : QString();
    wrapUp();
}

void LAUGantryMainWindow::onAbort()
{
    if (scanning) {
        scanAborted = true;
        if (!railWidget->isHoming()) {       // between moves: nothing to stop
            finishScan(QString("Scan aborted. The frames saved so far are in %1").arg(QDir::toNativeSeparators(scanFolder)));
            return;
        }
    }
    railWidget->abortHome();
}

void LAUGantryMainWindow::onScanRequested(bool withTarget)
{
    startScan(withTarget ? ScanTarget : ScanBare);
}

void LAUGantryMainWindow::onImageScanRequested()
{
    startScan(ScanImage);
}

void LAUGantryMainWindow::startScan(ScanKind kind)
{
    const bool withTarget = (kind != ScanBare);
    if (withTarget && QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString().isEmpty()) {
        cameraWidget->showMessage(QString("The platen hasn't been calibrated yet, so a scan of it can't be merged. Run Calibrate platen first."));
        return;
    }
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        cameraWidget->showMessage(QString("Tick Enable Sliders first, so the sliders can scan the platen."));
        return;
    }

    double xLow, xHigh, yLow, yHigh;
    if (!railWidget->scanRange(0, &xLow, &xHigh) || !railWidget->scanRange(2, &yLow, &yHigh)) {
        cameraWidget->showMessage(QString("The sliders' travel isn't known. Press Calibrate on both sliders first."));
        return;
    }

    // the camera's mount fixes which way it faces: across the picture is the single rail (X), up it is the pair (Y)
    double viewAcross, viewAlong;
    if (!LAUBreadboardCalibration::boardFootprint(cameraWidget->calibrationResult(), &viewAcross, &viewAlong)) {
        cameraWidget->showMessage(QString("The camera's view of the platen isn't known. Calibrate the camera first."));
        return;
    }
    const int stopsX = stopsForTravel(xHigh - xLow, viewAcross);
    const int stopsY = stopsForTravel(yHigh - yLow, viewAlong);
    const int total = stopsX * stopsY;

    // a target to measure is chosen first, so a target that can't be used is turned away before the scan
    wrapUpFolder.clear();
    if (kind == ScanTarget) {
        const QString path = QFileDialog::getOpenFileName(this, QString("Which Printed Target Is on the Platen?"), LAULastDirectory::get(),
                                                          QString("Print target (*.tif *.tiff)"));
        if (path.isEmpty()) {
            return;
        }
        LAULastDirectory::rememberFile(path);
        LAUPrintTarget target;
        if (!target.load(path)) {
            cameraWidget->showMessage(QString("Can't measure %1: %2").arg(QFileInfo(path).fileName(), target.error()));
            return;
        }
        measureTarget = target;
        measureName = QFileInfo(path).completeBaseName();
    }

    const QString title = (kind == ScanTarget) ? QString("Scan Target") : (kind == ScanImage) ? QString("Scan Platen Image") : QString("Calibrate Platen");
    if (!cradleIsClear(title)) {
        return;
    }
    QString what;
    if (kind == ScanTarget) {
        what = QString("Take the i1 Pro off the carriage and place the printed target on the platen. The camera finds its %1 patches first; "
                       "the i1 measures them after.").arg(measureTarget.patches.count());
    } else if (kind == ScanImage) {
        what = QString("Take the i1 Pro off the carriage. The camera scans whatever is on the platen and merges it into one picture; "
                       "nothing is measured.");
    } else {
        what = QString("Clear everything off the platen and take the i1 Pro off the carriage: this pass needs to see all the holes, "
                       "and it finds where the i1 measures by looking through the plate's hole.");
    }
    const QMessageBox::StandardButton answer = QMessageBox::warning(this, title, what +
        QString("\n\nThe camera sees %1 x %2 mm of the platen, so the sliders will stop at %3 positions (%4 x %5). Make sure nothing is in their way.")
            .arg(viewAcross, 0, 'f', 0).arg(viewAlong, 0, 'f', 0).arg(total).arg(stopsX).arg(stopsY),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Ok) {
        return;
    }

    // back and forth along the single rail, stepping the heavy rail pair only between rows
    const QList<double> xs = axisStops(xLow, xHigh, stopsX);
    const QList<double> ys = axisStops(yLow, yHigh, stopsY);
    scanStops.clear();
    for (int j = 0; j < ys.count(); j++) {
        for (int i = 0; i < xs.count(); i++) {
            scanStops.append({ xs.at((j % 2) ? xs.count() - 1 - i : i), ys.at(j) });
        }
    }

    scanFolder = QDir(scanBase).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) +
                                              ((kind == ScanTarget) ? QString("_target") : (kind == ScanImage) ? QString("_image") : QString("_bare")));
    scanFrames = QJsonArray();
    scanIndex = 0;
    scanWithTarget = withTarget;
    scanMeasures = (kind == ScanTarget);
    scanAborted = false;
    waitingForFrame = false;
    scanning = true;

    cameraWidget->beginScan();
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(title);
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, &LAUGantryMainWindow::onAbort);
    homeDialog->open();
    moveToNextStop();
}

void LAUGantryMainWindow::moveToNextStop()
{
    const int total = scanStops.count();
    homeDialog->setMessage(QString("Scanning the platen: position %1 of %2...").arg(scanIndex + 1).arg(total));
    homeDialog->setProgress(scanIndex, total);
    railWidget->moveRails(scanStops.at(scanIndex).xMM, scanStops.at(scanIndex).yMM);
}

void LAUGantryMainWindow::onScanMoveFinished(bool ok, const QString &message)
{
    if (!ok) {
        finishScan(scanAborted ? QString("Scan aborted. The sliders stopped where they were. Frames saved so far: %1").arg(scanIndex) : message);
        return;
    }
    scanTimer->start(SCAN_SETTLE_MS);
}

void LAUGantryMainWindow::onScanTimer()
{
    if (!scanning) {
        return;
    }
    if (waitingForFrame) {
        finishScan(QString("The camera stopped delivering frames, so the scan was stopped after %1 positions.").arg(scanIndex));
        return;
    }
    waitingForFrame = true;
    scanTimer->start(SCAN_FRAME_TIMEOUT_MS);
    cameraWidget->requestScanFrame();
}

void LAUGantryMainWindow::onScanFrame(cv::Mat raw16, double exposureMs)
{
    if (!scanning || !waitingForFrame) {
        return;
    }
    waitingForFrame = false;
    scanTimer->stop();

    if (!saveScanFrame(raw16, exposureMs)) {
        finishScan(QString("Couldn't save the scan to %1").arg(QDir::toNativeSeparators(scanFolder)));
        return;
    }
    if (++scanIndex < scanStops.count()) {
        moveToNextStop();
        return;
    }

    returnHomeAfterScan();

    QSettings settings;
    if (scanWithTarget) {
        const QString bare = settings.value(QString("LAUGantryMainWindow::bareScanFolder")).toString();
        if (bare.isEmpty()) {
            wrapUpMessage = QString("Scan saved in %1, but it can't be merged until the platen has been calibrated. Run Calibrate platen first.")
                                .arg(QDir::toNativeSeparators(scanFolder));
            wrapUp();
            return;
        }
        startMerge(bare);
    } else {
        settings.setValue(QString("LAUGantryMainWindow::bareScanFolder"), scanFolder);
        startMerge(QString());
    }
}

bool LAUGantryMainWindow::saveScanFrame(const cv::Mat &raw16, double exposureMs)
{
    if (!QDir().mkpath(scanFolder)) {
        return false;
    }

    // The Python pipeline takes the board frame's X along the rail pair and Y along the single rail
    const double pair = railWidget->reportedMM(2);
    const double single = railWidget->reportedMM(0);
    const QString name = QString::asprintf("frame_%+04.0f_%+04.0f.png", pair, single);

    cv::Mat grey;
    raw16.convertTo(grey, CV_8U, 0.25);              // Mono10 -> 8 bits, which is what scan_map.py reads
    if (!QImage(grey.data, grey.cols, grey.rows, (int)grey.step, QImage::Format_Grayscale8).save(QDir(scanFolder).filePath(name))) {
        return false;
    }

    QJsonObject frame;
    frame["file"] = name;
    frame["carriage_mm"] = QJsonArray{ pair, single };
    frame["averaged"] = 8;
    frame["exposure_ms"] = exposureMs;
    frame["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    scanFrames.append(frame);

    QJsonObject scan;
    scan["image_size"] = QJsonArray{ raw16.cols, raw16.rows };
    scan["pixel_mm"] = LAUBreadboardCalibration::PIXEL_MM;
    scan["focal_mm"] = cameraWidget->calibrationResult().focalMM;      // the merge scales its hole search by this lens
    scan["carriage_axes"] = QString("carriage_mm = [rail pair (Y, SDK axis 2), single rail (X, SDK axis 0)]");
    scan["left_end"] = QJsonObject{ { "x_single_rail", railWidget->homeDirection(0) == 1 ? QString("motor") : QString("idler") },
                                    { "y_rail_pair", railWidget->homeDirection(2) == 1 ? QString("motor") : QString("idler") } };
    scan["frames"] = scanFrames;

    // rewritten after every frame, so an aborted scan is still a valid, shorter scan
    QFile file(QDir(scanFolder).filePath(QString("scan.json")));
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) && file.write(QJsonDocument(scan).toJson()) > 0;
}

void LAUGantryMainWindow::finishScan(const QString &message)
{
    scanning = false;
    waitingForFrame = false;
    scanTimer->stop();
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->endScan(message);
}

void LAUGantryMainWindow::onGoHome()
{
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        cameraWidget->onHomeFinished(false, QString("Tick Enable Sliders first, so the sliders can move to their home position."));
        return;
    }

    railWidget->goToHome();
    if (railWidget->isHoming()) {
        railBox->setEnabled(false);          // hands off the sliders until they get there
        homeDialog = new LAUHomeDialog(this);
        connect(homeDialog, &LAUHomeDialog::emitAbort, railWidget, &LAUMultiVelmexWidget::abortHome);
        homeDialog->open();
    }
}

void LAUGantryMainWindow::onHomeFinished(bool ok, QString message)
{
    if (measuring) {
        onPatchMoveFinished(ok, message);
        return;
    }
    if (scanning) {
        onScanMoveFinished(ok, message);
        return;
    }
    if (returningHome) {
        returningHome = false;
        if (homeDialog) {
            homeDialog->finish();
            homeDialog = nullptr;
        }
        if (!ok) {
            wrapUpNote = QString("\nThe sliders did not return to their home position: %1").arg(message);
        }
        wrapUp();
        return;
    }
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->onHomeFinished(ok, message);
}

void LAUGantryMainWindow::onRailsEnabled(bool state)
{
    if (!state || railWidget->isControllerStarted()) {
        return;
    }

    // the dialog saves the IP/port to the QSettings keys the controller reads when it opens the FMC4030
    LAUFuyuConnectDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        railWidget->connectToController();
    } else {
        railBox->setChecked(false);
    }
}
