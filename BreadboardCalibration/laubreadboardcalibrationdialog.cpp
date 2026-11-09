/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laubreadboardcalibrationdialog.h"

#include <QDir>
#include <QUrl>
#include <QDateTime>
#include <QSettings>
#include <QVariantList>
#include <QPushButton>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QDesktopServices>
#include <QStandardPaths>
#include <opencv2/calib3d.hpp>

namespace
{
    // what the live dialog tracks, and how much it may change over the last few frames and still count as steady
    struct LAULiveParameter
    {
        const char *name;
        const char *unit;
        int decimals;
        double steadyChange;
    };
    const LAULiveParameter LIVE_PARAMETERS[] = { { "Focal length", " mm", 3, 0.02 },
                                                 { "k1", "", 4, 0.005 },
                                                 { "k2", "", 4, 0.01 },
                                                 { "Tilt", "°", 2, 0.1 },
                                                 { "Height", " mm", 1, 1.0 } };
}

LAULiveCalibrationDialog::LAULiveCalibrationDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Live Calibration"));
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);
    setModal(true);
    resize(1120, 860);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setSpacing(8);

    view = new LAUImageView();
    layout->addWidget(view, 1);

    QGridLayout *grid = new QGridLayout();
    grid->setHorizontalSpacing(24);
    grid->addWidget(new QLabel(QString("Now")), 1, 0);
    grid->addWidget(new QLabel(QString("Change, last %1 frames").arg((int)HistoryLength)), 2, 0);
    for (int n = 0; n < ParameterCount; n++) {
        QLabel *name = new QLabel(QString::fromUtf8(LIVE_PARAMETERS[n].name));
        name->setAlignment(Qt::AlignRight);
        nowLabels[n] = new QLabel(QString("—"));
        nowLabels[n]->setAlignment(Qt::AlignRight);
        changeLabels[n] = new QLabel(QString("—"));
        changeLabels[n]->setAlignment(Qt::AlignRight);
        grid->addWidget(name, 0, n + 1);
        grid->addWidget(nowLabels[n], 1, n + 1);
        grid->addWidget(changeLabels[n], 2, n + 1);
    }
    layout->addLayout(grid);

    verdict = new QLabel();
    verdict->setWordWrap(true);
    QFont f = verdict->font();
    f.setPointSize(f.pointSize() + 3);
    verdict->setFont(f);
    layout->addWidget(verdict);
    setVerdict(QString("Looking for the breadboard's holes..."), QString("gray"));

    QLabel *hint = new QLabel(QString("Right-click the picture to change what it shows."));
    hint->setStyleSheet(QString("color: gray;"));
    layout->addWidget(hint);

    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    okButton = buttons->button(QDialogButtonBox::Ok);
    okButton->setText(QString("OK - lock in this calibration"));
    okButton->setEnabled(false);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    setContextMenuPolicy(Qt::ActionsContextMenu);
    viewActions = new QActionGroup(this);
    const QStringList names = { QString("Live camera"), QString("Holes found"), QString("Lens corrected"),
                                QString("Lines into the distance"), QString("Vanishing point") };
    for (int n = 0; n < names.count(); n++) {
        QAction *a = new QAction(names.at(n), this);
        a->setCheckable(true);
        a->setData(n);
        viewActions->addAction(a);
        addAction(a);
    }
    viewActions->actions().at(ViewHoles)->setChecked(true);
    connect(viewActions, &QActionGroup::triggered, this, &LAULiveCalibrationDialog::showView);
}

void LAULiveCalibrationDialog::setVerdict(const QString &text, const QString &color)
{
    verdict->setText(text);
    verdict->setStyleSheet(QString("color: %1;").arg(color));
}

void LAULiveCalibrationDialog::onFrame(cv::Mat raw16)
{
    liveFrame = raw16;
    if (viewActions->checkedAction()->data().toInt() == ViewLive || !lastGood.ok) {
        view->setFrame(liveFrame);
    }
}

void LAULiveCalibrationDialog::onResult(LAUBreadboardCalibrationResult r)
{
    if (!r.ok) {
        history.clear();                     // steady means steady in a row
        for (int n = 0; n < ParameterCount; n++) {
            nowLabels[n]->setText(QString("—"));
            changeLabels[n]->setText(QString("—"));
            changeLabels[n]->setStyleSheet(QString());
        }
        setVerdict(r.message + (lastGood.ok ? QString(" OK will lock in the last valid calibration.") : QString()), QString("#c62828"));
        return;
    }

    lastGood = r;
    okButton->setEnabled(true);
    history.append(QVector<double>({ r.focalMM, r.dist[0], r.dist[1], r.tiltDeg, r.heightMM }));
    if (history.count() > HistoryLength) {
        history.removeFirst();
    }

    bool steady = history.count() >= SteadyFrames;
    for (int n = 0; n < ParameterCount; n++) {
        double lowest = history.first().at(n);
        double highest = lowest;
        for (const QVector<double> &frame : history) {
            lowest = qMin(lowest, frame.at(n));
            highest = qMax(highest, frame.at(n));
        }
        const LAULiveParameter &p = LIVE_PARAMETERS[n];
        const double change = highest - lowest;
        const bool withinLimit = change <= p.steadyChange;
        steady = steady && withinLimit;
        nowLabels[n]->setText(QString::number(history.last().at(n), 'f', p.decimals) + QString::fromUtf8(p.unit));
        changeLabels[n]->setText(QString::number(change, 'f', p.decimals) + QString::fromUtf8(p.unit));
        changeLabels[n]->setStyleSheet(QString("color: %1;").arg(withinLimit ? QString("#2e7d32") : QString("#c62828")));
    }

    const QString quality = QString("%1 holes, accuracy %2 px.").arg(r.holesUsed).arg(r.rms, 0, 'f', 2);
    if (history.count() < SteadyFrames) {
        setVerdict(QString("Measuring... %1").arg(quality), QString("gray"));
    } else if (steady) {
        setVerdict(QString("Steady. %1 Press OK to lock it in.").arg(quality), QString("#2e7d32"));
    } else {
        setVerdict(QString("Still changing from frame to frame. %1 Check that nothing is moving.").arg(quality), QString("#ef6c00"));
    }
    showView();
}

void LAULiveCalibrationDialog::showView()
{
    const int id = viewActions->checkedAction()->data().toInt();
    if (id == ViewLive || !lastGood.ok) {
        view->setFrame(liveFrame);
    } else if (id == ViewHoles) {
        view->setImage(lastGood.holes);
    } else if (id == ViewCorrected) {
        view->setImage(lastGood.undistorted);
    } else if (id == ViewLines) {
        view->setImage(lastGood.lines);
    } else {
        view->setImage(lastGood.vanishing);
    }
}

LAUBreadboardCalibrationDialog::LAUBreadboardCalibrationDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Breadboard Camera Calibration"));
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint);
    resize(1280, 940);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);
    layout->addWidget(new LAUBreadboardCalibrationWidget(), 1);

    QHBoxLayout *buttons = new QHBoxLayout();
    QPushButton *closeButton = new QPushButton(QString("Close"));
    closeButton->setMinimumHeight(44);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    buttons->addStretch();
    buttons->addWidget(closeButton);
    layout->addLayout(buttons);
}

LAUBreadboardCalibrationWidget::LAUBreadboardCalibrationWidget(QWidget *parent) : QWidget(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    // every command is on the right-click menu
    setContextMenuPolicy(Qt::ActionsContextMenu);

    // plain text items: without this the Windows style reserves an empty icon column on the left
    setStyleSheet(QString("QMenu { background: palette(window); border: 1px solid palette(mid); padding: 4px; }"
                          "QMenu::item { padding: 6px 24px 6px 12px; color: palette(window-text); }"
                          "QMenu::item:selected { background: palette(highlight); color: palette(highlighted-text); }"
                          "QMenu::item:disabled { color: palette(mid); }"));

    calibrateAction = new QAction(QString("Calibrate"), this);
    calibrateAction->setEnabled(false);
    connect(calibrateAction, &QAction::triggered, this, &LAUBreadboardCalibrationWidget::onCalibrateClicked);
    addAction(calibrateAction);

    folderAction = new QAction(QString("Open results folder"), this);
    folderAction->setEnabled(false);
    connect(folderAction, &QAction::triggered, this, &LAUBreadboardCalibrationWidget::onOpenFolderClicked);
    addAction(folderAction);

    exportAction = new QAction(QString("Export frame to disk..."), this);
    connect(exportAction, &QAction::triggered, this, &LAUBreadboardCalibrationWidget::onExportFrame);
    addAction(exportAction);

    view = new LAUImageView();
    layout->addWidget(view, 1);

    camera = new QLabel(QString("Connecting to the camera..."));
    camera->setStyleSheet(QString("color: gray;"));
    layout->addWidget(camera);

    status = new QLabel(QString("Point the camera at the breadboard, then right-click the picture and choose Calibrate."));
    status->setWordWrap(true);
    QFont f = status->font();
    f.setPointSize(f.pointSize() + 3);
    status->setFont(f);
    layout->addWidget(status);

    // calibration runs in its own thread so the live picture keeps moving
    calibration = new LAUBreadboardCalibration();
    calibration->moveToThread(&calibrationThread);
    connect(&calibrationThread, &QThread::finished, calibration, &QObject::deleteLater);
    connect(this, &LAUBreadboardCalibrationWidget::emitCalibrate, calibration, &LAUBreadboardCalibration::onCalibrate, Qt::QueuedConnection);
    connect(calibration, &LAUBreadboardCalibration::emitResult, this, &LAUBreadboardCalibrationWidget::onResult, Qt::QueuedConnection);
    calibrationThread.start();

    grabber = new LAUBaslerGrabber(this);
    connect(grabber, &LAUBaslerGrabber::emitFrame, this, &LAUBreadboardCalibrationWidget::onFrame, Qt::QueuedConnection);
    connect(grabber, &LAUBaslerGrabber::emitAveragedFrame, this, &LAUBreadboardCalibrationWidget::onAveragedFrame, Qt::QueuedConnection);
    connect(grabber, &LAUBaslerGrabber::emitError, this, &LAUBreadboardCalibrationWidget::onCameraError, Qt::QueuedConnection);
    connect(grabber, &LAUBaslerGrabber::emitCameraName, this, [this](QString name) {
        camera->setText(QString("Camera: %1").arg(name));
    });
    grabber->start();

    if (restoreLens()) {
        useLensInView();
    }
}

LAUBreadboardCalibrationWidget::~LAUBreadboardCalibrationWidget()
{
    grabber->stop();
    calibrationThread.quit();
    calibrationThread.wait();
}

void LAUBreadboardCalibrationWidget::setStatus(const QString &text)
{
    status->setText(text);
}

void LAUBreadboardCalibrationWidget::onFrame(cv::Mat raw16, double exposureMs)
{
    liveFrame = raw16;
    if (liveDialog) {
        liveDialog->onFrame(raw16);
    }
    if (!busy) {
        calibrateAction->setEnabled(true);
    }
    camera->setText(camera->text().section(QString("   "), 0, 0) + QString("   exposure %1 ms").arg(exposureMs, 0, 'f', 1));
    // the main picture rests behind the live dialog: the GPU only needs to do one of them
    if (!liveDialog) {
        showView();
    }
    grabber->frameDone();
}

void LAUBreadboardCalibrationWidget::onCameraError(QString message)
{
    if (liveDialog) {
        liveDialog->reject();
    }
    busy = false;
    camera->setText(message);
    setStatus(QString("The camera isn't available. Check its USB cable, and that no other program is using it, "
                      "then reopen this window."));
    calibrateAction->setEnabled(false);
}

void LAUBreadboardCalibrationWidget::onCalibrateClicked()
{
    const QMessageBox::StandardButton answer = QMessageBox::warning(this, QString("Calibrate"),
        QString("Calibration requires the sliders to be in their home position:\n\n"
                "    X axis in the middle of its travel\n"
                "    Y axis at the idler end\n\n") +
        (homeMoveFlag ? QString("Press OK to move them there now. Make sure nothing is in their way.")
                      : QString("Press OK once they are there.")),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);

    // the camera may have dropped out while the warning was up
    if (answer != QMessageBox::Ok || !calibrateAction->isEnabled()) {
        return;
    }

    busy = true;
    calibrateAction->setEnabled(false);
    if (homeMoveFlag) {
        setStatus(QString("Moving the sliders to their home position..."));
        emit emitGoHome();
    } else {
        startLive();
    }
}

void LAUBreadboardCalibrationWidget::onHomeFinished(bool ok, QString message)
{
    if (!busy) {
        return;
    }
    if (!ok) {
        busy = false;
        calibrateAction->setEnabled(true);
        setStatus(message);
        return;
    }
    startLive();
}

void LAUBreadboardCalibrationWidget::startLive()
{
    setStatus(QString("Checking the calibration on the live picture..."));
    liveDialog = new LAULiveCalibrationDialog(this);
    connect(liveDialog, &QDialog::finished, this, &LAUBreadboardCalibrationWidget::onLiveFinished);
    liveDialog->open();
    requestLiveFrame();
}

void LAUBreadboardCalibrationWidget::requestLiveFrame()
{
    liveInFlight = true;
    grabber->requestAverage(8);
}

void LAUBreadboardCalibrationWidget::onLiveFinished(int code)
{
    const LAUBreadboardCalibrationResult r = liveDialog->lockedResult();
    liveDialog->deleteLater();
    liveDialog = nullptr;
    if (code == QDialog::Accepted && r.ok) {
        applyResult(r);
    } else {
        busy = false;
        calibrateAction->setEnabled(true);
        setStatus(QString("Calibration cancelled. Nothing was saved."));
    }
}

void LAUBreadboardCalibrationWidget::enablePlatenScan()
{
    // the gantry window's menu is just its three jobs; the picture follows them by itself
    for (QAction *action : actions()) {
        if (action != calibrateAction) {
            removeAction(action);
        }
    }
    calibrateAction->setText(QString("Calibrate camera"));
    setStatus(QString("Point the camera at the breadboard, then right-click the picture and choose Calibrate camera."));

    scanBareAction = new QAction(QString("Calibrate platen"), this);
    scanBareAction->setEnabled(false);
    connect(scanBareAction, &QAction::triggered, this, [this]() { emit emitScanRequested(false); });
    addAction(scanBareAction);

    scanTargetAction = new QAction(QString("Scan platen"), this);
    scanTargetAction->setEnabled(false);
    connect(scanTargetAction, &QAction::triggered, this, [this]() { emit emitScanRequested(true); });
    addAction(scanTargetAction);

    guidesAction = new QAction(QString("Show platen guides"), this);
    guidesAction->setCheckable(true);
    guidesAction->setChecked(true);
    connect(guidesAction, &QAction::toggled, this, [this](bool state) { view->setOverlay(state ? guides : LAUViewOverlay()); });
    addAction(guidesAction);
    addAction(exportAction);

    if (result.ok) {                         // restored from the last session
        scanBareAction->setEnabled(true);
        scanTargetAction->setEnabled(true);
        setStatus(result.message);
    }
}

void LAUBreadboardCalibrationWidget::beginScan()
{
    busy = true;
    calibrateAction->setEnabled(false);
    scanBareAction->setEnabled(false);
    scanTargetAction->setEnabled(false);
    grabber->setAutoExposure(false);
}

void LAUBreadboardCalibrationWidget::endScan(const QString &message)
{
    busy = false;
    scanFrameRequested = false;
    grabber->setAutoExposure(true);
    calibrateAction->setEnabled(true);
    scanBareAction->setEnabled(true);
    scanTargetAction->setEnabled(true);
    setStatus(message);
}

void LAUBreadboardCalibrationWidget::requestScanFrame()
{
    scanFrameRequested = true;
    grabber->requestAverage(8);
}

void LAUBreadboardCalibrationWidget::onAveragedFrame(cv::Mat raw16, double exposureMs)
{
    if (scanFrameRequested) {
        scanFrameRequested = false;
        emit emitScanFrame(raw16, exposureMs);
        return;
    }
    if (!liveInFlight) {
        setStatus(QString("Finding the holes and measuring the lens..."));
    }
    emit emitCalibrate(raw16, exposureMs);
}

void LAUBreadboardCalibrationWidget::onResult(LAUBreadboardCalibrationResult r)
{
    if (liveInFlight) {
        liveInFlight = false;
        if (liveDialog) {                    // otherwise it came back after the dialog closed: drop it
            liveDialog->onResult(r);
            requestLiveFrame();
        }
        return;
    }
    applyResult(r);
}

void LAUBreadboardCalibrationWidget::applyResult(const LAUBreadboardCalibrationResult &r)
{
    busy = false;
    calibrateAction->setEnabled(true);
    if (!r.ok) {
        setStatus(r.message);
        return;
    }
    result = r;
    const QString base = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString folder = QDir(base).filePath(QString("BreadboardCalibration/%1").arg(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss"))));
    QString err;
    if (LAUBreadboardCalibration::save(result, folder, &err)) {
        savedFolder = folder;
        folderAction->setEnabled(true);
        setStatus(result.message + QString("\nSaved to %1").arg(QDir::toNativeSeparators(folder)));
    } else {
        setStatus(result.message + QString("\nCouldn't save the results: %1").arg(err));
    }

    saveLens();
    useLensInView();
    showView();
    if (scanBareAction) {
        scanBareAction->setEnabled(true);
        scanTargetAction->setEnabled(true);
    }
}

void LAUBreadboardCalibrationWidget::useLensInView()
{
    // the same camera matrix as result.undistorted, so the live video lines up with it
    const cv::Mat K(result.K);
    const cv::Mat dist(1, 5, CV_64F, result.dist);
    const cv::Mat newK = cv::getOptimalNewCameraMatrix(K, dist, result.imageSize, 0);
    view->setUndistortion(result.K, result.dist, cv::Matx33d(newK));
    view->enableUndistortion(true);
}

void LAUBreadboardCalibrationWidget::saveLens() const
{
    QVariantList k, d;
    for (int n = 0; n < 9; n++) {
        k << result.K.val[n];
    }
    for (int n = 0; n < 5; n++) {
        d << result.dist[n];
    }
    QSettings settings;
    settings.beginGroup(QString("LAUBreadboardCalibrationWidget"));
    settings.setValue(QString("K"), k);
    settings.setValue(QString("dist"), d);
    settings.setValue(QString("imageWidth"), result.imageSize.width);
    settings.setValue(QString("imageHeight"), result.imageSize.height);
    settings.setValue(QString("focalMM"), result.focalMM);
    settings.setValue(QString("tiltDeg"), result.tiltDeg);
    settings.setValue(QString("heightMM"), result.heightMM);
    settings.setValue(QString("rms"), result.rms);
    settings.setValue(QString("savedFolder"), savedFolder);
    settings.setValue(QString("calibratedAt"), QDateTime::currentDateTime().toString(QString("yyyy-MM-dd HH:mm")));
}

bool LAUBreadboardCalibrationWidget::restoreLens()
{
    QSettings settings;
    settings.beginGroup(QString("LAUBreadboardCalibrationWidget"));
    const QVariantList k = settings.value(QString("K")).toList();
    const QVariantList d = settings.value(QString("dist")).toList();
    const cv::Size size(settings.value(QString("imageWidth"), 0).toInt(), settings.value(QString("imageHeight"), 0).toInt());
    const double heightMM = settings.value(QString("heightMM"), 0.0).toDouble();
    if (k.count() != 9 || d.count() != 5 || size.width <= 0 || size.height <= 0 || heightMM <= 0.0 || k.at(0).toDouble() <= 0.0) {
        return false;
    }

    for (int n = 0; n < 9; n++) {
        result.K.val[n] = k.at(n).toDouble();
    }
    for (int n = 0; n < 5; n++) {
        result.dist[n] = d.at(n).toDouble();
    }
    result.imageSize = size;
    result.heightMM = heightMM;
    result.focalMM = settings.value(QString("focalMM"), 0.0).toDouble();
    result.tiltDeg = settings.value(QString("tiltDeg"), 0.0).toDouble();
    result.rms = settings.value(QString("rms"), 0.0).toDouble();
    result.ok = true;
    result.message = QString("Using the saved lens calibration from %1 (focal length %2 mm). If the lens or camera has moved, right-click and calibrate again.")
                         .arg(settings.value(QString("calibratedAt")).toString()).arg(result.focalMM, 0, 'f', 2);

    savedFolder = settings.value(QString("savedFolder")).toString();
    folderAction->setEnabled(!savedFolder.isEmpty());
    setStatus(result.message);
    return true;
}

void LAUBreadboardCalibrationWidget::showView()
{
    view->setFrame(liveFrame);
}

void LAUBreadboardCalibrationWidget::setGuides(const LAUViewOverlay &overlay)
{
    guides = overlay;
    if (guidesAction && guidesAction->isChecked()) {
        view->setOverlay(guides);
    }
}

void LAUBreadboardCalibrationWidget::onExportFrame()
{
    if (liveFrame.empty()) {
        setStatus(QString("There is no camera frame to export yet."));
        return;
    }

    // the frame as shown (levelled, and lens corrected once calibrated) at full resolution, whatever the zoom;
    // taken now so the file holds the moment of the click, not the moment the file name is chosen
    cv::Mat shown = LAUBreadboardCalibration::picture(liveFrame);
    if (result.ok && result.imageSize == shown.size()) {
        const cv::Mat K(result.K);
        const cv::Mat dist(1, 5, CV_64F, result.dist);
        cv::Mat corrected;
        cv::undistort(shown, corrected, K, dist, cv::getOptimalNewCameraMatrix(K, dist, result.imageSize, 0));
        shown = corrected;
    }
    const QImage picture = LAUBreadboardCalibration::toQImage(shown);

    const QString base = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString name = QString("frame_%1.png").arg(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")));
    const QString path = QFileDialog::getSaveFileName(this, QString("Export Frame"), QDir(base).filePath(name),
                                                      QString("PNG image (*.png);;TIFF image (*.tif *.tiff);;JPEG image (*.jpg *.jpeg)"));
    if (path.isEmpty()) {
        return;
    }
    if (picture.save(path)) {
        setStatus(QString("Frame saved to %1").arg(QDir::toNativeSeparators(path)));
    } else {
        QMessageBox::warning(this, QString("Export Frame"), QString("Couldn't save the frame to %1").arg(QDir::toNativeSeparators(path)));
    }
}

void LAUBreadboardCalibrationWidget::onOpenFolderClicked()
{
    if (!savedFolder.isEmpty()) {
        QDesktopServices::openUrl(QUrl::fromLocalFile(savedFolder));
    }
}
