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
#include <QVBoxLayout>
#include <cmath>

namespace
{
    // the same spacing as the scans the Python pipeline was proven on, across the full travel
    const int SCAN_STOPS_X = 7;              // single rail, 610 mm: the light axis does the stepping
    const int SCAN_STOPS_Y = 5;              // rail pair, 310 mm
    const int SCAN_SETTLE_MS = 500;
    const int SCAN_FRAME_TIMEOUT_MS = 15000;
    const double HALF_PITCH_MM = 12.7;

    // Evenly spaced stops.  A step that is a whole number of half hole-pitches would hide a wrong
    // direction from the Python calibration (see vision/scan_map.py), so nudge the step off them.
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
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitScanFrame, this, &LAUGantryMainWindow::onScanFrame);
    scanBase = QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).filePath(QString("GantryScans"));
    scanTimer = new QTimer(this);
    scanTimer->setSingleShot(true);
    connect(scanTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onScanTimer);
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
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        cameraWidget->showMessage(QString("Tick Enable Sliders first, so the sliders can scan the platen."));
        return;
    }

    double xLow, xHigh, yLow, yHigh;
    if (!railWidget->scanRange(0, &xLow, &xHigh) || !railWidget->scanRange(2, &yLow, &yHigh)) {
        cameraWidget->showMessage(QString("The sliders' travel isn't known. Press Calibrate on both sliders first."));
        return;
    }

    const int total = SCAN_STOPS_X * SCAN_STOPS_Y;
    const QString title = withTarget ? QString("Scan Platen") : QString("Calibrate Platen");
    const QMessageBox::StandardButton answer = QMessageBox::warning(this, title,
        (withTarget ? QString("Place the target on the platen.") : QString("Clear everything off the platen: this pass needs to see all the holes.")) +
        QString("\n\nThe sliders will travel across the whole platen and stop at %1 positions. Make sure nothing is in their way.").arg(total),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Ok) {
        return;
    }

    // back and forth along the single rail, stepping the heavy rail pair only between rows
    const QList<double> xs = axisStops(xLow, xHigh, SCAN_STOPS_X);
    const QList<double> ys = axisStops(yLow, yHigh, SCAN_STOPS_Y);
    scanStops.clear();
    for (int j = 0; j < ys.count(); j++) {
        for (int i = 0; i < xs.count(); i++) {
            scanStops.append({ xs.at((j % 2) ? xs.count() - 1 - i : i), ys.at(j) });
        }
    }

    scanFolder = QDir(scanBase).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) + (withTarget ? QString("_target") : QString("_bare")));
    scanFrames = QJsonArray();
    scanIndex = 0;
    scanWithTarget = withTarget;
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

    const QString folder = QDir::toNativeSeparators(scanFolder);
    QSettings settings;
    QString command;
    if (scanWithTarget) {
        const QString bare = settings.value(QString("LAUGantryMainWindow::bareScanFolder")).toString();
        command = bare.isEmpty() ? QString("Run a bare platen scan too, then in vision: python mosaic.py \"%1\" --calib \"<bare scan>\\out\"").arg(folder)
                                 : QString("Next, in vision: python mosaic.py \"%1\" --calib \"%2\\out\"").arg(folder, bare);
    } else {
        settings.setValue(QString("LAUGantryMainWindow::bareScanFolder"), folder);
        command = QString("Next, in vision: python scan_map.py \"%1\" --axes %2x,%3y").arg(folder)
                      .arg(railWidget->homeDirection(2) == 1 ? QString("+") : QString("-"))
                      .arg(railWidget->homeDirection(0) == 1 ? QString("+") : QString("-"));
    }
    finishScan(QString("Scan complete: %1 frames saved to %2\n%3").arg(scanStops.count()).arg(folder, command));
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
    scan["focal_mm"] = LAUBreadboardCalibration::NOMINAL_FOCAL_MM;
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
    if (scanning) {
        onScanMoveFinished(ok, message);
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
