/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUGantryMainWindow, measuring a target: after Scan target's camera pass is merged, find the patches, hand over
// to the i1 Pro, and stop on every patch for a reading.
//
#include "laugantrymainwindow.h"

#include <QDir>
#include <QFile>
#include <QLabel>
#include <QTimer>
#include <QSettings>
#include <QDateTime>
#include <QFileInfo>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "lautargetfinder.h"
#include "laui1attachdialog.h"
#include "lauargyllrunner.h"
#include "lauimageinspectdialog.h"

namespace
{
    const int MEASURE_SETTLE_MS = 500;       // the i1 rides on the carriage: let it stop shaking before a reading
    const double STRIP_MM = 0.5;             // patches whose page centres agree to this lie in one strip
    const int I1_BANDS = 36, I1_FIRST_NM = 380, I1_STEP_NM = 10;
}

void LAUGantryMainWindow::setUpMeasuring()
{
    qRegisterMetaType<QVector<double>>("QVector<double>");
    i1 = new LAUi1Worker();
    i1->moveToThread(&i1Thread);
    connect(&i1Thread, &QThread::finished, i1, &QObject::deleteLater);
    connect(this, &LAUGantryMainWindow::emitI1Start, i1, &LAUi1Worker::onStart, Qt::QueuedConnection);
    connect(this, &LAUGantryMainWindow::emitI1Measure, i1, &LAUi1Worker::onMeasure, Qt::QueuedConnection);
    connect(i1, &LAUi1Worker::emitReady, this, &LAUGantryMainWindow::onI1Ready, Qt::QueuedConnection);
    connect(i1, &LAUi1Worker::emitReading, this, &LAUGantryMainWindow::onI1Reading, Qt::QueuedConnection);
    i1Thread.start();
    measureTimer = new QTimer(this);
    measureTimer->setSingleShot(true);
    connect(measureTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onMeasureTimer);
    argyll = new LAUArgyllRunner(this);
    connect(argyll, &LAUArgyllRunner::finished, this, &LAUGantryMainWindow::onSpec2cieFinished);
}

// The camera's pass is merged: find the patches in the picture, confirm them with the user, and turn each into rails
void LAUGantryMainWindow::locateTarget(const QString &mosaicFolder)
{
    LAUPlatenGeometry geometry;
    QString why;
    if (!geometry.load(mosaicFolder, &why)) {
        cameraWidget->showMessage(why);
        return;
    }
    if (!geometry.hasRails()) {
        cameraWidget->showMessage(QString("The platen calibration doesn't know where the i1 looks, so the patches can't be reached. "
                                          "Run Calibrate platen again."));
        return;
    }
    const QImage picture = QImage(QDir(mosaicFolder).filePath(QString("platen_mosaic.png"))).convertToFormat(QImage::Format_Grayscale8);
    if (picture.isNull()) {
        cameraWidget->showMessage(QString("Couldn't open the merged picture in %1").arg(QDir::toNativeSeparators(mosaicFolder)));
        return;
    }
    cv::Mat mosaic(picture.height(), picture.width(), CV_8UC1);
    for (int r = 0; r < picture.height(); r++) {
        memcpy(mosaic.ptr(r), picture.constScanLine(r), picture.width());
    }
    const cv::Matx23d &M = geometry.pictureToBoard;
    const double mmPerPixel = std::hypot(M(0, 0), M(1, 0));

    cameraWidget->showMessage(QString("Finding the target's patches in the merged picture..."));
    const LAUTargetFinderResult found = LAUTargetFinder::findPrint(mosaic, mmPerPixel, measureTarget);
    if (!found.ok) {
        cameraWidget->showMessage(found.message);
        return;
    }

    // every patch centre: picture -> board mm -> rails
    QList<PatchStop> stops;
    QStringList outOfReach;
    for (int k = 0; k < (int)found.centres.size(); k++) {
        double single = 0.0, pair = 0.0;
        if (!geometry.railsFor(geometry.boardOf(found.centres[k]), &single, &pair) || !geometry.reachable(single, pair)) {
            outOfReach << found.locations[k];
            continue;
        }
        stops.append({ k, single, pair });
    }
    if (!outOfReach.isEmpty()) {
        cameraWidget->showMessage(QString("%1 of the target's %2 patches are out of the i1's reach (%3%4). Move the target toward the middle "
                                          "of the platen and Scan target again.")
                                      .arg(outOfReach.count()).arg(found.centres.size())
                                      .arg(QStringList(outOfReach.mid(0, 6)).join(QString(", ")))
                                      .arg(outOfReach.count() > 6 ? QString(", ...") : QString()));
        return;
    }

    // the user checks what the camera found before the i1 goes near it
    QDialog check(this);
    check.setWindowTitle(QString("Target Found"));
    check.resize(1000, 860);
    QVBoxLayout *layout = new QVBoxLayout(&check);
    cv::Mat rgb;
    cv::cvtColor(found.check, rgb, cv::COLOR_BGR2RGB);
    layout->addWidget(new LAUImageInspectView(QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).copy()), 1);
    QLabel *text = new QLabel(found.message + QString("\n\nEach yellow outline should sit on a patch. If they do, the i1 Pro measures them next."));
    text->setWordWrap(true);
    layout->addWidget(text);
    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(QString("Measure with the i1 Pro"));
    connect(buttons, &QDialogButtonBox::accepted, &check, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &check, &QDialog::reject);
    layout->addWidget(buttons);
    if (check.exec() != QDialog::Accepted) {
        cameraWidget->showMessage(QString("The patches were found but not measured."));
        return;
    }

    // One strip of the page at a time, back and forth along the single rail, so the heavy rail pair only steps between
    // strips.  Strips come from the page, not the rails: a sheet laid down a little turned has each strip drift along
    // the rail pair.  Which page direction runs along the rail pair: fit pair = a u + b v + c over the patches.
    cv::Mat lhs((int)stops.count(), 3, CV_64F), rhs((int)stops.count(), 1, CV_64F), fit;
    for (int k = 0; k < stops.count(); k++) {
        const cv::Point2d page = measureTarget.patches.at(stops[k].patch).centreMM();
        lhs.at<double>(k, 0) = page.x;
        lhs.at<double>(k, 1) = page.y;
        lhs.at<double>(k, 2) = 1.0;
        rhs.at<double>(k) = stops[k].pairMM;
    }
    cv::solve(lhs, rhs, fit, cv::DECOMP_SVD);
    const bool stripsAcrossU = std::abs(fit.at<double>(0)) >= std::abs(fit.at<double>(1));
    const double towardPair = (stripsAcrossU ? fit.at<double>(0) : fit.at<double>(1)) >= 0.0 ? 1.0 : -1.0;
    const auto strip = [&](const PatchStop &s) {
        const cv::Point2d page = measureTarget.patches.at(s.patch).centreMM();
        return std::lround(towardPair * (stripsAcrossU ? page.x : page.y) / STRIP_MM);
    };
    std::sort(stops.begin(), stops.end(), [&](const PatchStop &a, const PatchStop &b) {
        const long sa = strip(a), sb = strip(b);
        return (sa != sb) ? sa < sb : a.singleMM < b.singleMM;
    });
    for (int start = 0, row = 0; start < stops.count(); row++) {
        int end = start;
        while (end < stops.count() && strip(stops[end]) == strip(stops[start])) {
            end++;
        }
        if (row % 2) {
            std::reverse(stops.begin() + start, stops.begin() + end);
        }
        start = end;
    }
    patchStops = stops;
    measureFolder = QDir(QFileInfo(mosaicFolder).absolutePath()).filePath(QString("i1"));
    startI1();
}

// the hand-over from the camera to the i1: open it, calibrate it on its tile, then see it go into its cradle
void LAUGantryMainWindow::startI1()
{
    const QMessageBox::StandardButton answer = QMessageBox::information(this, QString("Switching to the i1 Pro"),
        QString("The camera has found all %1 patches. The i1 Pro measures them next.\n\n"
                "Connect the i1 Pro's USB cable and put it on its white tile, then press OK to calibrate it.").arg(patchStops.count()),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok);
    if (answer != QMessageBox::Ok) {
        cameraWidget->showMessage(QString("The patches were found but not measured."));
        return;
    }

    readings = QVector<QVector<double>>(measureTarget.patches.count());
    patchIndex = 0;
    measuring = true;
    measureAborting = false;
    waitingForReading = false;
    cameraWidget->beginScan();               // also holds the exposure still, which the cradle check relies on
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(QString("Switching to the i1 Pro"));
    homeDialog->setMessage(QString("Calibrating the i1 Pro on its white tile..."));
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, &LAUGantryMainWindow::onMeasureAbort);
    homeDialog->open();
    emit emitI1Start();
}

void LAUGantryMainWindow::onI1Ready(bool ok, QString message, int serial)
{
    if (!measuring) {
        return;
    }
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    if (measureAborting) {
        endMeasure(QString("Measuring was aborted before it began."));
        return;
    }
    if (!ok) {
        endMeasure(message);
        return;
    }
    i1Serial = serial;

    const QString bare = QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString();
    const QImage cradle(QDir(QDir(bare).filePath(QString("mosaic"))).filePath(QString("mask.png")));
    if (cradle.isNull()) {
        endMeasure(QString("The platen calibration doesn't say where the i1's cradle is. Run Calibrate platen again."));
        return;
    }
    const QImage grey = cradle.convertToFormat(QImage::Format_Grayscale8);
    cv::Mat mask(grey.height(), grey.width(), CV_8UC1);
    for (int r = 0; r < grey.height(); r++) {
        memcpy(mask.ptr(r), grey.constScanLine(r), grey.width());
    }

    LAUi1AttachDialog attach(mask, this);
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitLiveFrame, &attach, &LAUi1AttachDialog::onFrame);
    if (attach.exec() != QDialog::Accepted) {
        endMeasure(QString("Measuring was cancelled. The patches were found; the i1 Pro is calibrated."));
        return;
    }

    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(QString("Measuring the Target"));
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, &LAUGantryMainWindow::onMeasureAbort);
    homeDialog->open();
    moveToNextPatch();
}

void LAUGantryMainWindow::moveToNextPatch()
{
    const PatchStop &stop = patchStops.at(patchIndex);
    homeDialog->setMessage(QString("Measuring patch %1 (%2 of %3)...").arg(measureTarget.patches.at(stop.patch).location).arg(patchIndex + 1).arg(patchStops.count()));
    homeDialog->setProgress(patchIndex, patchStops.count());
    railWidget->moveRails(stop.singleMM, stop.pairMM);
}

void LAUGantryMainWindow::onPatchMoveFinished(bool ok, const QString &message)
{
    if (!ok) {
        endMeasure(measureAborting ? QString("Measuring aborted after %1 of %2 patches.").arg(patchIndex).arg(patchStops.count()) : message);
        return;
    }
    measureTimer->start(MEASURE_SETTLE_MS);
}

void LAUGantryMainWindow::onMeasureTimer()
{
    if (!measuring) {
        return;
    }
    if (measureAborting) {
        endMeasure(QString("Measuring aborted after %1 of %2 patches.").arg(patchIndex).arg(patchStops.count()));
        return;
    }
    waitingForReading = true;
    emit emitI1Measure(patchStops.at(patchIndex).patch);
}

void LAUGantryMainWindow::onI1Reading(int index, QVector<double> reflectance, QString error)
{
    if (!measuring || !waitingForReading) {
        return;
    }
    waitingForReading = false;
    const QString location = measureTarget.patches.at(index).location;
    if (reflectance.count() != I1_BANDS) {
        endMeasure(QString("The i1 Pro failed on patch %1: %2").arg(location, error));
        return;
    }
    readings[index] = reflectance;

    // kept up to date, so an abort or a failure leaves every reading taken so far on disk
    const QString spectral = QDir(measureFolder).filePath(measureName + QString("_spectral.ti3"));
    if (!QDir().mkpath(measureFolder) || !writeSpectral(spectral)) {
        endMeasure(QString("Couldn't save the readings in %1").arg(QDir::toNativeSeparators(measureFolder)));
        return;
    }
    if (measureAborting) {
        endMeasure(QString("Measuring aborted after %1 of %2 patches. The readings so far are in %3")
                       .arg(patchIndex + 1).arg(patchStops.count()).arg(QDir::toNativeSeparators(spectral)));
        return;
    }
    if (++patchIndex < patchStops.count()) {
        moveToNextPatch();
        return;
    }

    // all measured: Argyll adds XYZ (D50, 2 degree observer) from the spectra, for colprof
    homeDialog->setMessage(QString("Adding XYZ to the readings..."));
    const QString out = QDir(measureFolder).filePath(measureName + QString(".ti3"));
    QFile::remove(out);
    if (!argyll->start(QString("spec2cie"), { spectral, out }, measureFolder)) {
        endMeasure(QString("Measured all %1 patches, but spec2cie couldn't start: %2. The spectra are in %3")
                       .arg(patchStops.count()).arg(argyll->lastError(), QDir::toNativeSeparators(spectral)));
    }
}

void LAUGantryMainWindow::onSpec2cieFinished(QString tool, bool success)
{
    if (!measuring || tool != QString("spec2cie")) {
        return;
    }
    const QString out = QDir(measureFolder).filePath(measureName + QString(".ti3"));
    if (success && QFile::exists(out)) {
        endMeasure(QString("Measured all %1 patches with the i1 Pro. For colprof: %2").arg(patchStops.count()).arg(QDir::toNativeSeparators(out)));
    } else {
        endMeasure(QString("Measured all %1 patches, but spec2cie failed: %2. The spectra are in %3")
                       .arg(patchStops.count())
                       .arg(argyll->lastError(), QDir::toNativeSeparators(QDir(measureFolder).filePath(measureName + QString("_spectral.ti3")))));
    }
}

// The readings so far as Argyll's .ti3, spectra only, with the target's own SAMPLE_ID, SAMPLE_LOC and CMYK
bool LAUGantryMainWindow::writeSpectral(const QString &path) const
{
    QStringList fields{ QString("SAMPLE_ID"), QString("SAMPLE_LOC"), QString("CMYK_C"), QString("CMYK_M"), QString("CMYK_Y"), QString("CMYK_K") };
    for (int b = 0; b < I1_BANDS; b++) {
        fields << QString("SPEC_%1").arg(I1_FIRST_NM + b * I1_STEP_NM);
    }
    QStringList rows;
    for (int k = 0; k < measureTarget.patches.count(); k++) {
        if (readings[k].isEmpty()) {
            continue;
        }
        const LAUPrintPatch &p = measureTarget.patches.at(k);
        QString row = QString("%1 \"%2\" %3 %4 %5 %6").arg(p.sampleId, p.location).arg(p.cmyk[0], 0, 'f', 5).arg(p.cmyk[1], 0, 'f', 5)
                          .arg(p.cmyk[2], 0, 'f', 5).arg(p.cmyk[3], 0, 'f', 5);
        for (double v : readings[k]) {
            row += QString(" %1").arg(v, 0, 'f', 5);
        }
        rows << row;
    }
    QString text;
    text += QString("CTI3\n\nDESCRIPTOR \"Argyll Calibration Target chart information 3\"\nORIGINATOR \"LAU GantryController\"\n");
    text += QString("CREATED \"%1\"\n").arg(QDateTime::currentDateTime().toString(QString("ddd MMM dd HH:mm:ss yyyy")));
    text += QString("KEYWORD \"DEVICE_CLASS\"\nDEVICE_CLASS \"OUTPUT\"\nKEYWORD \"COLOR_REP\"\nCOLOR_REP \"CMYK_XYZ\"\n");
    text += QString("KEYWORD \"TARGET_INSTRUMENT\"\nTARGET_INSTRUMENT \"GretagMacbeth i1 Pro\"\n");
    text += QString("KEYWORD \"INSTRUMENT_SERIAL\"\nINSTRUMENT_SERIAL \"%1\"\n").arg(i1Serial);
    text += QString("KEYWORD \"SPECTRAL_BANDS\"\nSPECTRAL_BANDS \"%1\"\n").arg(I1_BANDS);
    text += QString("KEYWORD \"SPECTRAL_START_NM\"\nSPECTRAL_START_NM \"%1.000000\"\n").arg(I1_FIRST_NM);
    text += QString("KEYWORD \"SPECTRAL_END_NM\"\nSPECTRAL_END_NM \"%1.000000\"\n").arg(I1_FIRST_NM + (I1_BANDS - 1) * I1_STEP_NM);
    text += QString("KEYWORD \"SPECTRAL_NORM\"\nSPECTRAL_NORM \"100.000000\"\n\n");
    text += QString("NUMBER_OF_FIELDS %1\nBEGIN_DATA_FORMAT\n%2\nEND_DATA_FORMAT\n\n").arg(fields.count()).arg(fields.join(QChar(' ')));
    text += QString("NUMBER_OF_SETS %1\nBEGIN_DATA\n%2\nEND_DATA\n").arg(rows.count()).arg(rows.join(QChar('\n')));
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text) && file.write(text.toLatin1()) > 0;
}

void LAUGantryMainWindow::onMeasureAbort()
{
    measureAborting = true;
    if (railWidget->isHoming()) {
        railWidget->abortHome();             // the move ends, then onPatchMoveFinished finishes up
    }
    // otherwise a reading or the tile calibration is under way: it finishes, and the abort is seen after it
}

void LAUGantryMainWindow::endMeasure(const QString &message)
{
    measuring = false;
    waitingForReading = false;
    measureTimer->stop();
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->endScan(message);
}
