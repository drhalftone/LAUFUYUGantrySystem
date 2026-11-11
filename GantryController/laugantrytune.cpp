/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Fine-tune page corners: the rails put the i1 holder's hole over each corner of the page found by the last scan
// and centre the corner in it, and the rails' positions at the centred corners give a correction from the model's
// rail positions to the true ones.  It is saved with that scan's platen_mosaic.json, so LAUPlatenGeometry::railsFor
// applies it to every point of that picture.
//
// The camera does the centring (LAUHoleView).  For each corner the rails first stop over plain paper 12 mm in from
// both edges, where the hole shows only paper, and the bright disc says exactly where the hole is.  Then, over the
// corner, the two paper edges seen through the hole give the corner, and the rails move by the miss until it is
// under TOLERANCE_MM.  When the camera can't do it, the dialog's arrows let the user centre that corner by hand.
//
#include "laugantrymainwindow.h"
#include "laucornertunedialog.h"

#include <cmath>
#include <complex>
#include <QDir>
#include <QFile>
#include <QTimer>
#include <QDateTime>
#include <QSettings>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QMessageBox>
#include <QPushButton>
#include <QJsonDocument>
#include <QImage>
#include <QLabel>
#include <QDialog>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <opencv2/imgproc.hpp>

namespace
{
    const double LARGE_NUDGE_MM = 5.0;       // more than this at a corner suggests the wrong corner was centred
    const double INSETS_MM[] = { 12.0, 20.0, 30.0 };   // where the hole is seen on plain paper, in from both edges
    const double TOLERANCE_MM = 0.1;         // a corner this close to the hole's centre is centred (the rails repeat to about this)
    const int MAX_MOVES = 4;
    const double STUCK_MM = 0.02;            // a move smaller than this, at the end of the travel, isn't worth making
    const int SETTLE_MS = 500;
    const int FRAME_TIMEOUT_MS = 15000;
    const double ZOOM_HALF_MM = 12.0;        // the dialog's view around the hole, each way

    // rails kept within the i1's travel
    void clampToTravel(const LAUPlatenGeometry &g, double *single, double *pair)
    {
        *single = std::min(std::max(*single, g.singleLow), g.singleHigh);
        *pair = std::min(std::max(*pair, g.pairLow), g.pairHigh);
    }

    cv::Point2d unitOf(const cv::Point2d &v)
    {
        return v * (1.0 / std::max(cv::norm(v), 1e-12));
    }
}

void LAUGantryMainWindow::onCornerTuneRequested()
{
    const QString title = QString("Fine-tune Page Corners");
    if (scanning || measuring || merging || returningHome || tuning || railWidget->isHoming()) {
        cameraWidget->showMessage(QString("Wait until the sliders and the merge are finished, then try again."));
        return;
    }
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        QMessageBox::information(this, title, QString("Tick Enable Sliders first, so the sliders can move to the page's corners."));
        return;
    }

    // the newest scan in which a page was found
    QDir base(scanBase);
    tuneFolder.clear();
    for (const QString &name : base.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        if (QFile::exists(base.filePath(name + QString("/mosaic/page.json")))) {
            tuneFolder = base.filePath(name + QString("/mosaic"));
            break;
        }
    }
    if (tuneFolder.isEmpty()) {
        QMessageBox::information(this, title, QString("No page has been found on the platen yet. Put the page on the platen and choose Scan platen image first."));
        return;
    }
    const QString scanName = QFileInfo(QFileInfo(tuneFolder).absolutePath()).fileName();
    QFile file(QDir(tuneFolder).filePath(QString("page.json")));
    const QJsonObject page = file.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject();
    const QJsonArray corners = page["corners_board_mm"].toArray();
    QString error;
    if (corners.count() != 4) {
        QMessageBox::warning(this, title, QString("The page found in %1 couldn't be read. Scan the platen image again.").arg(scanName));
        return;
    }
    if (!tuneGeometry.load(tuneFolder, &error) || !tuneGeometry.hasRails()) {
        QMessageBox::warning(this, title, error.isEmpty() ? QString("Scan %1 doesn't know where the i1's hole is: run Calibrate platen with the i1 off, then scan the page again.").arg(scanName) : error);
        return;
    }
    holeView.load(QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString());

    // the travel is wherever the limit sliders are now, without the scans' 5 mm margin from the end switches: a corner
    // at the edge of the platen may need it, and the sliders only move between them anyway
    tuneSavedTravel[0] = tuneGeometry.singleLow;
    tuneSavedTravel[1] = tuneGeometry.singleHigh;
    tuneSavedTravel[2] = tuneGeometry.pairLow;
    tuneSavedTravel[3] = tuneGeometry.pairHigh;
    double low = 0.0, high = 0.0;
    if (railWidget->limitRange(0, &low, &high)) {
        tuneGeometry.singleLow = low;
        tuneGeometry.singleHigh = high;
    }
    if (railWidget->limitRange(2, &low, &high)) {
        tuneGeometry.pairLow = low;
        tuneGeometry.pairHigh = high;
    }

    // as the merged picture shows them: page.json lists them clockwise from the picture's top-left
    const QStringList names{ QString("top-left"), QString("top-right"), QString("bottom-right"), QString("bottom-left") };
    std::vector<cv::Point2d> P;
    for (int k = 0; k < 4; k++) {
        P.emplace_back(corners[k].toArray().at(0).toDouble(), corners[k].toArray().at(1).toDouble());
    }
    const auto onPaper = [this](double ds, double dp) {
        return cv::norm(cv::Point2d(tuneGeometry.ex[0], tuneGeometry.ex[1]) * dp + cv::Point2d(tuneGeometry.ey[0], tuneGeometry.ey[1]) * ds);
    };
    tuneCorners.clear();
    QStringList unreachable;
    for (int k = 0; k < 4; k++) {
        TuneCorner c;
        c.name = names[k];
        c.board = P[k];
        tuneGeometry.railsFor(c.board, &c.plannedSingle, &c.plannedPair, false);
        tuneGeometry.railsFor(c.board, &c.startSingle, &c.startPair, true);
        // a corner just past the travel: the rails go as near as they can, and the camera sees it off-centre in the hole
        c.nearSingle = c.startSingle;
        c.nearPair = c.startPair;
        clampToTravel(tuneGeometry, &c.nearSingle, &c.nearPair);
        c.single = c.nearSingle;
        c.pair = c.nearPair;
        c.reachable = onPaper(c.nearSingle - c.startSingle, c.nearPair - c.startPair) < STUCK_MM;
        for (double inset : INSETS_MM) {
            const cv::Point2d at = P[k] + unitOf(P[(k + 1) % 4] - P[k]) * inset + unitOf(P[(k + 3) % 4] - P[k]) * inset;
            double single = 0.0, pair = 0.0;
            tuneGeometry.railsFor(at, &single, &pair, true);
            if (tuneGeometry.reachable(single, pair)) {
                c.insets << cv::Point2d(single, pair);
            }
        }
        if (!c.reachable) {
            unreachable << c.name;
        }
        tuneCorners << c;
    }

    QString text = QString("The sliders will put the i1's hole over each corner of the %1 x %2 mm page found in scan %3, and the camera will "
                           "centre each corner in the hole through it.\n\n"
                           "Before starting, take the i1 Pro out of its holder so the camera sees the paper through the hole, and don't move the page.")
                       .arg(page["width_mm"].toDouble(), 0, 'f', 1).arg(page["length_mm"].toDouble(), 0, 'f', 1).arg(scanName);
    if (!holeView.isValid()) {
        text += QString("\n\nThe platen calibration can't be read, so you'll centre each corner by hand with the arrows.");
    }
    if (!unreachable.isEmpty()) {
        text += QString("\n\nThe %1 corner%2 may be past the i1's travel: the sliders will go as close as they can, and you'll be told if %3 can't be reached.")
                    .arg(unreachable.join(QString(", "))).arg(unreachable.count() > 1 ? "s" : "").arg(unreachable.count() > 1 ? "they" : "it");
    }
    if (QMessageBox::information(this, title, text, QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok) != QMessageBox::Ok) {
        return;
    }

    if (!tuneTimer) {
        tuneTimer = new QTimer(this);
        tuneTimer->setSingleShot(true);
        connect(tuneTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onTuneTimer);
    }
    tuning = true;
    tuneCancelling = false;
    tuneWaitingFrame = false;
    tuneIndex = -1;
    tuneApertures.clear();
    tuneWindow.release();
    cameraWidget->lockScanActions(true);
    railBox->setEnabled(false);              // the dialog's arrows move the sliders now
    tuneDialog = new LAUCornerTuneDialog(this);
    connect(tuneDialog, &LAUCornerTuneDialog::emitNudge, this, &LAUGantryMainWindow::onTuneNudge);
    connect(tuneDialog, &LAUCornerTuneDialog::emitCentered, this, &LAUGantryMainWindow::onTuneCentered);
    connect(tuneDialog, &LAUCornerTuneDialog::emitSkip, this, &LAUGantryMainWindow::onTuneSkip);
    connect(tuneDialog, &LAUCornerTuneDialog::emitCancel, this, &LAUGantryMainWindow::onTuneCancel);
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitLiveFrame, tuneDialog, [this](cv::Mat raw16) { showTuneZoom(raw16); });
    tuneDialog->show();
    nextTuneCorner();
}

void LAUGantryMainWindow::nextTuneCorner()
{
    tuneIndex++;
    if (tuneIndex >= tuneCorners.count()) {
        finishTune();
        return;
    }
    goToTuneCorner();
}

void LAUGantryMainWindow::goToTuneCorner()
{
    TuneCorner &c = tuneCorners[tuneIndex];
    tuneDialog->setBusy(true);
    tuneDialog->setNudge(QString());
    c.single = c.nearSingle;
    c.pair = c.nearPair;
    c.moves = 0;
    c.insetTry = 0;
    if (holeView.isValid() && !c.insets.isEmpty()) {
        tuneStage = TuneToInset;
        tuneDialog->setMessage(QString("The page's %1 corner: finding the hole on plain paper next to it...").arg(c.name));
        railWidget->moveRails(c.insets[0].x, c.insets[0].y);
        return;
    }
    tuneStage = (holeView.isValid() && !tuneApertures.isEmpty()) ? TuneToCorner : TuneManual;
    tuneDialog->setMessage(QString("Moving the hole to the page's %1 corner...").arg(c.name));
    railWidget->moveRails(c.single, c.pair);  // answers with emitHomeFinished -> onTuneMoveFinished
}

void LAUGantryMainWindow::onTuneMoveFinished(bool ok, const QString &message)
{
    if (tuneCancelling) {
        endTune(QString("Fine-tuning the page corners was cancelled. Nothing was changed."));
        return;
    }
    if (!ok) {
        endTune(QString("Fine-tuning the page corners stopped: %1").arg(message));
        return;
    }
    TuneCorner &c = tuneCorners[tuneIndex];
    if (tuneStage == TuneToInset) {
        tuneTimer->start(SETTLE_MS);         // then a frame of the hole on plain paper
        return;
    }
    c.single = railWidget->reportedMM(0);
    c.pair = railWidget->reportedMM(2);
    if (tuneStage == TuneToCorner) {
        tuneTimer->start(SETTLE_MS);         // then a frame of the corner
        return;
    }

    int number = 0, total = 0;
    for (int k = 0; k < tuneCorners.count(); k++) {
        total++;
        number += (k <= tuneIndex);
    }
    tuneDialog->setBusy(false);
    tuneDialog->setMessage(QString("<b>Corner %1 of %2: the page's %3 corner.</b><br><br>%4In the live picture, the corner of the paper should be "
                                   "in the middle of the i1's hole. Use the arrows to move the paper until it is, then press <b>Centered</b>.")
                               .arg(number).arg(total).arg(c.name)
                               .arg(c.note.isEmpty() ? QString() : QString("The camera couldn't centre it: %1<br><br>").arg(c.note.toHtmlEscaped())));
    const double ds = c.single - c.startSingle, dp = c.pair - c.startPair;
    tuneDialog->setNudge((std::abs(ds) < 0.005 && std::abs(dp) < 0.005)
                             ? QString("Not nudged yet.")
                             : QString("Nudged so far:\nsingle rail %1 mm\nrail pair %2 mm").arg(ds, 0, 'f', 2).arg(dp, 0, 'f', 2));
}

// settled: ask for an averaged frame, or the camera has gone quiet
void LAUGantryMainWindow::onTuneTimer()
{
    if (!tuning) {
        return;
    }
    if (tuneWaitingFrame) {
        tuneWaitingFrame = false;
        tuneByHand(QString("the camera stopped delivering frames."));
        return;
    }
    tuneWaitingFrame = true;
    tuneTimer->start(FRAME_TIMEOUT_MS);
    cameraWidget->requestScanFrame();
}

void LAUGantryMainWindow::onTuneFrame(const cv::Mat &raw16)
{
    if (!tuning || !tuneWaitingFrame) {
        return;
    }
    tuneWaitingFrame = false;
    tuneTimer->stop();
    if (tuneCancelling) {
        endTune(QString("Fine-tuning the page corners was cancelled. Nothing was changed."));
        return;
    }
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);      // Mono10 -> 8 bits, as the scans are saved
    TuneCorner &c = tuneCorners[tuneIndex];
    const QString checkPath = QDir(tuneFolder).filePath(QString("corner_tune_%1%2.png").arg(c.name).arg(tuneStage == TuneToInset ? "_hole" : ""));

    if (tuneStage == TuneToInset) {
        const double pair = railWidget->reportedMM(2), single = railWidget->reportedMM(0);
        cv::Point2d aperture = holeView.calibratedAperture();
        if (!tuneApertures.isEmpty()) {
            aperture = cv::Point2d(0, 0);
            for (const cv::Point2d &a : tuneApertures) {
                aperture += a * (1.0 / tuneApertures.count());
            }
        }
        LAUHoleView::Hole hole;
        QString why;
        cv::Mat check;
        if (holeView.findHole(gray, pair, single, aperture + holeView.boardAxis(0) * pair + holeView.boardAxis(1) * single, &hole, &why, &check)) {
            tuneApertures << hole.aperture;
            tuneWindow = hole.window;
        } else {
            c.note = QString("the hole wasn't clear on the paper next to the corner (%1)").arg(why);
        }
        if (!check.empty()) {
            QImage(check.data, check.cols, check.rows, (int)check.step, QImage::Format_BGR888).save(checkPath);
        }
        if (!c.note.isEmpty() && ++c.insetTry < c.insets.count()) {
            c.note.clear();                  // print under the hole, most likely: try further in
            railWidget->moveRails(c.insets[c.insetTry].x, c.insets[c.insetTry].y);
            return;
        }
        if (tuneApertures.isEmpty()) {
            tuneStage = TuneManual;
        } else {
            tuneStage = TuneToCorner;
            tuneDialog->setMessage(QString("The page's %1 corner: centring it in the hole...").arg(c.name));
        }
        railWidget->moveRails(c.single, c.pair);
        return;
    }

    // over the corner: where the hole is now, from where it was seen, and where the corner is
    cv::Point2d aperture(0, 0);
    for (const cv::Point2d &a : tuneApertures) {
        aperture += a * (1.0 / tuneApertures.count());
    }
    const double pair = c.pair, single = c.single;   // reported where the move ended
    const cv::Point2d hole = aperture + holeView.boardAxis(0) * pair + holeView.boardAxis(1) * single;
    const int k = tuneIndex;
    const cv::Point2d along1 = tuneCorners[(k + 1) % 4].board - c.board, along2 = tuneCorners[(k + 3) % 4].board - c.board;
    LAUHoleView::Corner found;
    QString why;
    cv::Mat check;
    const bool ok = holeView.findCorner(gray, pair, single, hole, tuneWindow, c.board, along1, along2, &found, &why, &check);
    if (!check.empty()) {
        QImage(check.data, check.cols, check.rows, (int)check.step, QImage::Format_BGR888).save(checkPath);
    }
    if (!ok) {
        tuneByHand(why);
        return;
    }

    // the rails' move that puts the hole's centre on the corner: ex * dPair + ey * dSingle = miss
    const cv::Point2d miss = found.at - hole;
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const double det = ax.x * ay.y - ay.x * ax.y;
    const double dPair = (miss.x * ay.y - ay.x * miss.y) / det, dSingle = (ax.x * miss.y - miss.x * ax.y) / det;
    c.missMM = cv::norm(miss);
    double nextSingle = c.single + dSingle, nextPair = c.pair + dPair;
    clampToTravel(tuneGeometry, &nextSingle, &nextPair);
    const double step = cv::norm(ax * (nextPair - c.pair) + ay * (nextSingle - c.single));
    if (c.missMM >= TOLERANCE_MM && step < STUCK_MM) {
        // the sliders are as close as they go: measured, but out of reach
        c.single += dSingle;
        c.pair += dPair;
        c.automatic = true;
        c.confirmed = true;
        tuneOutOfReach(true, QString("the sliders stop %1 mm short of it").arg(c.missMM, 0, 'f', 1));
        return;
    }
    if (c.missMM < TOLERANCE_MM || c.moves >= MAX_MOVES) {
        // centred, or as near as the travel allows: the miss left is added to where the rails are rather than moved,
        // which may put a corner just past the travel beyond it -- it is only used to fit the correction
        c.single += dSingle;
        c.pair += dPair;
        c.automatic = true;
        c.confirmed = true;
        nextTuneCorner();
        return;
    }
    c.moves++;
    tuneDialog->setMessage(QString("The page's %1 corner: centring it in the hole. It was %2 mm off; move %3...")
                               .arg(c.name).arg(c.missMM, 0, 'f', 2).arg(c.moves));
    railWidget->moveRails(nextSingle, nextPair);
}

// The live picture around the hole, enlarged for the dialog.  The hole is fixed to the camera, so it is always in the
// same place in the frame; green marks where the i1 measures and its 9.8 mm hole, yellow where the corner should be.
void LAUGantryMainWindow::showTuneZoom(const cv::Mat &raw16)
{
    if (!tuning || !tuneDialog || raw16.empty() || tuneIndex < 0 || tuneIndex >= tuneCorners.count() || !singleRail.isKnown() || !pairRail.isKnown()) {
        return;
    }
    const double now = guideClock.elapsed() / 1000.0;
    const double pair = pairRail.estimate(now), single = singleRail.estimate(now);
    cv::Point2d aperture = holeView.isValid() ? holeView.calibratedAperture() : tuneGeometry.A0;
    if (!tuneApertures.isEmpty()) {
        aperture = cv::Point2d(0, 0);
        for (const cv::Point2d &a : tuneApertures) {
            aperture += a * (1.0 / tuneApertures.count());
        }
    }
    const cv::Point2d ax(tuneGeometry.ex[0], tuneGeometry.ex[1]), ay(tuneGeometry.ey[0], tuneGeometry.ey[1]);
    const cv::Point2d hole = aperture + ax * pair + ay * single;
    cv::Point2d centre, edge, corner;
    if (!platenOverlay.project(hole, pair, single, &centre) || !platenOverlay.project(hole + cv::Point2d(4.9, 0.0), pair, single, &edge)) {
        return;
    }
    const double pxPerMM = cv::norm(edge - centre) / 4.9;
    const int half = std::max(8, (int)std::lround(ZOOM_HALF_MM * pxPerMM));
    const cv::Rect box((int)std::lround(centre.x) - half, (int)std::lround(centre.y) - half, 2 * half + 1, 2 * half + 1);
    const cv::Rect inside = box & cv::Rect(0, 0, raw16.cols, raw16.rows);
    if (inside.area() <= 0) {
        return;
    }
    cv::Mat crop = cv::Mat::zeros(box.size(), CV_8U), grey;
    raw16(inside).convertTo(grey, CV_8U, 0.25);  // Mono10 -> 8 bits
    grey.copyTo(crop(inside - box.tl()));
    const int size = 420;
    const double scale = size / (double)box.width;
    cv::Mat big, colour;
    cv::resize(crop, big, cv::Size(size, size), 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(big, colour, cv::COLOR_GRAY2BGR);
    const auto at = [&](const cv::Point2d &pixel) {   // raw pixel -> zoom, 1/16 pixel
        return cv::Point((int)std::lround(((pixel.x - box.x + 0.5) * scale - 0.5) * 16.0), (int)std::lround(((pixel.y - box.y + 0.5) * scale - 0.5) * 16.0));
    };
    const cv::Scalar green(0, 220, 0), yellow(0, 230, 255);
    std::vector<cv::Point> ring;
    for (int n = 0; n <= 48; n++) {
        cv::Point2d p;
        if (platenOverlay.project(hole + cv::Point2d(std::cos(n * CV_PI / 24.0), std::sin(n * CV_PI / 24.0)) * 4.9, pair, single, &p)) {
            ring.push_back(at(p));
        }
    }
    cv::polylines(colour, ring, false, green, 1, cv::LINE_AA, 4);
    const cv::Point c = at(centre);
    const int arm = 14 * 16;
    cv::line(colour, c - cv::Point(arm, 0), c + cv::Point(arm, 0), green, 1, cv::LINE_AA, 4);
    cv::line(colour, c - cv::Point(0, arm), c + cv::Point(0, arm), green, 1, cv::LINE_AA, 4);
    if (platenOverlay.project(tuneCorners[tuneIndex].board, pair, single, &corner)) {
        const cv::Point k = at(corner);
        cv::line(colour, k - cv::Point(arm / 2, arm / 2), k + cv::Point(arm / 2, arm / 2), yellow, 2, cv::LINE_AA, 4);
        cv::line(colour, k - cv::Point(arm / 2, -arm / 2), k + cv::Point(arm / 2, -arm / 2), yellow, 2, cv::LINE_AA, 4);
    }
    cv::putText(colour, QString("%1 corner").arg(tuneCorners[tuneIndex].name).toStdString(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(colour, QString("%1 mm across").arg(2 * ZOOM_HALF_MM, 0, 'f', 0).toStdString(), cv::Point(10, size - 12), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    tuneDialog->setZoom(QImage(colour.data, colour.cols, colour.rows, (int)colour.step, QImage::Format_BGR888).copy());
}

// the camera couldn't: the dialog's arrows for this corner
void LAUGantryMainWindow::tuneByHand(const QString &reason)
{
    TuneCorner &c = tuneCorners[tuneIndex];
    if (!c.reachable && tuneStage == TuneToCorner) {
        tuneOutOfReach(false, reason);       // as close as the sliders go, and the camera couldn't see it there
        return;
    }
    c.note = reason;
    const bool overCorner = (tuneStage != TuneToInset);
    tuneStage = TuneManual;
    if (overCorner) {
        onTuneMoveFinished(true, QString()); // shows the corner's message with the arrows enabled
    } else {
        railWidget->moveRails(c.single, c.pair);   // still beside it, over plain paper
    }
}

// The corner can't be brought under the hole: say so, and go on to the next corner unless the user would rather stop.
// measured: the camera saw it from as close as the sliders go, so it still counts in the fit.
void LAUGantryMainWindow::tuneOutOfReach(bool measured, const QString &why)
{
    TuneCorner &c = tuneCorners[tuneIndex];
    c.note = QString("out of reach: %1").arg(why);
    const bool last = (tuneIndex + 1 >= tuneCorners.count());
    QString text = QString("The page's %1 corner is out of reach: %2.").arg(c.name, why);
    text += measured ? QString("\n\nThe camera still saw it through the hole, so it counts toward the correction.")
                     : QString("\n\nThe camera couldn't see it through the hole from there, so it is left out.");
    QMessageBox box(QMessageBox::Information, QString("Fine-tune Page Corners"), text, QMessageBox::NoButton, tuneDialog);
    QPushButton *next = box.addButton(last ? QString("Finish") : QString("Next corner"), QMessageBox::AcceptRole);
    box.addButton(QMessageBox::Cancel);
    box.setDefaultButton(next);
    box.exec();
    if (!tuning) {
        return;
    }
    if (box.clickedButton() != next) {
        endTune(QString("Fine-tuning the page corners was cancelled. Nothing was changed."));
        return;
    }
    if (!measured) {
        c.confirmed = false;
    }
    nextTuneCorner();
}

// Each arrow moves the paper that way in the live picture by the chosen step, measured on the paper
void LAUGantryMainWindow::onTuneNudge(int right, int down)
{
    if (!tuning || tuneStage != TuneManual || railWidget->isHoming()) {
        return;
    }
    TuneCorner &c = tuneCorners[tuneIndex];
    const double step = tuneDialog->stepMM();
    double ds = right * step, dp = down * step;   // without the camera's model: straight along the rails

    // how the corner moves in the picture per mm of each rail, from the camera's model
    cv::Point2d p0, pS, pP;
    if (platenOverlay.project(c.board, c.pair, c.single, &p0) && platenOverlay.project(c.board, c.pair, c.single + 1.0, &pS) &&
        platenOverlay.project(c.board, c.pair + 1.0, c.single, &pP)) {
        const cv::Point2d js = pS - p0, jp = pP - p0;
        const double det = js.x * jp.y - jp.x * js.y;
        if (std::abs(det) > 1e-9) {
            // js * s + jp * p = (right, down), then scaled to move the paper by the step
            const double s = (right * jp.y - jp.x * down) / det, p = (js.x * down - right * js.y) / det;
            const cv::Point2d onBoard = cv::Point2d(tuneGeometry.ex[0], tuneGeometry.ex[1]) * p + cv::Point2d(tuneGeometry.ey[0], tuneGeometry.ey[1]) * s;
            const double scale = step / std::max(cv::norm(onBoard), 1e-9);
            ds = s * scale;
            dp = p * scale;
        }
    }
    if (!tuneGeometry.reachable(c.single + ds, c.pair + dp)) {
        tuneDialog->setNudge(QString("That would take the sliders past the i1's travel."));
        return;
    }
    c.single += ds;
    c.pair += dp;
    tuneDialog->setBusy(true);
    railWidget->moveRails(c.single, c.pair);
}

void LAUGantryMainWindow::onTuneCentered()
{
    if (!tuning || tuneStage != TuneManual || railWidget->isHoming()) {
        return;
    }
    tuneCorners[tuneIndex].confirmed = true;
    tuneCorners[tuneIndex].automatic = false;
    nextTuneCorner();
}

void LAUGantryMainWindow::onTuneSkip()
{
    if (!tuning || tuneStage != TuneManual || railWidget->isHoming()) {
        return;
    }
    tuneCorners[tuneIndex].confirmed = false;
    nextTuneCorner();
}

void LAUGantryMainWindow::onTuneCancel()
{
    if (!tuning) {
        return;
    }
    if (railWidget->isHoming()) {
        tuneCancelling = true;
        tuneDialog->setMessage(QString("Stopping the sliders..."));
        railWidget->abortHome();             // onTuneMoveFinished ends it once they have stopped
        return;
    }
    if (tuneWaitingFrame) {
        tuneCancelling = true;               // onTuneFrame ends it when the frame comes in
        return;
    }
    endTune(QString("Fine-tuning the page corners was cancelled. Nothing was changed."));
}

// The correction: from the model's rail positions to the centred ones.  Four or three corners give an affine map
// (offset, scale along each rail, skew), two a similarity (offset, turn, scale), one an offset.
void LAUGantryMainWindow::finishTune()
{
    tuneDialog->hide();
    QList<TuneCorner> used;
    for (const TuneCorner &c : tuneCorners) {
        if (c.confirmed) {
            used << c;
        }
    }
    if (used.isEmpty()) {
        endTune(QString("No corner was centred, so nothing was changed."));
        return;
    }

    cv::Matx23d F(1, 0, 0, 0, 1, 0);
    if (used.count() >= 3) {
        cv::Mat A((int)used.count(), 3, CV_64F), bs((int)used.count(), 1, CV_64F), bp((int)used.count(), 1, CV_64F);
        for (int k = 0; k < used.count(); k++) {
            A.at<double>(k, 0) = used[k].plannedSingle;
            A.at<double>(k, 1) = used[k].plannedPair;
            A.at<double>(k, 2) = 1.0;
            bs.at<double>(k) = used[k].single;
            bp.at<double>(k) = used[k].pair;
        }
        cv::Mat fs, fp;
        cv::solve(A, bs, fs, cv::DECOMP_SVD);
        cv::solve(A, bp, fp, cv::DECOMP_SVD);
        F = cv::Matx23d(fs.at<double>(0), fs.at<double>(1), fs.at<double>(2), fp.at<double>(0), fp.at<double>(1), fp.at<double>(2));
    } else if (used.count() == 2) {
        using C = std::complex<double>;
        const C z0(used[0].plannedSingle, used[0].plannedPair), z1(used[1].plannedSingle, used[1].plannedPair);
        const C w0(used[0].single, used[0].pair), w1(used[1].single, used[1].pair);
        const C a = (std::abs(z1 - z0) > 1e-9) ? (w1 - w0) / (z1 - z0) : C(1.0, 0.0), b = w0 - a * z0;
        F = cv::Matx23d(a.real(), -a.imag(), b.real(), a.imag(), a.real(), b.imag());
    } else {
        F = cv::Matx23d(1, 0, used[0].single - used[0].plannedSingle, 0, 1, used[0].pair - used[0].plannedPair);
    }

    // the summary: each corner's nudge and what the fit leaves, in mm on the paper
    const auto onPaper = [this](double ds, double dp) {
        return cv::norm(cv::Point2d(tuneGeometry.ex[0], tuneGeometry.ex[1]) * dp + cv::Point2d(tuneGeometry.ey[0], tuneGeometry.ey[1]) * ds);
    };
    QString rows;
    QJsonArray json;
    double worstNudge = 0.0, worstLeft = 0.0;
    for (const TuneCorner &c : tuneCorners) {
        QJsonObject o;
        o["corner"] = c.name;
        o["board_mm"] = QJsonArray{ c.board.x, c.board.y };
        o["model_single_pair_mm"] = QJsonArray{ c.plannedSingle, c.plannedPair };
        o["started_single_pair_mm"] = QJsonArray{ c.startSingle, c.startPair };
        o["reachable"] = c.reachable;
        o["confirmed"] = c.confirmed;
        if (!c.note.isEmpty()) {
            o["camera_note"] = c.note;
        }
        if (!c.confirmed) {
            rows += QString("<tr><td>%1</td><td colspan=3>%2</td></tr>").arg(c.name, c.reachable ? QString("skipped") : QString("out of reach"));
            json.append(o);
            continue;
        }
        const double nudge = onPaper(c.single - c.startSingle, c.pair - c.startPair);
        const double fs = F(0, 0) * c.plannedSingle + F(0, 1) * c.plannedPair + F(0, 2);
        const double fp = F(1, 0) * c.plannedSingle + F(1, 1) * c.plannedPair + F(1, 2);
        const double left = onPaper(c.single - fs, c.pair - fp);
        worstNudge = std::max(worstNudge, nudge);
        worstLeft = std::max(worstLeft, left);
        o["centred_single_pair_mm"] = QJsonArray{ c.single, c.pair };
        o["centred_by"] = c.automatic ? QString("camera") : QString("hand");
        if (c.automatic) {
            o["camera_moves"] = c.moves;
            o["camera_last_miss_mm"] = c.missMM;
        }
        o["nudge_mm"] = nudge;
        o["left_after_fit_mm"] = left;
        json.append(o);
        rows += QString("<tr><td>%1</td><td>%2</td><td align=right>%3 mm</td><td align=right>%4 mm</td></tr>")
                    .arg(c.name, c.automatic ? (c.reachable ? QString("camera") : QString("camera, past the travel")) : QString("by hand"))
                    .arg(nudge, 0, 'f', 2).arg(left, 0, 'f', 2);
    }
    const QString scanName = QFileInfo(QFileInfo(tuneFolder).absolutePath()).fileName();
    QString text = QString("<p>%1 of %2 corners centred.</p><table cellspacing=6><tr><th align=left>Corner</th><th align=left>Centred</th>"
                           "<th>Moved by</th><th>Left after the fit</th></tr>%3</table>")
                       .arg(used.count()).arg(tuneCorners.count()).arg(rows);
    QJsonArray apertures;
    if (!tuneApertures.isEmpty()) {
        cv::Point2d mean(0, 0);
        for (const cv::Point2d &a : tuneApertures) {
            mean += a * (1.0 / tuneApertures.count());
            apertures.append(QJsonArray{ a.x, a.y });
        }
        double spread = 0.0;
        for (const cv::Point2d &a : tuneApertures) {
            spread = std::max(spread, cv::norm(a - mean));
        }
        text += QString("<p>The hole was seen %1 time%2, within %3 mm, and sits %4 mm from where the platen calibration has it.</p>")
                    .arg(tuneApertures.count()).arg(tuneApertures.count() > 1 ? "s" : "").arg(spread, 0, 'f', 2)
                    .arg(cv::norm(mean - holeView.calibratedAperture()), 0, 'f', 2);
    }
    if (worstNudge > LARGE_NUDGE_MM) {
        text += QString("<p><b>One corner needed more than %1 mm.</b> Check that the page's own corner was centred, not another mark.</p>").arg(LARGE_NUDGE_MM, 0, 'f', 0);
    }
    text += QString("<p>The camera's views of each corner are saved as corner_tune_*.png in %1.</p>").arg(QDir::toNativeSeparators(tuneFolder).toHtmlEscaped());
    text += QString("<p>Save this correction with scan %1? Points taken from its picture will then use it.</p>").arg(scanName);

    QMessageBox box(QMessageBox::Question, QString("Fine-tune Page Corners"), text, QMessageBox::NoButton, this);
    box.setTextFormat(Qt::RichText);
    QPushButton *save = box.addButton(QString("Save"), QMessageBox::AcceptRole);
    box.addButton(QString("Discard"), QMessageBox::RejectRole);
    box.setDefaultButton(save);
    box.exec();
    if (box.clickedButton() != save) {
        endTune(QString("The page-corner correction was discarded. Nothing was changed."));
        return;
    }

    tuneGeometry.railsFix = F;
    double pageW = 0.0, pageH = 0.0;
    const bool mapped = savePageMap(F, &pageW, &pageH);
    tuneGeometry.singleLow = tuneSavedTravel[0];
    tuneGeometry.singleHigh = tuneSavedTravel[1];
    tuneGeometry.pairLow = tuneSavedTravel[2];
    tuneGeometry.pairHigh = tuneSavedTravel[3];
    QJsonObject record;
    record["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    record["corners"] = json;
    record["hole_seen_at_rails_zero_mm"] = apertures;
    record["calibrated_hole_at_rails_zero_mm"] = QJsonArray{ holeView.calibratedAperture().x, holeView.calibratedAperture().y };
    record["fix_single_pair"] = QJsonArray{ QJsonArray{ F(0, 0), F(0, 1), F(0, 2) }, QJsonArray{ F(1, 0), F(1, 1), F(1, 2) } };
    record["fit"] = (used.count() >= 3) ? QString("affine") : (used.count() == 2) ? QString("similarity") : QString("offset");
    record["worst_nudge_mm"] = worstNudge;
    record["worst_left_after_fit_mm"] = worstLeft;
    QFile out(QDir(tuneFolder).filePath(QString("corner_tune.json")));
    const bool saved = tuneGeometry.save(tuneFolder) && out.open(QIODevice::WriteOnly | QIODevice::Truncate) && out.write(QJsonDocument(record).toJson()) > 0;
    endTune((saved && mapped) ? QString("Saved the page-corner correction with %1: corners moved by up to %2 mm, %3 mm left after the fit. "
                                        "Go to page position now puts the hole anywhere on the %4 x %5 mm page.")
                        .arg(QDir::toNativeSeparators(tuneFolder)).arg(worstNudge, 0, 'f', 2).arg(worstLeft, 0, 'f', 2).arg(pageW, 0, 'f', 1).arg(pageH, 0, 'f', 1)
                  : QString("Couldn't save the page-corner correction in %1").arg(QDir::toNativeSeparators(tuneFolder)));
}

// The page's own coordinates to slider positions: mm from the page's top-left corner as the merged picture shows it,
// X toward its top-right corner and Y toward its bottom-left.  A perspective map through the four corners' slider
// positions -- where the camera centred them, or for a corner it didn't, the corrected model's -- so it is exact at
// every centred corner and carries the correction smoothly across the page.  Saved as page_sliders.json.
bool LAUGantryMainWindow::savePageMap(const cv::Matx23d &fix, double *widthMM, double *heightMM)
{
    std::vector<cv::Point2d> B;
    for (const TuneCorner &c : tuneCorners) {
        B.push_back(c.board);
    }
    *widthMM = 0.5 * (cv::norm(B[1] - B[0]) + cv::norm(B[2] - B[3]));
    *heightMM = 0.5 * (cv::norm(B[3] - B[0]) + cv::norm(B[2] - B[1]));
    const std::vector<cv::Point2f> page{ { 0.0f, 0.0f }, { (float)*widthMM, 0.0f }, { (float)*widthMM, (float)*heightMM }, { 0.0f, (float)*heightMM } };
    std::vector<cv::Point2f> rails;
    QJsonArray corners;
    for (int k = 0; k < 4; k++) {
        const TuneCorner &c = tuneCorners[k];
        double single = c.single, pair = c.pair;
        if (!c.confirmed) {
            single = fix(0, 0) * c.plannedSingle + fix(0, 1) * c.plannedPair + fix(0, 2);
            pair = fix(1, 0) * c.plannedSingle + fix(1, 1) * c.plannedPair + fix(1, 2);
        }
        rails.emplace_back((float)single, (float)pair);
        corners.append(QJsonObject{ { "corner", c.name }, { "page_mm", QJsonArray{ page[k].x, page[k].y } },
                                    { "single_pair_mm", QJsonArray{ single, pair } }, { "centred", c.confirmed } });
    }
    const cv::Mat H = cv::getPerspectiveTransform(page, rails);
    QJsonArray h;
    for (int r = 0; r < 3; r++) {
        h.append(QJsonArray{ H.at<double>(r, 0), H.at<double>(r, 1), H.at<double>(r, 2) });
    }
    QJsonObject o;
    o["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    o["page_size_mm"] = QJsonArray{ *widthMM, *heightMM };
    o["page_axes"] = QString("mm from the page's top-left corner as the merged picture shows it; X toward its top-right corner, Y toward its bottom-left");
    o["page_to_single_pair"] = h;            // (single, pair, w) = H (x, y, 1)
    o["corners"] = corners;
    QFile file(QDir(tuneFolder).filePath(QString("page_sliders.json")));
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) && file.write(QJsonDocument(o).toJson()) > 0;
}

// Go to page position: the hole over a point given in the page's own mm, from the newest page_sliders.json
void LAUGantryMainWindow::onPagePositionRequested()
{
    const QString title = QString("Go to Page Position");
    if (scanning || measuring || merging || returningHome || tuning || pageMoving || railWidget->isHoming()) {
        cameraWidget->showMessage(QString("Wait until the sliders and the merge are finished, then try again."));
        return;
    }
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        QMessageBox::information(this, title, QString("Tick Enable Sliders first, so the sliders can move."));
        return;
    }
    QDir base(scanBase);
    QString folder;
    for (const QString &name : base.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        if (QFile::exists(base.filePath(name + QString("/mosaic/page_sliders.json")))) {
            folder = base.filePath(name + QString("/mosaic"));
            break;
        }
    }
    if (folder.isEmpty()) {
        QMessageBox::information(this, title, QString("No page has been fine-tuned yet. Scan the platen image, then choose Fine-tune page corners."));
        return;
    }
    QFile file(QDir(folder).filePath(QString("page_sliders.json")));
    const QJsonObject o = file.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject();
    const QJsonArray h = o["page_to_single_pair"].toArray(), size = o["page_size_mm"].toArray();
    if (h.count() != 3 || size.count() != 2) {
        QMessageBox::warning(this, title, QString("%1 couldn't be read. Fine-tune the page corners again.").arg(QDir::toNativeSeparators(file.fileName())));
        return;
    }
    cv::Matx33d H;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            H(r, c) = h[r].toArray().at(c).toDouble();
        }
    }
    const double width = size[0].toDouble(), height = size[1].toDouble();
    const QString scanName = QFileInfo(QFileInfo(folder).absolutePath()).fileName();

    QDialog dialog(this);
    dialog.setWindowTitle(title);
    QVBoxLayout *layout = new QVBoxLayout(&dialog);
    QLabel *label = new QLabel(QString("The %1 x %2 mm page fine-tuned in scan %3.\nX is from its left edge and Y from its top edge, "
                                       "as the merged picture shows the page.").arg(width, 0, 'f', 1).arg(height, 0, 'f', 1).arg(scanName));
    label->setWordWrap(true);
    layout->addWidget(label);
    QFormLayout *form = new QFormLayout();
    QSettings settings;
    QDoubleSpinBox *x = new QDoubleSpinBox(), *y = new QDoubleSpinBox();
    for (QDoubleSpinBox *box : { x, y }) {
        box->setDecimals(2);
        box->setSuffix(QString(" mm"));
        box->setSingleStep(1.0);
    }
    x->setRange(0.0, width);
    y->setRange(0.0, height);
    x->setValue(settings.value(QString("LAUGantryMainWindow::pageX"), width / 2.0).toDouble());
    y->setValue(settings.value(QString("LAUGantryMainWindow::pageY"), height / 2.0).toDouble());
    form->addRow(QString("X"), x);
    form->addRow(QString("Y"), y);
    layout->addLayout(form);
    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(QString("Go"));
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    settings.setValue(QString("LAUGantryMainWindow::pageX"), x->value());
    settings.setValue(QString("LAUGantryMainWindow::pageY"), y->value());

    const cv::Vec3d r = H * cv::Vec3d(x->value(), y->value(), 1.0);
    const double single = r[0] / r[2], pair = r[1] / r[2];
    double sLow = 0.0, sHigh = 0.0, pLow = 0.0, pHigh = 0.0;
    if (!railWidget->limitRange(0, &sLow, &sHigh) || !railWidget->limitRange(2, &pLow, &pHigh) ||
        single < sLow || single > sHigh || pair < pLow || pair > pHigh) {
        QMessageBox::information(this, title, QString("(%1, %2) mm on the page needs the sliders at X %3 mm, Y %4 mm, outside their limit sliders.")
                                                  .arg(x->value(), 0, 'f', 2).arg(y->value(), 0, 'f', 2).arg(single, 0, 'f', 2).arg(pair, 0, 'f', 2));
        return;
    }
    pageMoveMessage = QString("The i1's hole is over (%1, %2) mm on the page: sliders at X %3 mm, Y %4 mm.")
                          .arg(x->value(), 0, 'f', 2).arg(y->value(), 0, 'f', 2).arg(single, 0, 'f', 2).arg(pair, 0, 'f', 2);
    pageMoving = true;
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(title);
    homeDialog->setMessage(QString("Moving the i1's hole to (%1, %2) mm on the page...").arg(x->value(), 0, 'f', 2).arg(y->value(), 0, 'f', 2));
    connect(homeDialog, &LAUHomeDialog::emitAbort, railWidget, &LAUMultiVelmexWidget::abortHome);
    homeDialog->open();
    railWidget->moveRails(single, pair);     // answers with emitHomeFinished
}

void LAUGantryMainWindow::endTune(const QString &message)
{
    tuning = false;
    tuneCancelling = false;
    tuneWaitingFrame = false;
    if (tuneTimer) {
        tuneTimer->stop();
    }
    if (tuneDialog) {
        tuneDialog->deleteLater();
        tuneDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->lockScanActions(false);
    cameraWidget->showMessage(message);
}
