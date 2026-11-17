/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Measure slop by tracking the hole.  The camera rides on the carriage and the i1 holder swings a little on it, so
// where the hole lands in the camera's view after a move is the slop itself: no page edge or corner is needed, only
// plain paper under the hole, where the hole is a bright disc (LAUHoleView::findHole) whose centre, less where the
// sliders are, is the hole's place relative to the carriage.
//
//   Spot:     a patch of plain paper in from a page corner (12, 20 or 30 mm, as the corner fine-tune uses), with room
//             to move 1" each way in X.
//   Learn:    move and return: 1" right and back to the spot (arriving moving left), 1" left and back (arriving
//             moving right), the hole found after each return, until the shift between the two is pinned down.
//   Predict:  keep moving and returning, but add to each return the X correction that should put the hole at the
//             midpoint of the two, and see where it lands; each landing refines that side's estimate, until both
//             sides land within tolerance a few times in a row.  Abort stops it and keeps what was measured.
//
// The close-up tracks the hole live, a few times a second, with a bullseye and a trail.
//
#include "laugantrymainwindow.h"
#include "lauimageinspectdialog.h"

#include <cmath>
#include <algorithm>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QTimer>
#include <QDialog>
#include <QDateTime>
#include <QSettings>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QMessageBox>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QTextStream>
#include <QJsonDocument>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <opencv2/imgproc.hpp>

namespace
{
    const int SETTLE_MS = 500;
    const int FRAME_TIMEOUT_MS = 15000;
    const int MIN_LEARN = 3;                 // returns from each side before the slop is trusted
    const int MAX_LEARN = 12;                // and at most, before predicting anyway
    const int TRACK_EVERY_MS = 200;          // live tracking, a few times a second
    const int TRAIL = 150;
    const double INSETS_MM[] = { 12.0, 20.0, 30.0 };

    cv::Point2d unitOf(const cv::Point2d &v)
    {
        return v * (1.0 / std::max(cv::norm(v), 1e-12));
    }

    int sideIndex(int side)                  // side +1 comes from the right, moving left: index 0
    {
        return side > 0 ? 0 : 1;
    }

    QString sideName(int side)
    {
        return side > 0 ? QString("from the right, moving left") : QString("from the left, moving right");
    }
}

void LAUGantryMainWindow::onEdgeSlopRequested()
{
    const QString title = QString("Measure Slop by Tracking the Hole");
    if (scanning || measuring || merging || returningHome || tuning || edgeSloping || pageMoving || railWidget->isHoming()) {
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
        if (QFile::exists(base.filePath(name + QString("/mosaic/page.json")))) {
            folder = base.filePath(name + QString("/mosaic"));
            break;
        }
    }
    if (folder.isEmpty()) {
        QMessageBox::information(this, title, QString("No page has been found on the platen yet. Put the page on the platen and choose Scan platen image first."));
        return;
    }
    edgeScan = QFileInfo(QFileInfo(folder).absolutePath()).fileName();
    QFile file(QDir(folder).filePath(QString("page.json")));
    const QJsonArray corners = (file.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject())["corners_board_mm"].toArray();
    QString error;
    if (corners.count() != 4 || !slopGeometry.load(folder, &error) || !slopGeometry.hasRails()) {
        QMessageBox::warning(this, title, error.isEmpty() ? QString("The page or the hole's position in scan %1 couldn't be read.").arg(edgeScan) : error);
        return;
    }
    if (!holeView.load(QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString())) {
        QMessageBox::warning(this, title, QString("The platen calibration can't be read, so the camera can't find the hole. Run Calibrate platen first."));
        return;
    }
    double low = 0.0, high = 0.0;
    if (railWidget->limitRange(0, &low, &high)) {
        slopGeometry.singleLow = low;
        slopGeometry.singleHigh = high;
    }
    if (railWidget->limitRange(2, &low, &high)) {
        slopGeometry.pairLow = low;
        slopGeometry.pairHigh = high;
    }

    QSettings settings;
    QDialog dialog(this);
    dialog.setWindowTitle(title);
    QVBoxLayout *layout = new QVBoxLayout(&dialog);
    QLabel *label = new QLabel(QString("The camera rides on the carriage, so where the i1's hole lands in its view after a move is the slop. Over a patch "
                                       "of plain paper near a page corner, the sliders move 1\" right and back, then 1\" left and back, and the camera "
                                       "finds the hole after each return. Once the shift between the two directions is known, each return is corrected "
                                       "for it, and refined, until the hole lands in the same place from both sides. The close-up tracks the hole live.\n\n"
                                       "Take the i1 Pro out of its holder first so the camera sees the paper through the hole, and don't move the page."));
    label->setWordWrap(true);
    layout->addWidget(label);
    QFormLayout *form = new QFormLayout();
    QDoubleSpinBox *away = new QDoubleSpinBox(), *tolerance = new QDoubleSpinBox(), *inRow = new QDoubleSpinBox();
    away->setRange(2.0, 60.0);
    away->setSuffix(QString(" mm"));
    away->setValue(settings.value(QString("LAUGantryMainWindow::edgeAway"), 25.4).toDouble());
    tolerance->setRange(0.005, 0.5);
    tolerance->setDecimals(3);
    tolerance->setSingleStep(0.005);
    tolerance->setSuffix(QString(" mm"));
    tolerance->setValue(settings.value(QString("LAUGantryMainWindow::edgeTolerance"), 0.02).toDouble());
    inRow->setRange(1, 20);
    inRow->setDecimals(0);
    inRow->setValue(settings.value(QString("LAUGantryMainWindow::edgeHitsNeeded"), 3).toInt());
    form->addRow(QString("Move each way"), away);
    form->addRow(QString("Lands still within"), tolerance);
    form->addRow(QString("Done after this many in a row, from each side"), inRow);
    layout->addLayout(form);
    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(QString("Start"));
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    settings.setValue(QString("LAUGantryMainWindow::edgeAway"), away->value());
    settings.setValue(QString("LAUGantryMainWindow::edgeTolerance"), tolerance->value());
    settings.setValue(QString("LAUGantryMainWindow::edgeHitsNeeded"), (int)inRow->value());
    edgeAway = away->value();
    edgeTolerance = tolerance->value();
    edgeTries = (int)inRow->value();

    // plain-paper spots: in from each corner, with room to move each way in X
    std::vector<cv::Point2d> P;
    for (int k = 0; k < 4; k++) {
        P.emplace_back(corners[k].toArray().at(0).toDouble(), corners[k].toArray().at(1).toDouble());
    }
    edgeSpots.clear();
    for (double inset : INSETS_MM) {
        for (int k = 0; k < 4; k++) {
            const cv::Point2d at = P[k] + unitOf(P[(k + 1) % 4] - P[k]) * inset + unitOf(P[(k + 3) % 4] - P[k]) * inset;
            double x = 0.0, y = 0.0;
            slopGeometry.railsFor(at, &x, &y, true);
            if (slopGeometry.reachable(x - edgeAway - 1.0, y) && slopGeometry.reachable(x + edgeAway + 1.0, y)) {
                edgeSpots << cv::Point2d(x, y);
            }
        }
    }
    if (edgeSpots.isEmpty()) {
        QMessageBox::warning(this, title, QString("No spot near a page corner leaves room to move %1 mm each way in X with the limit sliders where they are. "
                                                  "Move less, or widen the limit sliders.").arg(edgeAway, 0, 'f', 1));
        return;
    }

    slopAperture = holeView.calibratedAperture();
    QFile tuned(QDir(folder).filePath(QString("corner_tune.json")));
    if (tuned.open(QIODevice::ReadOnly)) {
        const QJsonArray seen = QJsonDocument::fromJson(tuned.readAll()).object()["hole_seen_at_rails_zero_mm"].toArray();
        if (!seen.isEmpty()) {
            slopAperture = cv::Point2d(0, 0);
            for (const QJsonValue &a : seen) {
                slopAperture += cv::Point2d(a.toArray().at(0).toDouble(), a.toArray().at(1).toDouble()) * (1.0 / seen.count());
            }
        }
    }
    edgeFolder = QDir(scanBase).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) + QString("_holeslop"));
    QDir().mkpath(edgeFolder);

    if (!edgeTimer) {
        edgeTimer = new QTimer(this);
        edgeTimer->setSingleShot(true);
        connect(edgeTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onEdgeTimer);
    }
    edgeSloping = true;
    edgeCancelling = false;
    edgeWaitingFrame = false;
    edgeWasAborted = false;
    edgeLearning = true;
    edgeFailures = 0;
    edgeApertures[0].clear();
    edgeApertures[1].clear();
    edgeHits[0] = edgeHits[1] = 0;
    edgeStops.clear();
    edgeTrail.clear();
    edgeTracked = false;
    edgeTrackedAt = 0;
    edgeReferenceSet = false;
    edgeEllipseValid = false;
    edgeSpotTry = 0;
    edgeSide = +1;
    slopSeen = LAUHoleView::Corner();
    slopSeenValid = false;
    slopSeenHole = false;
    slopSeenWhy.clear();
    slopZoomTitle = QString("tracking the hole");
    slopZoomNote.clear();
    cameraWidget->lockScanActions(true);
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(title);
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, [this]() {
        edgeCancelling = true;
        if (railWidget->isHoming()) {
            railWidget->abortHome();         // onEdgeMoveFinished ends it once they have stopped
        } else if (!edgeWaitingFrame && !edgeTimer->isActive()) {
            edgeAborted();
        }
    });
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitLiveFrame, homeDialog, [this](cv::Mat raw16) {
        trackHoleLive(raw16);
        showSlopZoom(raw16);
    });
    homeDialog->open();
    edgeY = edgeSpots[0].y;
    edgeMoveTo(edgeSpots[0].x, EdgeSpot, QString("Looking for plain paper to see the hole on (spot 1 of %1)...").arg(edgeSpots.count()));
}

// the hole relative to the carriage, arriving from side i, on average
cv::Point2d LAUGantryMainWindow::edgeMean(int i) const
{
    cv::Point2d mean(0, 0);
    for (const cv::Point2d &a : edgeApertures[i]) {
        mean += a * (1.0 / edgeApertures[i].count());
    }
    return mean;
}

// the X, slider mm, to add arriving from this side so the hole lands at the midpoint of the two sides
double LAUGantryMainWindow::edgeCorrection(int side) const
{
    if (edgeApertures[0].isEmpty() || edgeApertures[1].isEmpty()) {
        return 0.0;
    }
    const cv::Point2d ay = holeView.boardAxis(1);
    const cv::Point2d middle = (edgeMean(0) + edgeMean(1)) * 0.5;
    return (middle - edgeMean(sideIndex(side))).dot(ay) / ay.dot(ay);
}

void LAUGantryMainWindow::edgeMoveTo(double x, EdgeStage stage, const QString &message)
{
    edgeStage = stage;
    edgeX = x;
    homeDialog->setMessage(message);
    railWidget->moveRails(x, edgeY);         // answers with emitHomeFinished -> onEdgeMoveFinished
}

// out 1" on this side, then back
void LAUGantryMainWindow::edgeNextReturn()
{
    const cv::Point2d ay = holeView.boardAxis(1);
    const double slop = (edgeApertures[0].isEmpty() || edgeApertures[1].isEmpty()) ? 0.0 : (edgeMean(1) - edgeMean(0)).dot(ay) / ay.dot(ay);
    QString message = edgeLearning ? QString("Learning: out %1 mm %2, then back to the spot.\nReturns so far: %3 moving left, %4 moving right.")
                                         .arg(edgeAway, 0, 'f', 1).arg(edgeSide > 0 ? QString("right") : QString("left"))
                                         .arg(edgeApertures[0].count()).arg(edgeApertures[1].count())
                                   : QString("Predicting: out %1 mm %2, then back with %3 mm added for the slop.\n"
                                             "Landed still in a row: %4 of %5 moving left, %6 of %5 moving right.")
                                         .arg(edgeAway, 0, 'f', 1).arg(edgeSide > 0 ? QString("right") : QString("left"))
                                         .arg(edgeCorrection(edgeSide), 0, 'f', 3).arg(edgeHits[0]).arg(edgeTries).arg(edgeHits[1]);
    if (!edgeApertures[0].isEmpty() && !edgeApertures[1].isEmpty()) {
        message += QString("\nX slop so far: %1 mm.").arg(slop, 0, 'f', 3);
    }
    edgeMoveTo(edgeSpotX + edgeSide * edgeAway, EdgeOut, message);
}

void LAUGantryMainWindow::onEdgeMoveFinished(bool ok, const QString &message)
{
    if (edgeCancelling) {
        edgeAborted();
        return;
    }
    if (!ok) {
        endEdgeSlop(QString("Measuring the slop stopped: %1").arg(message));
        return;
    }
    if (edgeStage == EdgeOut) {
        const double back = edgeSpotX + (edgeLearning ? 0.0 : edgeCorrection(edgeSide));
        edgeMoveTo(back, EdgeBack, QString("Back to the spot %1...").arg(sideName(edgeSide)));
        return;
    }
    edgeTimer->start(SETTLE_MS);             // then a frame of the hole
}

void LAUGantryMainWindow::onEdgeTimer()
{
    if (!edgeSloping) {
        return;
    }
    if (edgeCancelling) {
        edgeAborted();
        return;
    }
    if (edgeWaitingFrame) {
        edgeWaitingFrame = false;
        endEdgeSlop(QString("Measuring the slop stopped: the camera stopped delivering frames."));
        return;
    }
    edgeWaitingFrame = true;
    edgeTimer->start(FRAME_TIMEOUT_MS);
    cameraWidget->requestScanFrame();
}

void LAUGantryMainWindow::onEdgeFrame(const cv::Mat &raw16)
{
    if (!edgeSloping || !edgeWaitingFrame) {
        return;
    }
    edgeWaitingFrame = false;
    edgeTimer->stop();
    if (edgeCancelling) {
        edgeAborted();
        return;
    }
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);      // Mono10 -> 8 bits, as the scans are saved
    const double pair = railWidget->reportedMM(2), single = railWidget->reportedMM(0);
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const cv::Point2d guess = (edgeTracked ? edgeTrackedAperture : slopAperture) + ax * pair + ay * single;
    LAUHoleView::Hole hole;
    QString why;
    cv::Mat check;
    const bool found = holeView.findHole(gray, pair, single, guess, &hole, &why, &check);
    const int stop = edgeStops.count() + 1;
    if (!check.empty()) {
        QImage(check.data, check.cols, check.rows, (int)check.step, QImage::Format_BGR888)
            .save(QDir(edgeFolder).filePath(QString("stop_%1.png").arg(stop, 3, 10, QChar('0'))));
    }
    if (found) {
        keepEdgeEllipse(hole, pair, single);
        slopAperture = hole.aperture;        // look there next time
        slopSeen = LAUHoleView::Corner();
        slopSeenValid = true;
        slopSeenHole = true;
        slopSeenAperture = hole.aperture;
        slopSeenRadius = 0.0;
    }

    // finding a spot: plain paper under the hole, or on to the next spot
    if (edgeStage == EdgeSpot) {
        if (found) {
            edgeSpotX = edgeX;
            edgeReference = hole.aperture;
            edgeReferenceSet = true;
            edgeSide = +1;
            edgeNextReturn();
            return;
        }
        if (++edgeSpotTry >= edgeSpots.count()) {
            endEdgeSlop(QString("No plain paper was found under the hole at any of the %1 spots near the page's corners (last: %2). "
                                "The views are saved in %3").arg(edgeSpots.count()).arg(why).arg(QDir::toNativeSeparators(edgeFolder)));
            return;
        }
        edgeY = edgeSpots[edgeSpotTry].y;
        edgeMoveTo(edgeSpots[edgeSpotTry].x, EdgeSpot, QString("Looking for plain paper to see the hole on (spot %1 of %2)...")
                                                           .arg(edgeSpotTry + 1).arg(edgeSpots.count()));
        return;
    }

    if (!found && ++edgeFailures <= 3) {
        edgeTimer->start(SETTLE_MS);         // look again
        return;
    }
    edgeFailures = 0;
    EdgeStop s;
    s.predicting = !edgeLearning;
    s.side = edgeSide;
    s.commandedX = edgeX;
    s.reportedX = single;
    s.reportedY = pair;
    s.found = found;
    s.aperture = hole.aperture;
    s.note = why;
    if (!found) {
        edgeStops << s;
        cameraWidget->showMessage(QString("The camera couldn't find the hole (%1). Its view is saved as stop_%2.png in %3")
                                      .arg(why).arg(stop, 3, 10, QChar('0')).arg(QDir::toNativeSeparators(edgeFolder)));
        edgeAborted();                       // what was measured is still kept
        return;
    }
    const int i = sideIndex(edgeSide);
    if (!edgeLearning) {
        // how far, in X, the hole landed from where the midpoint of the two sides puts it at the spot
        const cv::Point2d middle = (edgeMean(0) + edgeMean(1)) * 0.5;
        s.missX = (middle + ax * pair + ay * edgeSpotX - hole.centre).dot(ay) / ay.dot(ay);
        edgeHits[i] = (std::abs(s.missX) < edgeTolerance) ? edgeHits[i] + 1 : 0;
    }
    edgeStops << s;
    edgeApertures[i] << hole.aperture;       // learning or predicting, every landing refines this side

    if (edgeLearning) {
        // learned once each side has a few and the X slop between them is pinned down to well within the tolerance
        const int n0 = edgeApertures[0].count(), n1 = edgeApertures[1].count();
        if (n0 >= MIN_LEARN && n1 >= MIN_LEARN) {
            double v[2] = { 0.0, 0.0 };
            for (int k = 0; k < 2; k++) {
                const cv::Point2d mean = edgeMean(k);
                for (const cv::Point2d &a : edgeApertures[k]) {
                    const double d = (a - mean).dot(ay) / ay.dot(ay);
                    v[k] += d * d / std::max(1, (int)edgeApertures[k].count() - 1);
                }
            }
            const double standardError = std::sqrt(v[0] / n0 + v[1] / n1);
            if (standardError < 0.5 * edgeTolerance || (n0 >= MAX_LEARN && n1 >= MAX_LEARN)) {
                edgeLearning = false;
            }
        }
    } else if (edgeHits[0] >= edgeTries && edgeHits[1] >= edgeTries) {
        edgeConverged();                     // show the results and ask whether to keep going
        return;
    }
    edgeSide = -edgeSide;                    // move and return, the other way
    edgeNextReturn();
}

// The live picture around the hole, enlarged, with what the camera found at the last stop: the page's two edges
// (cyan) and their corner (magenta), and the i1's hole as a bullseye (green) where its rim was found.  The corner is
// fixed to the page and the hole to the carriage, so each is carried along with the sliders between stops.
void LAUGantryMainWindow::showSlopZoom(const cv::Mat &raw16)
{
    if (!edgeSloping || !homeDialog || raw16.empty() || !singleRail.isKnown() || !pairRail.isKnown()) {
        return;
    }
    const double ZOOM_HALF_MM = 9.0;
    const double now = guideClock.elapsed() / 1000.0;
    const double pair = pairRail.estimate(now), single = singleRail.estimate(now);
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const bool found = slopSeenValid && slopSeenHole;
    const cv::Point2d hole = (found ? slopSeenAperture : slopAperture) + ax * pair + ay * single;
    // tracking the hole: the bullseye stays fixed to the carriage, at the midpoint of the two sides once both are
    // known (else where the hole was first seen), and the hole the camera sees is drawn against it; otherwise the
    // bullseye is the hole
    const bool tracking = edgeSloping;
    cv::Point2d reference = hole;
    if (tracking) {
        const cv::Point2d anchor = (!edgeApertures[0].isEmpty() && !edgeApertures[1].isEmpty()) ? (edgeMean(0) + edgeMean(1)) * 0.5
                                   : edgeReferenceSet                                            ? edgeReference
                                                                                                 : slopAperture;
        reference = anchor + ax * pair + ay * single;
    }
    cv::Point2d centre, edge;
    if (!platenOverlay.project(reference, pair, single, &centre) || !platenOverlay.project(reference + cv::Point2d(4.9, 0.0), pair, single, &edge)) {
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
    const int size = 460;
    const double scale = size / (double)box.width;
    cv::Mat big, colour;
    cv::resize(crop, big, cv::Size(size, size), 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(big, colour, cv::COLOR_GRAY2BGR);
    const auto at = [&](const cv::Point2d &pixel) {   // raw pixel -> zoom, 1/16 pixel
        return cv::Point((int)std::lround(((pixel.x - box.x + 0.5) * scale - 0.5) * 16.0), (int)std::lround(((pixel.y - box.y + 0.5) * scale - 0.5) * 16.0));
    };
    const auto project = [&](const cv::Point2d &board, cv::Point *zoomed) {
        cv::Point2d p;
        if (!platenOverlay.project(board, pair, single, &p)) {
            return false;
        }
        *zoomed = at(p);
        return true;
    };
    // a board-mm line, a little at a time (the lens bends straight lines)
    const auto drawLine = [&](const cv::Point2d &from, const cv::Point2d &to, const cv::Scalar &c, int thickness) {
        std::vector<cv::Point> run;
        for (int s = 0; s <= 80; s++) {
            cv::Point p;
            if (project(from + (to - from) * (s / 80.0), &p)) {
                run.push_back(p);
            }
        }
        cv::polylines(colour, run, false, c, thickness, cv::LINE_AA, 4);
    };
    const auto drawCircle = [&](const cv::Point2d &c, double r, const cv::Scalar &ink, int thickness) {
        std::vector<cv::Point> ring;
        for (int s = 0; s <= 72; s++) {
            cv::Point p;
            if (project(c + cv::Point2d(std::cos(s * CV_PI / 36.0), std::sin(s * CV_PI / 36.0)) * r, &p)) {
                ring.push_back(p);
            }
        }
        cv::polylines(colour, ring, false, ink, thickness, cv::LINE_AA, 4);
    };
    const cv::Scalar green(0, 220, 0), cyan(255, 255, 0), magenta(255, 0, 255), grey160(160, 160, 160);

    // the page's edges and corner, as found at the last stop
    if (slopSeenValid) {
        for (int e = 0; e < 2; e++) {
            for (const cv::Point2d &b : slopSeen.edgePoints[e]) {
                cv::Point p;
                if (project(b, &p)) {
                    cv::circle(colour, p, 16, cyan, 1, cv::LINE_AA, 4);
                }
            }
            if (slopSeen.edgeFitted[e]) {
                drawLine(slopSeen.edgeAt[e] - slopSeen.edgeDir[e] * 2.0 * ZOOM_HALF_MM, slopSeen.edgeAt[e] + slopSeen.edgeDir[e] * 2.0 * ZOOM_HALF_MM, cyan, 1);
            }
        }
        cv::Point k;
        if (slopSeen.atValid && project(slopSeen.at, &k)) {
            cv::circle(colour, k, 6 * 16, magenta, 2, cv::LINE_AA, 4);
            cv::line(colour, k - cv::Point(9 * 16, 0), k + cv::Point(9 * 16, 0), magenta, 1, cv::LINE_AA, 4);
            cv::line(colour, k - cv::Point(0, 9 * 16), k + cv::Point(0, 9 * 16), magenta, 1, cv::LINE_AA, 4);
        }
    }

    // tracking the hole: where it has been, fixed to the board, as a fading trail
    if (edgeSloping) {
        for (int k = 0; k < edgeTrail.count(); k++) {
            cv::Point p;
            if (project(edgeTrail[k], &p)) {
                const double age = (edgeTrail.count() - 1 - k) / std::max(1.0, (double)edgeTrail.count());
                cv::circle(colour, p, 2 * 16, cv::Scalar(0, (int)std::lround(230 - 150 * age), 255), cv::FILLED, cv::LINE_AA, 4);
            }
        }
    }

    // the i1's hole, always as a bullseye: bright where its rim was found, dim where it is expected until then
    {
        const cv::Point2d at = tracking ? reference : hole;
        const double r = (!tracking && found && slopSeenRadius > 0.0) ? slopSeenRadius : 4.9;
        const cv::Scalar ink = (found || tracking) ? green : cv::Scalar(0, 150, 0);
        for (double f : { 1.0, 2.0 / 3.0, 1.0 / 3.0 }) {
            drawCircle(at, r * f, ink, f == 1.0 ? 2 : 1);
        }
        cv::Point c;
        if (project(at, &c)) {
            cv::circle(colour, c, 3 * 16, ink, cv::FILLED, cv::LINE_AA, 4);
        }
        drawLine(at - cv::Point2d(r, 0), at + cv::Point2d(r, 0), ink, 1);
        drawLine(at - cv::Point2d(0, r), at + cv::Point2d(0, r), ink, 1);
    }
    // tracking: the hole as the camera sees it, a yellow circle and centre, joined to the bullseye's centre
    if (tracking && found) {
        const cv::Scalar yellow(0, 230, 255);
        if (edgeEllipseValid) {
            // the best-fit ellipse to the hole's outline, as the camera sees it
            const cv::Point2d c0 = edgeEllipse.ellipseCentre + ax * pair + ay * single;
            const double t = edgeEllipse.ellipseAngleDeg * CV_PI / 180.0;
            const cv::Point2d u(std::cos(t), std::sin(t)), v(-std::sin(t), std::cos(t));
            std::vector<cv::Point> ring;
            for (int s = 0; s <= 96; s++) {
                const double a = s * CV_PI / 48.0;
                cv::Point p;
                if (project(c0 + u * (edgeEllipse.ellipseSemiA * std::cos(a)) + v * (edgeEllipse.ellipseSemiB * std::sin(a)), &p)) {
                    ring.push_back(p);
                }
            }
            cv::polylines(colour, ring, false, yellow, 2, cv::LINE_AA, 4);
        } else {
            drawCircle(hole, 4.9, yellow, 2);
        }
        cv::Point c, b;
        if (project(hole, &c)) {
            cv::line(colour, c - cv::Point(7 * 16, 0), c + cv::Point(7 * 16, 0), yellow, 2, cv::LINE_AA, 4);
            cv::line(colour, c - cv::Point(0, 7 * 16), c + cv::Point(0, 7 * 16), yellow, 2, cv::LINE_AA, 4);
            if (project(reference, &b)) {
                cv::line(colour, b, c, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 4);
            }
        }
        const cv::Point2d d = hole - reference;
        const double det = ax.x * ay.y - ay.x * ax.y;
        const double dx = (ax.x * d.y - d.x * ax.y) / det, dy = (d.x * ay.y - ay.x * d.y) / det;   // slider mm
        cv::putText(colour, QString("hole from the bullseye: X %1, Y %2 mm").arg(dx, 0, 'f', 3).arg(dy, 0, 'f', 3).toStdString(), cv::Point(10, 66),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, yellow, 1, cv::LINE_AA);
    }
    if (found) {
        cv::Point c;
        if (slopSeen.atValid) {
            cv::Point k;
            if (project(slopSeen.at, &k) && project(hole, &c)) {
                cv::line(colour, c, k, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 4);
            }
            const cv::Point2d d = hole - slopSeen.at;
            cv::putText(colour, QString("hole from corner: %1, %2 mm").arg(d.x, 0, 'f', 3).arg(d.y, 0, 'f', 3).toStdString(), cv::Point(10, 46),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1, cv::LINE_AA);
        }
    }

    QString title = QString("the i1's hole");
    if (!slopZoomTitle.isEmpty()) {
        title = slopZoomTitle;
    }
    cv::putText(colour, title.toStdString(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    if (!slopZoomNote.isEmpty()) {
        cv::putText(colour, slopZoomNote.toStdString(), cv::Point(10, 46), cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1, cv::LINE_AA);
    }
    if (slopSeenValid && !slopSeenHole && !slopSeenWhy.isEmpty()) {
        cv::putText(colour, slopSeenWhy.left(64).toStdString(), cv::Point(10, 66), cv::FONT_HERSHEY_SIMPLEX, 0.4, cyan, 1, cv::LINE_AA);
    }
    cv::putText(colour, tracking ? "green bullseye: fixed to the carriage   yellow: the hole the camera sees" : "green bullseye: i1 hole   magenta: corner   cyan: page edges", cv::Point(10, size - 28), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(colour, QString("%1 mm across").arg(2 * ZOOM_HALF_MM, 0, 'f', 0).toStdString(), cv::Point(10, size - 10), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    homeDialog->setZoom(QImage(colour.data, colour.cols, colour.rows, (int)colour.step, QImage::Format_BGR888).copy());
}

// the hole's ellipse for the close-up, its centre made relative to the carriage so it moves with it
void LAUGantryMainWindow::keepEdgeEllipse(const LAUHoleView::Hole &hole, double pairMM, double singleMM)
{
    if (!hole.hasEllipse) {
        return;
    }
    edgeEllipse = hole;
    edgeEllipse.ellipseCentre = hole.ellipseCentre - holeView.boardAxis(0) * pairMM - holeView.boardAxis(1) * singleMM;
    edgeEllipseValid = true;
}

// The close-up's live tracking: the hole found in the live picture a few times a second, over paper
void LAUGantryMainWindow::trackHoleLive(const cv::Mat &raw16)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (!edgeSloping || raw16.empty() || now - edgeTrackedAt < TRACK_EVERY_MS || !singleRail.isKnown() || !pairRail.isKnown()) {
        return;
    }
    edgeTrackedAt = now;
    const double t = guideClock.elapsed() / 1000.0;
    const double pair = pairRail.estimate(t), single = singleRail.estimate(t);
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);
    LAUHoleView::Hole hole;
    QString why;
    if (!holeView.findHole(gray, pair, single, (edgeTracked ? edgeTrackedAperture : slopAperture) + ax * pair + ay * single, &hole, &why)) {
        return;                              // over print or the board: keep the last
    }
    edgeTracked = true;
    edgeTrackedAperture = hole.aperture;
    keepEdgeEllipse(hole, pair, single);
    edgeTrail << hole.centre;
    while (edgeTrail.count() > TRAIL) {
        edgeTrail.removeFirst();
    }
    slopSeen = LAUHoleView::Corner();
    slopSeenValid = true;
    slopSeenHole = true;
    slopSeenAperture = hole.aperture;
    slopSeenRadius = 0.0;
    if (!edgeApertures[0].isEmpty() && !edgeApertures[1].isEmpty()) {
        const cv::Point2d middle = (edgeMean(0) + edgeMean(1)) * 0.5;
        slopZoomNote = QString("hole vs carriage: %1 mm in X from the midpoint").arg((hole.aperture - middle).dot(ay) / ay.dot(ay), 0, 'f', 3);
    }
}

// Abort: the sliders have stopped; whatever was measured is still saved and plotted, if both sides were seen
void LAUGantryMainWindow::edgeAborted()
{
    if (edgeApertures[0].isEmpty() || edgeApertures[1].isEmpty()) {
        endEdgeSlop(QString("Measuring the slop was stopped before the hole was seen arriving from both sides, so there's no slop to report. "
                            "The stops so far are in %1").arg(QDir::toNativeSeparators(edgeFolder)));
        return;
    }
    edgeWasAborted = true;
    finishEdgeSlop();
}

// the results so far: hole_slop.csv, .json and .png; returns the summary and where the plot is
QString LAUGantryMainWindow::writeEdgeResults(QString *plotOut)
{
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const double det = ax.x * ay.y - ay.x * ax.y;
    const auto toSliders = [&](const cv::Point2d &m) {   // board mm -> (X single, Y pair) slider mm
        return cv::Point2d((ax.x * m.y - m.x * ax.y) / det, (m.x * ay.y - ay.x * m.y) / det);
    };
    const cv::Point2d left = edgeMean(0), right = edgeMean(1), middle = (left + right) * 0.5;
    const cv::Point2d slop = toSliders(right - left);   // arriving moving right, less arriving moving left
    const bool converged = edgeHits[0] >= edgeTries && edgeHits[1] >= edgeTries;

    // ---- the log
    QFile csv(QDir(edgeFolder).filePath(QString("hole_slop.csv")));
    if (csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "stop,stage,arriving,commanded_x_mm,reported_x_mm,reported_y_mm,found,hole_vs_carriage_x_mm,hole_vs_carriage_y_mm,miss_x_mm,note\n";
        for (int k = 0; k < edgeStops.count(); k++) {
            const EdgeStop &s = edgeStops[k];
            const cv::Point2d d = toSliders(s.aperture - middle);
            out << k + 1 << ',' << (s.predicting ? "predict" : "learn") << ',' << (s.side > 0 ? "moving_left" : "moving_right") << ','
                << QString::number(s.commandedX, 'f', 4) << ',' << QString::number(s.reportedX, 'f', 4) << ',' << QString::number(s.reportedY, 'f', 4) << ','
                << (s.found ? 1 : 0) << ',' << (s.found ? QString::number(d.x, 'f', 4) : QString()) << ',' << (s.found ? QString::number(d.y, 'f', 4) : QString())
                << ',' << (s.predicting ? QString::number(s.missX, 'f', 4) : QString()) << ",\"" << QString(s.note).replace(QChar('"'), QChar('\'')) << "\"\n";
        }
    }

    // ---- the plot: left, where the hole sat relative to the carriage after each return (slider mm from the
    // midpoint); right, how far each predicted return landed from where it was meant to, in X
    const int width = 1200, height = 680, top = 130;
    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    const cv::Scalar ink(40, 40, 40), grid(215, 215, 215), leftColour(200, 90, 20), rightColour(30, 120, 230);
    double largest = std::max(edgeTolerance * 2.0, 0.02);
    for (const EdgeStop &s : edgeStops) {
        if (s.found) {
            largest = std::max(largest, cv::norm(toSliders(s.aperture - middle)));
        }
    }
    largest *= 1.15;
    const cv::Point centre(300, top + (height - top) / 2 - 10);
    const double half = 0.5 * (height - top) - 40.0, pxPerMM = half / largest;
    const auto scatterAt = [&](const cv::Point2d &d) {
        return cv::Point((int)std::lround(centre.x + d.x * pxPerMM), (int)std::lround(centre.y - d.y * pxPerMM));
    };
    for (int r = 1; r <= 4; r++) {
        cv::circle(plot, centre, (int)std::lround(half * r / 4.0), grid, 1, cv::LINE_AA);
        cv::putText(plot, QString("%1").arg(largest * r / 4.0, 0, 'f', 3).toStdString(), centre + cv::Point((int)std::lround(half * r / 4.0) + 2, -3),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    }
    cv::line(plot, centre - cv::Point((int)half, 0), centre + cv::Point((int)half, 0), grid, 1, cv::LINE_AA);
    cv::line(plot, centre - cv::Point(0, (int)half), centre + cv::Point(0, (int)half), grid, 1, cv::LINE_AA);
    cv::putText(plot, "+X", centre + cv::Point((int)half + 4, 4), cv::FONT_HERSHEY_SIMPLEX, 0.45, ink, 1, cv::LINE_AA);
    cv::putText(plot, "+Y", centre + cv::Point(-10, -(int)half - 6), cv::FONT_HERSHEY_SIMPLEX, 0.45, ink, 1, cv::LINE_AA);
    for (const EdgeStop &s : edgeStops) {
        if (!s.found) {
            continue;
        }
        const cv::Scalar colour = (s.side > 0) ? leftColour : rightColour;
        cv::circle(plot, scatterAt(toSliders(s.aperture - middle)), 5, colour, s.predicting ? cv::FILLED : 1, cv::LINE_AA);
    }
    for (int k = 0; k < 2; k++) {
        const cv::Point m = scatterAt(toSliders(edgeMean(k) - middle));
        cv::drawMarker(plot, m, k == 0 ? leftColour : rightColour, cv::MARKER_CROSS, 26, 2, cv::LINE_AA);
    }
    cv::putText(plot, "hole relative to the carriage, mm from the midpoint", cv::Point(70, top - 8), cv::FONT_HERSHEY_SIMPLEX, 0.45, ink, 1, cv::LINE_AA);

    // misses of the predicted returns
    std::vector<const EdgeStop *> predicted;
    for (const EdgeStop &s : edgeStops) {
        if (s.predicting && s.found) {
            predicted.push_back(&s);
        }
    }
    const int x0 = 660, x1 = width - 30, y0 = top + 10, y1 = height - 50;
    double worst = edgeTolerance * 2.0;
    for (const EdgeStop *s : predicted) {
        worst = std::max(worst, std::abs(s->missX));
    }
    worst *= 1.1;
    const int count = std::max(2, (int)predicted.size());
    const auto missAt = [&](int k, double miss) {
        return cv::Point((int)std::lround(x0 + (x1 - x0) * (k / (double)(count - 1))),
                         (int)std::lround(y0 + (y1 - y0) * 0.5 * (1.0 - std::clamp(miss / worst, -1.0, 1.0))));
    };
    cv::rectangle(plot, missAt(0, edgeTolerance), missAt(count - 1, -edgeTolerance), cv::Scalar(225, 245, 225), cv::FILLED);
    for (double v : { worst, 0.0, -worst }) {
        cv::line(plot, missAt(0, v), missAt(count - 1, v), grid, 1, cv::LINE_AA);
        cv::putText(plot, QString("%1").arg(v, 0, 'f', 3).toStdString(), missAt(0, v) + cv::Point(-56, 4), cv::FONT_HERSHEY_SIMPLEX, 0.35, ink, 1, cv::LINE_AA);
    }
    cv::Point previous[2];
    bool havePrevious[2] = { false, false };
    for (int k = 0; k < (int)predicted.size(); k++) {
        const int i = sideIndex(predicted[k]->side);
        const cv::Scalar colour = (i == 0) ? leftColour : rightColour;
        const cv::Point p = missAt(k, predicted[k]->missX);
        if (havePrevious[i]) {
            cv::line(plot, previous[i], p, colour, 1, cv::LINE_AA);
        }
        previous[i] = p;
        havePrevious[i] = true;
        cv::circle(plot, p, 5, colour, cv::FILLED, cv::LINE_AA);
    }
    cv::putText(plot, "predicted returns: how far each landed from the midpoint, mm in X", cv::Point(x0, top - 8), cv::FONT_HERSHEY_SIMPLEX, 0.45, ink, 1,
                cv::LINE_AA);

    const QStringList lines{
        QString("Slop by tracking the hole, scan %1, moving %2 mm each way").arg(edgeScan).arg(edgeAway, 0, 'f', 1),
        QString("arriving moving right vs moving left, the hole shifts X %1 mm, Y %2 mm  (in slider mm)").arg(slop.x, 0, 'f', 3).arg(slop.y, 0, 'f', 3),
        QString("correction: add %1 mm x lastX to X   (lastX: +1 after moving right, -1 after moving left)").arg(-0.5 * slop.x, 0, 'f', 4),
        converged ? QString("predicted returns land within %1 mm from both sides, %2 in a row").arg(edgeTolerance, 0, 'f', 3).arg(edgeTries)
                  : QString("ABORTED: %1 and %2 in a row within %3 mm, %4 needed").arg(edgeHits[0]).arg(edgeHits[1]).arg(edgeTolerance, 0, 'f', 3).arg(edgeTries),
        QString("open: learning   filled: predicted   crosses: averages   blue: arriving moving left   orange: moving right"),
    };
    for (int k = 0; k < lines.count(); k++) {
        cv::putText(plot, lines[k].toStdString(), cv::Point(14, 22 + 20 * k), cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
    }
    const QString plotPath = QDir(edgeFolder).filePath(QString("hole_slop.png"));
    QImage(plot.data, plot.cols, plot.rows, (int)plot.step, QImage::Format_BGR888).save(plotPath);

    QJsonObject json;
    json["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["scan"] = edgeScan;
    json["spot_x_y_mm"] = QJsonArray{ edgeSpotX, edgeY };
    json["away_mm"] = edgeAway;
    json["hole_vs_carriage_moving_left_board_mm"] = QJsonArray{ left.x, left.y };
    json["hole_vs_carriage_moving_right_board_mm"] = QJsonArray{ right.x, right.y };
    json["returns"] = QJsonArray{ (int)edgeApertures[0].count(), (int)edgeApertures[1].count() };
    json["slop_right_minus_left_x_mm"] = slop.x;
    json["slop_right_minus_left_y_mm"] = slop.y;
    json["correction_x_per_last_x_sign_mm"] = -0.5 * slop.x;   // add lastX * this to X to land at the midpoint
    json["converged"] = converged;
    json["aborted"] = edgeWasAborted;
    json["tolerance_mm"] = edgeTolerance;
    json["in_a_row_needed"] = edgeTries;
    json["in_a_row_reached"] = QJsonArray{ edgeHits[0], edgeHits[1] };
    QFile jsonFile(QDir(edgeFolder).filePath(QString("hole_slop.json")));
    if (jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        jsonFile.write(QJsonDocument(json).toJson());
    }

    *plotOut = plotPath;
    return QString("Slop by tracking the hole: arriving moving right the hole sits %1 mm further in X (%2 mm in Y) than arriving moving left, "
                   "over %3 + %4 returns. %5 Results are in %6")
        .arg(slop.x, 0, 'f', 3).arg(slop.y, 0, 'f', 3).arg(edgeApertures[0].count()).arg(edgeApertures[1].count())
        .arg(converged ? QString("Corrected returns now land within %1 mm from both sides.").arg(edgeTolerance, 0, 'f', 3)
                       : QString("Stopped before corrected returns landed within %1 mm %2 times in a row from both sides.")
                             .arg(edgeTolerance, 0, 'f', 3).arg(edgeTries))
        .arg(QDir::toNativeSeparators(edgeFolder));
}

// stopped (or aborted): save, end, and show the plot
void LAUGantryMainWindow::finishEdgeSlop()
{
    QString plotPath;
    const QString message = writeEdgeResults(&plotPath);
    endEdgeSlop(message);
    const QImage shown(plotPath);
    if (!shown.isNull()) {
        LAUImageInspectDialog *inspect = new LAUImageInspectDialog(shown, QString("Slop"), this);
        inspect->setAttribute(Qt::WA_DeleteOnClose);
        inspect->show();
    }
}

// Converged: corrected returns landed still enough times in a row from both sides.  Save, show the plot, and ask.
void LAUGantryMainWindow::edgeConverged()
{
    QString plotPath;
    const QString message = writeEdgeResults(&plotPath);
    QDialog ask(homeDialog ? static_cast<QWidget *>(homeDialog) : static_cast<QWidget *>(this));
    ask.setWindowTitle(QString("Slop Predicted"));
    QVBoxLayout *layout = new QVBoxLayout(&ask);
    QLabel *picture = new QLabel();
    const QImage plot(plotPath);
    if (!plot.isNull()) {
        picture->setPixmap(QPixmap::fromImage(plot.scaledToWidth(std::min(plot.width(), 1000), Qt::SmoothTransformation)));
    }
    layout->addWidget(picture);
    QLabel *text = new QLabel(QString("<b>The slop is being predicted well:</b> corrected returns landed within %1 mm of the bullseye %2 times in a row "
                                      "from both sides.<br><br>%3<br><br>Keep going to refine it further, or stop here?")
                                  .arg(edgeTolerance, 0, 'f', 3).arg(edgeTries).arg(message.toHtmlEscaped()));
    text->setWordWrap(true);
    layout->addWidget(text);
    QDialogButtonBox *buttons = new QDialogButtonBox();
    QPushButton *keep = buttons->addButton(QString("Keep going"), QDialogButtonBox::AcceptRole);
    QPushButton *stop = buttons->addButton(QString("Stop"), QDialogButtonBox::RejectRole);
    stop->setDefault(true);
    connect(keep, &QPushButton::clicked, &ask, &QDialog::accept);
    connect(stop, &QPushButton::clicked, &ask, &QDialog::reject);
    layout->addWidget(buttons);
    const bool keepGoing = ask.exec() == QDialog::Accepted;
    if (!edgeSloping) {
        return;
    }
    if (!keepGoing || edgeCancelling) {
        endEdgeSlop(message);                // the results are already saved
        return;
    }
    edgeHits[0] = edgeHits[1] = 0;           // refine until it converges again, then ask again
    edgeSide = -edgeSide;
    edgeNextReturn();
}

void LAUGantryMainWindow::endEdgeSlop(const QString &message)
{
    edgeSloping = false;
    edgeCancelling = false;
    edgeWaitingFrame = false;
    if (edgeTimer) {
        edgeTimer->stop();
    }
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    slopZoomTitle.clear();
    slopZoomNote.clear();
    railBox->setEnabled(true);
    cameraWidget->lockScanActions(false);
    cameraWidget->showMessage(message);
}

// ---- the slop correction, for the corner fine-tune and Go to page position --------------------------------------

// the newest hole-slop measurement's correction: X and Y, slider mm, per sign of the last X move
bool LAUGantryMainWindow::loadSlopCorrection()
{
    slopCorrectionLoaded = false;
    slopCorrX = slopCorrY = 0.0;
    slopCorrSource.clear();
    QDir base(scanBase);
    for (const QString &name : base.entryList(QStringList{ QString("*_holeslop") }, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        QFile f(base.filePath(name + QString("/hole_slop.json")));
        const QJsonObject o = f.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(f.readAll()).object() : QJsonObject();
        if (o.contains(QString("correction_x_per_last_x_sign_mm"))) {
            slopCorrX = o["correction_x_per_last_x_sign_mm"].toDouble();
            slopCorrY = -0.5 * o["slop_right_minus_left_y_mm"].toDouble();
            slopCorrSource = name + (o["converged"].toBool() ? QString() : QString(" (not converged)"));
            slopCorrectionLoaded = true;
            return true;
        }
    }
    return false;
}

// a new run of corrected moves: the direction of the last X move isn't known until the first move
void LAUGantryMainWindow::resetSlopTracking()
{
    slopDir = 0;
    slopLogicalX = railWidget->reportedMM(0);
    slopLogicalKnown = true;
}

// move to a slop-free position: the correction for the direction this move leaves X moving in is added
void LAUGantryMainWindow::moveRailsSlop(double single, double pair)
{
    if (slopLogicalKnown) {
        const double d = single - slopLogicalX;
        if (std::abs(d) > 0.005) {
            slopDir = (d > 0.0) ? 1 : -1;
        }
    }
    slopLogicalX = single;
    slopLogicalKnown = true;
    if (!slopCorrectionLoaded) {
        railWidget->moveRails(single, pair);
        return;
    }
    railWidget->moveRails(single + slopShiftX(), pair + slopShiftY());
}
