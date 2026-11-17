/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Measure slop: the i1's holder can turn a little on its bracket, so where its hole ends up depends on the way the
// sliders were moving when they stopped.  The camera rides on the carriage, not the holder, so it sees the hole move.
//
// The hole is put over a corner of the last page found, the corner a little in from the hole's centre, from each of
// the twelve hours of a clock (in slider space: 12 o'clock is +Y, the rail pair; 3 o'clock is +X, the single rail), a
// few rounds over.  For each hour the sliders go out to that hour, the given distance away, then back, settle, and
// LAUHoleView::findHoleAtCorner finds the corner from its two edges and the hole from its rim over the paper.  The
// hole's offset from the corner -- where the i1 lands on the page, carriage and holder both -- against its average is
// turned into the correction the sliders need, in slider mm, and plotted around a clock face.
//
// The slop left by the move back can depend on the move away, so the correction is solved for as a linear system in
// the signs of each slider's last two moves (c0 + c1 lastX + c2 lastY + c3 prevX + c4 prevY, for X and for Y).  Going
// straight out and back makes the move away the move back reversed, which can't tell the two apart, so the rounds
// alternate: straight out; farther out first and in to the hour, so the last two moves go the same way; or out to
// the hour 3 ahead first and across to the hour.
//
// Confirm slop sends the corner into the middle of the hole with each approach corrected by that prediction, adds what
// is still needed to the estimate after each pass, and saves the revised estimate as a new measurement.
//
#include "laugantrymainwindow.h"
#include "lauimageinspectdialog.h"

#include <cmath>
#include <algorithm>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QTimer>
#include <QDialog>
#include <QImage>
#include <QComboBox>
#include <QPushButton>
#include <QDateTime>
#include <QSettings>
#include <QStandardItemModel>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QMessageBox>
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
    const double MOVED_MM = 0.005;           // a slider that moved less than this didn't move
    const double CORNER_IN_MM = 2.0;         // the hole's centre, this far in from the corner along both edges
    const double CONVERGED_MM = 0.01;        // a confirm pass that changes the estimate less than this is the last

    cv::Point2d unitOf(const cv::Point2d &v)
    {
        return v * (1.0 / std::max(cv::norm(v), 1e-12));
    }

    // the hour's direction in slider space, (single, pair): 12 is +pair, 3 is +single
    cv::Point2d hourDirection(int hour)
    {
        const double a = hour * CV_PI / 6.0;
        return cv::Point2d(std::sin(a), std::cos(a));
    }

    int signOf(double v)
    {
        return (v > MOVED_MM) ? 1 : (v < -MOVED_MM) ? -1 : 0;
    }

    // a slider's last two moves after one more of sign s (0: it didn't move, and remembers what it had)
    void afterMove(int s, int *last, int *prev)
    {
        if (s) {
            *prev = *last;
            *last = s;
        }
    }

    // the terms the correction is linear in
    void slopTerms(int lastX, int lastY, int prevX, int prevY, double *f)
    {
        f[0] = 1.0;
        f[1] = lastX;
        f[2] = lastY;
        f[3] = prevX;
        f[4] = prevY;
    }

    // The way out for a round, so each slider's last two moves come in every combination: straight out to the hour and
    // back (the move before is the last one reversed); out FARTHER first, then in to the hour (on each slider the move
    // before goes the same way as the last); or out to the hour 3 ahead first and across (the sliders mixed).
    // Returns 0 for straight, FARTHER_FIRST, or the hour it goes to first.
    const int FARTHER_FIRST = -1;
    const double FARTHER = 1.5;              // that far out, times the approach
    int viaHour(int round, int hour)
    {
        switch (round % 3) {
            case 1:
                return FARTHER_FIRST;
            case 2:
                return (hour + 2) % 12 + 1;  // 3 ahead
            default:
                return 0;
        }
    }
}

void LAUGantryMainWindow::onSlopRequested()
{
    startSlop(false);
}

void LAUGantryMainWindow::onSlopConfirmRequested()
{
    startSlop(true);
}

// checkMode: confirm the newest measurement, the corner sent into the middle of the hole with the slop corrected
void LAUGantryMainWindow::startSlop(bool checkMode)
{
    const QString title = checkMode ? QString("Confirm Slop") : QString("Measure Slop");
    if (scanning || measuring || merging || returningHome || tuning || sloping || edgeSloping || pageMoving || railWidget->isHoming()) {
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
    slopScan = QFileInfo(QFileInfo(folder).absolutePath()).fileName();
    QFile file(QDir(folder).filePath(QString("page.json")));
    const QJsonArray corners = (file.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject())["corners_board_mm"].toArray();
    QString error;
    if (corners.count() != 4 || !slopGeometry.load(folder, &error) || !slopGeometry.hasRails()) {
        QMessageBox::warning(this, title, error.isEmpty() ? QString("The page or the hole's position in scan %1 couldn't be read.").arg(slopScan) : error);
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

    // which corner, how far out, how many rounds
    const QStringList names{ QString("top-left"), QString("top-right"), QString("bottom-right"), QString("bottom-left") };
    std::vector<cv::Point2d> P;
    for (int n = 0; n < 4; n++) {
        P.emplace_back(corners[n].toArray().at(0).toDouble(), corners[n].toArray().at(1).toDouble());
    }
    // a corner whose spot, or any hour's start around it, is past the limit sliders isn't offered
    const auto spotFor = [&](int k, double in, double *single, double *pair) {
        const cv::Point2d spot = P[k] + unitOf(P[(k + 1) % 4] - P[k]) * in + unitOf(P[(k + 3) % 4] - P[k]) * in;
        slopGeometry.railsFor(spot, single, pair, true);
    };
    // the newest measurement of this scan, to check: the corner it used, the slider position that centres that corner
    // in the hole, and the fit that predicts each approach's slop
    QString measured;
    QJsonObject measurement;
    for (const QString &name : base.entryList(QStringList{ QString("*_slop") }, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        QFile f(base.filePath(name + QString("/slop.json")));
        const QJsonObject o = f.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(f.readAll()).object() : QJsonObject();
        if (o["mode"].toString() == QString("measure") && o["scan"].toString() == slopScan && o["corner_centred_single_pair_mm"].toArray().count() == 2) {
            measured = name;
            measurement = o;
            break;
        }
    }
    const int measuredCorner = measurement["corner_index"].toInt(-1);
    if (checkMode && (measured.isEmpty() || measuredCorner < 0)) {
        QMessageBox::information(this, title, QString("The slop hasn't been measured on the page of scan %1 yet. Choose Measure slop first.").arg(slopScan));
        return;
    }
    const cv::Point2d centred(measurement["corner_centred_single_pair_mm"].toArray().at(0).toDouble(),
                              measurement["corner_centred_single_pair_mm"].toArray().at(1).toDouble());

    const auto fits = [&](int k, double out, bool check) {
        double single = 0.0, pair = 0.0;
        if (check) {
            single = centred.x;
            pair = centred.y;
        } else {
            spotFor(k, CORNER_IN_MM, &single, &pair);
        }
        if (!slopGeometry.reachable(single, pair)) {
            return false;
        }
        for (int hour = 1; hour <= 12; hour++) {
            for (double reach : { out, FARTHER * out }) {
                const cv::Point2d d = hourDirection(hour) * reach;
                if (!slopGeometry.reachable(single + d.x, pair + d.y)) {
                    return false;
                }
            }
        }
        return true;
    };
    QSettings settings;
    QDialog dialog(this);
    dialog.setWindowTitle(title);
    QVBoxLayout *layout = new QVBoxLayout(&dialog);
    const QString measureText = QString("The sliders bring the i1's hole back over a corner of the page found in scan %1, with the corner %2 mm in from "
                                        "the hole's centre along both edges, from each of the twelve hours of a clock. Rounds alternate the way out: "
                                        "straight out and back, farther out first and then in, or out to the hour 3 ahead first, so the move away and "
                                        "the move back can be told apart. Each time the camera finds the corner and the hole's rim, and the slop is solved for as a linear "
                                        "system in the signs of each slider's last two moves. Use 3 or more rounds.")
                                    .arg(slopScan).arg(CORNER_IN_MM, 0, 'f', 1);
    const QString confirmText = QString("The sliders bring the i1's hole onto the %1 corner measured in %2, from each of the twelve hours of a clock, "
                                        "adding to each approach the slop that measurement predicts. If the slop was measured right, the corner lands "
                                        "in the middle of the bullseye every time.")
                                    .arg(names.value(measuredCorner), measured);
    QLabel *label = new QLabel((checkMode ? confirmText : measureText) +
                               QString("\n\nTake the i1 Pro out of its holder first so the camera sees the paper through the hole, and don't move the page."));
    label->setWordWrap(true);
    layout->addWidget(label);
    QFormLayout *form = new QFormLayout();
    QComboBox *corner = new QComboBox();
    corner->addItems(names);
    corner->setCurrentIndex(std::clamp(settings.value(QString("LAUGantryMainWindow::slopCorner"), 0).toInt(), 0, 3));
    QDoubleSpinBox *distance = new QDoubleSpinBox(), *rounds = new QDoubleSpinBox();
    distance->setRange(0.5, 50.0);
    distance->setSuffix(QString(" mm"));
    distance->setValue(settings.value(QString("LAUGantryMainWindow::slopDistance"), 10.0).toDouble());
    rounds->setRange(1, 10);
    rounds->setDecimals(0);
    rounds->setValue(settings.value(QString("LAUGantryMainWindow::slopRounds"), 3).toInt());
    form->addRow(QString("Corner"), corner);
    form->addRow(QString("Approach from"), distance);
    form->addRow(QString(checkMode ? "Rounds of the clock per pass" : "Rounds of the clock"), rounds);
    QDoubleSpinBox *passes = new QDoubleSpinBox();
    passes->setRange(1, 10);
    passes->setDecimals(0);
    passes->setValue(settings.value(QString("LAUGantryMainWindow::slopPasses"), 3).toInt());
    passes->setToolTip(QString("After each pass the slop still left is added to the estimate, and the next pass uses the revised estimate. "
                               "It stops early once the estimate changes by less than %1 mm.").arg(CONVERGED_MM, 0, 'f', 3));
    if (checkMode) {
        form->addRow(QString("Passes, revising the estimate"), passes);
    }
    layout->addLayout(form);
    QLabel *none = new QLabel();
    none->setWordWrap(true);
    layout->addWidget(none);
    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(QString("Start"));
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    // grey out the corners too close to the edge of the travel, for the inset and approach chosen
    QStandardItemModel *model = qobject_cast<QStandardItemModel *>(corner->model());
    const auto refresh = [&]() {
        const bool check = checkMode;
        if (check) {
            corner->setCurrentIndex(measuredCorner);
        }
        corner->setEnabled(!check);
        int first = -1;
        for (int k = 0; k < 4; k++) {
            const bool ok = check ? (k == measuredCorner && fits(k, distance->value(), true)) : fits(k, distance->value(), false);
            model->item(k)->setEnabled(ok);
            model->item(k)->setText(ok ? names[k] : names[k] + QString("  (too close to the edge of the travel)"));
            if (ok && first < 0) {
                first = k;
            }
        }
        if (first >= 0 && !model->item(corner->currentIndex())->isEnabled()) {
            corner->setCurrentIndex(first);
        }
        buttons->button(QDialogButtonBox::Ok)->setEnabled(first >= 0);
        none->setText(first >= 0 ? QString()
                                 : QString("Every corner is too close to the edge of the travel for this approach. "
                                           "Approach from closer, or widen the limit sliders."));
        none->setVisible(first < 0);
    };
    connect(distance, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dialog, refresh);
    refresh();
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    slopVerify = checkMode;
    const int k = slopVerify ? measuredCorner : corner->currentIndex();
    if (k < 0 || !fits(k, distance->value(), slopVerify)) {
        return;
    }
    if (slopVerify) {
        const QStringList terms{ QString("constant"), QString("per_last_x_sign"), QString("per_last_y_sign"), QString("per_prev_x_sign"),
                                 QString("per_prev_y_sign") };
        const QJsonObject fx = measurement["fit_x"].toObject(), fy = measurement["fit_y"].toObject();
        for (int i = 0; i < SLOP_TERMS; i++) {
            slopFitX[i] = fx[terms[i]].toDouble();   // missing (an older measurement): 0
            slopFitY[i] = fy[terms[i]].toDouble();
        }
        slopRimRadius = measurement["rim_radius_mm"].toDouble();
        slopMeasured = measured;
        slopCentred = centred;
        settings.setValue(QString("LAUGantryMainWindow::slopPasses"), (int)passes->value());
    }
    slopPass = 1;
    slopPasses = slopVerify ? (int)passes->value() : 1;
    slopHistory = QJsonArray();
    settings.setValue(QString("LAUGantryMainWindow::slopCorner"), k);
    settings.setValue(QString("LAUGantryMainWindow::slopDistance"), distance->value());
    settings.setValue(QString("LAUGantryMainWindow::slopRounds"), (int)rounds->value());
    slopCorner = names[k];
    slopDistance = distance->value();
    slopRounds = (int)rounds->value();

    if (slopVerify) {
        slopSingle = centred.x;              // the corner in the middle of the hole
        slopPair = centred.y;
    } else {
        spotFor(k, CORNER_IN_MM, &slopSingle, &slopPair);
    }
    slopAlong1 = P[(k + 1) % 4] - P[k];
    slopAlong2 = P[(k + 3) % 4] - P[k];
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
    slopFolder = QDir(scanBase).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) + QString(slopVerify ? "_slopcheck" : "_slop"));
    QDir().mkpath(slopFolder);

    if (!slopTimer) {
        slopTimer = new QTimer(this);
        slopTimer->setSingleShot(true);
        connect(slopTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onSlopTimer);
    }
    sloping = true;
    slopCancelling = false;
    slopWaitingFrame = false;
    slopShots.clear();
    slopStep = 0;
    slopAtCentre = false;
    slopLeg = 0;
    slopAtSingle = railWidget->reportedMM(0);
    slopAtPair = railWidget->reportedMM(2);
    slopLastX = 0;
    slopLastY = 0;
    slopPrevX = 0;
    slopPrevY = 0;
    cameraWidget->lockScanActions(true);
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(title);
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, [this]() {
        slopCancelling = true;
        if (railWidget->isHoming()) {
            railWidget->abortHome();         // onSlopMoveFinished ends it once they have stopped
        } else if (!slopWaitingFrame && !slopTimer->isActive()) {
            endSlop(QString("Measuring the slop was cancelled."));
        }
    });
    slopSeenValid = false;
    slopSeenHole = false;
    slopSeenWhy.clear();
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitLiveFrame, homeDialog, [this](cv::Mat raw16) { showSlopZoom(raw16); });
    homeDialog->open();
    slopNextMove();
}

// The live picture around the hole, enlarged, with what the camera found at the last stop: the page's two edges
// (cyan) and their corner (magenta), and the i1's hole as a bullseye (green) where its rim was found.  The corner is
// fixed to the page and the hole to the carriage, so each is carried along with the sliders between stops.
void LAUGantryMainWindow::showSlopZoom(const cv::Mat &raw16)
{
    if ((!sloping && !edgeSloping) || !homeDialog || raw16.empty() || !singleRail.isKnown() || !pairRail.isKnown()) {
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

    QString title = QString("%1 corner").arg(slopCorner);
    if (!slopZoomTitle.isEmpty()) {
        title = slopZoomTitle;
    } else if (!slopShots.isEmpty()) {
        title += QString(", last stop from %1 o'clock").arg(slopShots.last().hour);
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

// the next leg of this shot: out (to the via hour, then the hour), or back in to the corner
void LAUGantryMainWindow::slopNextMove()
{
    const int total = 12 * slopRounds;
    const int round = slopStep / 12, hour = slopStep % 12 + 1;
    if (slopLeg == 0) {
        slopLegs.clear();
        const int via = viaHour(round, hour);
        if (via) {
            const cv::Point2d v = (via == FARTHER_FIRST) ? hourDirection(hour) * (FARTHER * slopDistance) : hourDirection(via) * slopDistance;
            slopLegs << cv::Point2d(slopSingle + v.x, slopPair + v.y);
        }
        const cv::Point2d d = hourDirection(hour) * slopDistance;
        slopLegs << cv::Point2d(slopSingle + d.x, slopPair + d.y);
    }
    slopAtCentre = slopLeg >= slopLegs.count();   // this move ends at the corner
    double single = slopSingle, pair = slopPair;
    if (!slopAtCentre) {
        single = slopLegs[slopLeg].x;
        pair = slopLegs[slopLeg].y;
        const bool viaLeg = slopLegs.count() > 1 && slopLeg == 0;
        const int via = viaHour(round, hour);
        homeDialog->setMessage(!viaLeg ? QString("Round %1 of %2: out to %3 o'clock...").arg(round + 1).arg(slopRounds).arg(hour)
                               : (via == FARTHER_FIRST) ? QString("Round %1 of %2: farther out past %3 o'clock first...").arg(round + 1).arg(slopRounds).arg(hour)
                                                        : QString("Round %1 of %2: out to %3 o'clock first...").arg(round + 1).arg(slopRounds).arg(via));
    } else if (slopVerify) {
        // the slop this approach will leave, from the signs each slider's last two moves will have, added in advance;
        // the correction can flip a small move's sign (12 and 6 o'clock in X), so settle on signs that agree with it
        int sx = signOf(single - slopAtSingle), sy = signOf(pair - slopAtPair);
        for (int iteration = 0; iteration < 3; iteration++) {
            int lx = slopLastX, px = slopPrevX, ly = slopLastY, py = slopPrevY;
            afterMove(sx, &lx, &px);
            afterMove(sy, &ly, &py);
            double f[SLOP_TERMS];
            slopTerms(lx, ly, px, py, f);
            single = slopSingle;
            pair = slopPair;
            for (int i = 0; i < SLOP_TERMS; i++) {
                single += slopFitX[i] * f[i];
                pair += slopFitY[i] * f[i];
            }
            sx = signOf(single - slopAtSingle);
            sy = signOf(pair - slopAtPair);
        }
        homeDialog->setMessage(QString("Pass %6 of %7, round %1 of %2: onto the corner from %3 o'clock, %4 mm X and %5 mm Y added for the slop...")
                                   .arg(round + 1).arg(slopRounds).arg(hour).arg(single - slopSingle, 0, 'f', 3).arg(pair - slopPair, 0, 'f', 3)
                                   .arg(slopPass).arg(slopPasses));
    } else {
        homeDialog->setMessage(QString("Round %1 of %2: back to the spot from %3 o'clock...").arg(round + 1).arg(slopRounds).arg(hour));
    }
    homeDialog->setProgress(slopStep, total);
    afterMove(signOf(single - slopAtSingle), &slopLastX, &slopPrevX);
    afterMove(signOf(pair - slopAtPair), &slopLastY, &slopPrevY);
    slopAtSingle = single;
    slopAtPair = pair;
    railWidget->moveRails(single, pair);     // answers with emitHomeFinished -> onSlopMoveFinished
}

void LAUGantryMainWindow::onSlopMoveFinished(bool ok, const QString &message)
{
    if (slopCancelling) {
        endSlop(QString("Measuring the slop was cancelled."));
        return;
    }
    if (!ok) {
        endSlop(QString("Measuring the slop stopped: %1").arg(message));
        return;
    }
    if (!slopAtCentre) {
        slopLeg++;                           // on out, or back in
        slopNextMove();
        return;
    }
    slopTimer->start(SETTLE_MS);             // then a frame of the hole
}

void LAUGantryMainWindow::onSlopTimer()
{
    if (!sloping) {
        return;
    }
    if (slopCancelling) {
        endSlop(QString("Measuring the slop was cancelled."));
        return;
    }
    if (slopWaitingFrame) {
        slopWaitingFrame = false;
        endSlop(QString("Measuring the slop stopped: the camera stopped delivering frames."));
        return;
    }
    slopWaitingFrame = true;
    slopTimer->start(FRAME_TIMEOUT_MS);
    cameraWidget->requestScanFrame();
}

void LAUGantryMainWindow::onSlopFrame(const cv::Mat &raw16)
{
    if (!sloping || !slopWaitingFrame) {
        return;
    }
    slopWaitingFrame = false;
    slopTimer->stop();
    if (slopCancelling) {
        endSlop(QString("Measuring the slop was cancelled."));
        return;
    }
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);      // Mono10 -> 8 bits, as the scans are saved

    SlopShot shot;
    shot.round = slopStep / 12 + 1;
    shot.hour = slopStep % 12 + 1;
    shot.lastX = slopLastX;
    shot.lastY = slopLastY;
    shot.prevX = slopPrevX;
    shot.prevY = slopPrevY;
    shot.via = viaHour(shot.round - 1, shot.hour);
    const double pair = railWidget->reportedMM(2), single = railWidget->reportedMM(0);
    // where to look: where the hole has been seen so far, else where the fine-tune saw it
    cv::Point2d guess = slopAperture + holeView.boardAxis(0) * pair + holeView.boardAxis(1) * single;
    int seen = 0;
    cv::Point2d sum(0, 0);
    for (const SlopShot &s : slopShots) {
        if (s.ok) {
            sum += s.hole;
            seen++;
        }
    }
    if (seen) {
        guess = sum * (1.0 / seen);
    }
    LAUHoleView::Corner corner;
    QString why;
    cv::Mat check;
    double radius = 0.0;
    shot.ok = holeView.findHoleAtCorner(gray, pair, single, guess, slopAlong1, slopAlong2, &shot.hole, &corner, &why, &check, &radius,
                                        slopVerify ? slopRimRadius : 0.0);
    shot.radius = radius;
    slopSeen = corner;
    slopSeenValid = true;
    slopSeenHole = shot.ok;
    slopSeenAperture = shot.hole - holeView.boardAxis(0) * pair - holeView.boardAxis(1) * single;
    slopSeenRadius = radius;
    slopSeenWhy = why;
    shot.corner = corner.at;
    shot.offset = shot.hole - shot.corner;
    shot.note = why;
    slopShots << shot;
    if (!check.empty()) {
        QImage(check.data, check.cols, check.rows, (int)check.step, QImage::Format_BGR888)
            .save(QDir(slopFolder).filePath(QString("shot_r%1_h%2.png").arg(shot.round).arg(shot.hour, 2, 10, QChar('0'))));
    }

    slopStep++;
    slopAtCentre = false;
    slopLeg = 0;
    if (slopStep >= 12 * slopRounds) {
        finishSlop();
        return;
    }
    slopNextMove();
}

void LAUGantryMainWindow::finishSlop()
{
    QList<SlopShot> good;
    for (const SlopShot &s : slopShots) {
        if (s.ok) {
            good << s;
        }
    }
    if (good.count() < 4) {
        endSlop(QString("Measuring the slop: the camera found the hole only %1 of %2 times, too few to say anything. %3")
                    .arg(good.count()).arg(slopShots.count()).arg(slopShots.isEmpty() ? QString() : slopShots.last().note));
        return;
    }

    // each shot's hole-from-corner offset against the average, as the slider move that would undo it:
    // ax * dPair + ay * dSingle = mean - offset.  A check aims at the corner itself, so against zero: what is left.
    cv::Point2d mean(0, 0);
    double rimRadius = 0.0, missRms = 0.0, missWorst = 0.0;
    for (const SlopShot &s : good) {
        mean += s.offset * (1.0 / good.count());
        rimRadius += s.radius / good.count();
        missRms += s.offset.dot(s.offset) / good.count();
        missWorst = std::max(missWorst, cv::norm(s.offset));
    }
    missRms = std::sqrt(missRms);
    const cv::Point2d measuredMean = mean;
    if (slopVerify) {
        mean = cv::Point2d(0, 0);
    }
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const double det = ax.x * ay.y - ay.x * ax.y;
    const auto correctionOf = [&](const SlopShot &s) {
        const cv::Point2d m = mean - s.offset;
        return cv::Point2d((ax.x * m.y - m.x * ax.y) / det, (m.x * ay.y - ay.x * m.y) / det);   // (single, pair)
    };

    // the linear system: every shot's correction against the signs of each slider's last two moves,
    // c = c0 + c1 * lastX + c2 * lastY + c3 * prevX + c4 * prevY, for X and for Y, solved by least squares
    const int n = good.count();
    cv::Mat A(n, SLOP_TERMS, CV_64F), bx(n, 1, CV_64F), by(n, 1, CV_64F);
    for (int i = 0; i < n; i++) {
        const cv::Point2d c = correctionOf(good[i]);
        double f[SLOP_TERMS];
        slopTerms(good[i].lastX, good[i].lastY, good[i].prevX, good[i].prevY, f);
        for (int j = 0; j < SLOP_TERMS; j++) {
            A.at<double>(i, j) = f[j];
        }
        bx.at<double>(i) = c.x;
        by.at<double>(i) = c.y;
    }
    cv::Mat fx, fy;
    cv::solve(A, bx, fx, cv::DECOMP_SVD);
    cv::solve(A, by, fy, cv::DECOMP_SVD);
    // how well the shots pin the terms down: a term the paths never varied apart from the others can't be told
    cv::Mat singular;
    cv::SVD::compute(A, singular, cv::SVD::NO_UV);
    const double conditioning = singular.at<double>(SLOP_TERMS - 1) / std::max(singular.at<double>(0), 1e-12);
    const cv::Mat ex = A * fx - bx, ey = A * fy - by;
    const double rx = std::sqrt(ex.dot(ex) / n), ry = std::sqrt(ey.dot(ey) / n);
    const auto term = [](const cv::Mat &f, int j) { return f.at<double>(j); };

    // ---- the results folder
    const QString folder = slopFolder;
    const QString tag = slopVerify ? QString("_pass%1").arg(slopPass) : QString();   // each confirm pass keeps its own files
    QFile csv(QDir(folder).filePath(QString("slop%1.csv").arg(tag)));
    if (csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "round,hour,via_hour,from_x_mm,from_y_mm,last_x_sign,last_y_sign,prev_x_sign,prev_y_sign,found,hole_board_x_mm,hole_board_y_mm,"
               "corner_board_x_mm,corner_board_y_mm,correction_x_mm,correction_y_mm,note\n";
        const auto mm = [](double v) { return QString::number(v, 'f', 4); };
        for (const SlopShot &s : slopShots) {
            const cv::Point2d d = hourDirection(s.hour) * slopDistance;
            const cv::Point2d c = s.ok ? correctionOf(s) : cv::Point2d(0, 0);
            out << s.round << ',' << s.hour << ',' << s.via << ',' << QString::number(d.x, 'f', 3) << ',' << QString::number(d.y, 'f', 3) << ','
                << s.lastX << ',' << s.lastY << ',' << s.prevX << ',' << s.prevY << ',' << (s.ok ? 1 : 0) << ',' << mm(s.hole.x) << ',' << mm(s.hole.y) << ',' << mm(s.corner.x) << ',' << mm(s.corner.y)
                << ',' << (s.ok ? mm(c.x) : QString()) << ',' << (s.ok ? mm(c.y) : QString()) << ",\""
                << QString(s.note).replace(QChar('"'), QChar('\'')) << "\"\n";
        }
    }

    // per hour: the average correction
    cv::Point2d hourMean[13];
    int hourCount[13] = { 0 };
    for (const SlopShot &s : good) {
        hourMean[s.hour] += correctionOf(s);
        hourCount[s.hour]++;
    }
    QJsonArray hours;
    double largest = 0.0;
    for (int h = 1; h <= 12; h++) {
        if (hourCount[h]) {
            hourMean[h] *= 1.0 / hourCount[h];
            hours.append(QJsonObject{ { "hour", h }, { "shots", hourCount[h] }, { "correction_x_y_mm", QJsonArray{ hourMean[h].x, hourMean[h].y } } });
        }
    }
    for (const SlopShot &s : good) {
        largest = std::max(largest, cv::norm(correctionOf(s)));
    }

    // ---- the polar plot: each shot's correction as a dot, coloured by the hour it came from, and each hour's average
    const int size = 900, mid = size / 2;
    const double plotRadius = 330.0;
    const double nice[] = { 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0 };
    double ring = nice[8];
    for (double r : nice) {
        if (std::ceil(std::max(largest, 1e-6) / r) <= 5) {
            ring = r;
            break;
        }
    }
    const int rings = std::max(1, (int)std::ceil(std::max(largest, 1e-6) / ring));
    const double pxPerMM = plotRadius / (rings * ring);
    cv::Mat plot(size, size, CV_8UC3, cv::Scalar(255, 255, 255));
    const auto toPlot = [&](const cv::Point2d &c) {   // +X right, +Y up
        return cv::Point((int)std::lround((mid + c.x * pxPerMM) * 16.0), (int)std::lround((mid - c.y * pxPerMM) * 16.0));
    };
    const cv::Scalar grid(215, 215, 215), ink(40, 40, 40);
    for (int r = 1; r <= rings; r++) {
        cv::circle(plot, cv::Point(mid * 16, mid * 16), (int)std::lround(r * ring * pxPerMM * 16.0), grid, 1, cv::LINE_AA, 4);
        cv::putText(plot, QString("%1").arg(r * ring, 0, 'g', 3).toStdString(), cv::Point(mid + (int)std::lround(r * ring * pxPerMM) + 3, mid - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    }
    cv::line(plot, cv::Point(mid - (int)plotRadius, mid), cv::Point(mid + (int)plotRadius, mid), grid, 1, cv::LINE_AA);
    cv::line(plot, cv::Point(mid, mid - (int)plotRadius), cv::Point(mid, mid + (int)plotRadius), grid, 1, cv::LINE_AA);
    cv::putText(plot, "+X", cv::Point(mid + (int)plotRadius + 6, mid + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
    cv::putText(plot, "+Y", cv::Point(mid - 10, mid - (int)plotRadius - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
    const auto hourColour = [](int h) {
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar((h - 1) * 15, 220, 200)), bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        const cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
        return cv::Scalar(v[0], v[1], v[2]);
    };
    for (int h = 1; h <= 12; h++) {       // the clock's hours, around the outside, where the hole came from
        const cv::Point2d d = hourDirection(h);
        const cv::Point at((int)std::lround(mid + d.x * (plotRadius + 45) - 8), (int)std::lround(mid - d.y * (plotRadius + 45) + 7));
        cv::putText(plot, QString::number(h).toStdString(), at, cv::FONT_HERSHEY_SIMPLEX, 0.7, hourColour(h), 2, cv::LINE_AA);
    }
    for (const SlopShot &s : good) {
        cv::circle(plot, toPlot(correctionOf(s)), 4 * 16, hourColour(s.hour), 1, cv::LINE_AA, 4);
    }
    for (int h = 1; h <= 12; h++) {
        if (hourCount[h]) {
            cv::line(plot, cv::Point(mid * 16, mid * 16), toPlot(hourMean[h]), hourColour(h), 2, cv::LINE_AA, 4);
            cv::circle(plot, toPlot(hourMean[h]), 7 * 16, hourColour(h), cv::FILLED, cv::LINE_AA, 4);
            const cv::Point p = toPlot(hourMean[h]);
            cv::putText(plot, QString::number(h).toStdString(), cv::Point(p.x / 16 + 9, p.y / 16 - 6), cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
        }
    }
    QStringList lines{
        slopVerify ? QString("CONFIRM %1, pass %2: slider correction still needed (mm) with the slop corrected, by hour").arg(slopMeasured).arg(slopPass)
                   : QString("Slider correction to add (mm), by the hour the hole came back from"),
        QString("%1 corner of scan %2, %3 mm approach, %4 of %5 shots found").arg(slopCorner, slopScan).arg(slopDistance, 0, 'f', 1).arg(n).arg(slopShots.count()),
        QString("terms (mm):        const    lastX    lastY    prevX    prevY      rms left"),
        QString("X correction:  %1 %2 %3 %4 %5   %6").arg(term(fx, 0), 8, 'f', 3).arg(term(fx, 1), 8, 'f', 3).arg(term(fx, 2), 8, 'f', 3)
            .arg(term(fx, 3), 8, 'f', 3).arg(term(fx, 4), 8, 'f', 3).arg(rx, 8, 'f', 3),
        QString("Y correction:  %1 %2 %3 %4 %5   %6").arg(term(fy, 0), 8, 'f', 3).arg(term(fy, 1), 8, 'f', 3).arg(term(fy, 2), 8, 'f', 3)
            .arg(term(fy, 3), 8, 'f', 3).arg(term(fy, 4), 8, 'f', 3).arg(ry, 8, 'f', 3),
        QString("X slop, +X vs -X last move: %1 mm; of it, from the move before: %2 mm   (terms told apart: %3)")
            .arg(2.0 * std::abs(term(fx, 1)), 0, 'f', 3).arg(2.0 * std::abs(term(fx, 3)), 0, 'f', 3)
            .arg(conditioning > 0.05 ? QString("well") : conditioning > 0.01 ? QString("poorly") : QString("no, vary the paths more")),
    };
    if (slopVerify) {
        lines << QString("hole's centre from the corner: %1 mm rms, %2 mm worst").arg(missRms, 0, 'f', 3).arg(missWorst, 0, 'f', 3);
    }
    for (int i = 0; i < lines.count(); i++) {
        cv::putText(plot, lines[i].toStdString(), cv::Point(14, 24 + 20 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
    }
    const QString plotPath = QDir(folder).filePath(QString("slop_polar%1.png").arg(tag));
    QImage(plot.data, plot.cols, plot.rows, (int)plot.step, QImage::Format_BGR888).save(plotPath);

    QJsonObject json;
    json["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["scan"] = slopScan;
    json["corner"] = slopCorner;
    json["spot_single_pair_mm"] = QJsonArray{ slopSingle, slopPair };
    json["approach_mm"] = slopDistance;
    json["rounds"] = slopRounds;
    json["shots"] = slopShots.count();
    json["found"] = n;
    json["clock"] = QString("in slider space: 12 o'clock is +Y (rail pair), 3 o'clock is +X (single rail); the hour the hole came back from");
    json["correction"] = QString("slider mm to add so the hole lands where the average puts it");
    const auto fitJson = [](const double *f, double rms) {
        QJsonObject o{ { "constant", f[0] }, { "per_last_x_sign", f[1] }, { "per_last_y_sign", f[2] }, { "per_prev_x_sign", f[3] }, { "per_prev_y_sign", f[4] } };
        if (rms >= 0.0) {
            o["rms_left"] = rms;
        }
        return o;
    };
    double solvedX[SLOP_TERMS], solvedY[SLOP_TERMS];
    for (int j = 0; j < SLOP_TERMS; j++) {
        solvedX[j] = term(fx, j);
        solvedY[j] = term(fy, j);
    }
    json["model"] = QString("correction = constant + per_last_x_sign * lastX + per_last_y_sign * lastY + per_prev_x_sign * prevX + per_prev_y_sign * prevY; "
                            "last and prev are the signs of each slider's last two moves");
    json["fit_x"] = fitJson(solvedX, rx);
    json["fit_y"] = fitJson(solvedY, ry);
    json["conditioning"] = conditioning;
    json["x_slop_mm"] = 2.0 * std::abs(term(fx, 1));
    json["y_slop_mm"] = 2.0 * std::abs(term(fy, 2));
    json["hours"] = hours;
    json["mode"] = slopVerify ? QString("check") : QString("measure");
    json["corner_index"] = (int)QStringList{ QString("top-left"), QString("top-right"), QString("bottom-right"), QString("bottom-left") }.indexOf(slopCorner);
    json["rim_radius_mm"] = slopVerify ? slopRimRadius : rimRadius;
    json["mean_hole_from_corner_board_mm"] = QJsonArray{ measuredMean.x, measuredMean.y };
    if (slopVerify) {
        json["checked"] = slopMeasured;
        json["applied_fit_x"] = fitJson(slopFitX, -1.0);
        json["applied_fit_y"] = fitJson(slopFitY, -1.0);
        json["miss_rms_mm"] = missRms;
        json["miss_worst_mm"] = missWorst;
    } else {
        // where the sliders put the corner in the middle of the hole, before the slop: the spot, less the average offset
        const cv::Point2d m = -measuredMean;
        json["corner_centred_single_pair_mm"] = QJsonArray{ slopSingle + (ax.x * m.y - m.x * ax.y) / det, slopPair + (m.x * ay.y - ay.x * m.y) / det };
    }
    QFile jsonFile(QDir(folder).filePath(QString("slop%1.json").arg(tag)));
    if (jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        jsonFile.write(QJsonDocument(json).toJson());
    }

    if (slopVerify) {
        // what this pass still needed, fitted the same way, is added to the estimate it used
        double change = 0.0;
        for (int i = 0; i < SLOP_TERMS; i++) {
            change = std::max(change, std::max(std::abs(term(fx, i)), std::abs(term(fy, i))));
        }
        slopHistory.append(QJsonObject{ { "pass", slopPass },
                                        { "applied_fit_x", fitJson(slopFitX, -1.0) },
                                        { "applied_fit_y", fitJson(slopFitY, -1.0) },
                                        { "found", n },
                                        { "miss_rms_mm", missRms },
                                        { "miss_worst_mm", missWorst },
                                        { "x_slop_left_mm", 2.0 * std::abs(term(fx, 1)) },
                                        { "conditioning", conditioning },
                                        { "largest_revision_mm", change } });
        for (int i = 0; i < SLOP_TERMS; i++) {
            slopFitX[i] += term(fx, i);
            slopFitY[i] += term(fy, i);
        }
        if (slopPass < slopPasses && change >= CONVERGED_MM) {
            slopPass++;                      // again, with the revised estimate
            slopShots.clear();
            slopStep = 0;
            slopLeg = 0;
            slopAtCentre = false;
            slopSeenValid = false;
            slopSeenHole = false;
            slopNextMove();
            return;
        }

        // the revised estimate, saved as a measurement so the next Confirm slop (and anything else) uses it
        const QString revised = QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) + QString("_slop");
        QDir().mkpath(QDir(scanBase).filePath(revised));
        QJsonObject r;
        r["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
        r["mode"] = QString("measure");
        r["revised_from"] = slopMeasured;
        r["confirmed_in"] = QFileInfo(folder).fileName();
        r["scan"] = slopScan;
        r["corner"] = slopCorner;
        r["corner_index"] = json["corner_index"];
        r["corner_centred_single_pair_mm"] = QJsonArray{ slopCentred.x, slopCentred.y };
        r["rim_radius_mm"] = slopRimRadius;
        r["approach_mm"] = slopDistance;
        r["clock"] = json["clock"];
        r["correction"] = QString("slider mm to add to the corner-centred position, from the signs of each slider's last two moves");
        r["model"] = json["model"];
        r["fit_x"] = fitJson(slopFitX, -1.0);
        r["fit_y"] = fitJson(slopFitY, -1.0);
        r["x_slop_mm"] = 2.0 * std::abs(slopFitX[1]);
        r["y_slop_mm"] = 2.0 * std::abs(slopFitY[2]);
        r["passes"] = slopHistory;
        QFile revisedFile(QDir(scanBase).filePath(revised + QString("/slop.json")));
        const bool saved = revisedFile.open(QIODevice::WriteOnly | QIODevice::Truncate) && revisedFile.write(QJsonDocument(r).toJson()) > 0;
        QFile historyFile(QDir(folder).filePath(QString("passes.json")));
        if (historyFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            historyFile.write(QJsonDocument(r).toJson());
        }

        QStringList steps;
        for (const QJsonValue &v : slopHistory) {
            const QJsonObject p = v.toObject();
            steps << QString("pass %1: %2 mm rms").arg(p["pass"].toInt()).arg(p["miss_rms_mm"].toDouble(), 0, 'f', 3);
        }
        const bool converged = change < CONVERGED_MM;
        endSlop(QString("Confirmed %1 over %2 pass%3 (%4); the hole's centre from the corner %5. %6 X slop now %7 mm. %8")
                    .arg(slopMeasured).arg(slopPass).arg(slopPass > 1 ? "es" : "").arg(steps.join(QString(", ")))
                    .arg(converged ? QString("stopped changing") : QString("was still changing by %1 mm").arg(change, 0, 'f', 3))
                    .arg(saved ? QString("Revised estimate saved as %1:").arg(revised) : QString("Couldn't save the revised estimate:"))
                    .arg(2.0 * std::abs(slopFitX[1]), 0, 'f', 3).arg(QString("Results are in %1").arg(QDir::toNativeSeparators(folder))));
        showSlopPlot(plotPath);
        return;
    }
    endSlop(QString("Slop measured from %1 of %2 approaches: X %3 mm between +X and -X last moves (%4 mm more from the move before), Y %5 mm "
                    "between +Y and -Y; %6 mm left unexplained%7. Results are in %8")
                .arg(n).arg(slopShots.count()).arg(2.0 * std::abs(term(fx, 1)), 0, 'f', 3).arg(2.0 * std::abs(term(fx, 3)), 0, 'f', 3)
                .arg(2.0 * std::abs(term(fy, 2)), 0, 'f', 3).arg(std::hypot(rx, ry), 0, 'f', 3)
                .arg(conditioning > 0.01 ? QString() : QString(" (the paths didn't vary enough to tell the last move from the one before: use 3 or more rounds)"))
                .arg(QDir::toNativeSeparators(folder)));
    showSlopPlot(plotPath);
}

// the plot in the app's own picture viewer, as the merged platen is shown
void LAUGantryMainWindow::showSlopPlot(const QString &path)
{
    const QImage plot(path);
    if (plot.isNull()) {
        cameraWidget->showMessage(QString("Couldn't open the slop plot %1").arg(QDir::toNativeSeparators(path)));
        return;
    }
    LAUImageInspectDialog *inspect = new LAUImageInspectDialog(plot, slopVerify ? QString("Slop Check") : QString("Slop"), this);
    inspect->setAttribute(Qt::WA_DeleteOnClose);
    inspect->show();
}

void LAUGantryMainWindow::endSlop(const QString &message)
{
    sloping = false;
    slopCancelling = false;
    slopWaitingFrame = false;
    if (slopTimer) {
        slopTimer->stop();
    }
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->lockScanActions(false);
    cameraWidget->showMessage(message);
}
