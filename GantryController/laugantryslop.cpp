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
// turned into the correction the sliders need, in slider mm, and plotted around a clock face; a fit of that
// correction to the sign of each slider's last move gives the slop.
//
#include "laugantrymainwindow.h"

#include <cmath>
#include <algorithm>
#include <QDir>
#include <QUrl>
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
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <opencv2/imgproc.hpp>

namespace
{
    const int SETTLE_MS = 500;
    const int FRAME_TIMEOUT_MS = 15000;
    const double MOVED_MM = 0.005;           // a slider that moved less than this didn't move
    const double CORNER_IN_MM = 2.0;         // the hole's centre, this far in from the corner along both edges

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
}

void LAUGantryMainWindow::onSlopRequested()
{
    const QString title = QString("Measure Slop");
    if (scanning || measuring || merging || returningHome || tuning || sloping || pageMoving || railWidget->isHoming()) {
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
            const cv::Point2d d = hourDirection(hour) * out;
            if (!slopGeometry.reachable(single + d.x, pair + d.y)) {
                return false;
            }
        }
        return true;
    };
    QSettings settings;
    QDialog dialog(this);
    dialog.setWindowTitle(title);
    QVBoxLayout *layout = new QVBoxLayout(&dialog);
    QLabel *label = new QLabel(QString("The sliders bring the i1's hole back over a corner of the page found in scan %1, with the corner %2 mm in from "
                                       "the hole's centre along both edges, from each of the twelve hours of a clock. Each time the camera finds the "
                                       "corner and the hole's rim, and how far the hole lands from the corner shows the slop.\n\n"
                                       "Take the i1 Pro out of its holder first so the camera sees the paper through the hole, and don't move the page.")
                                   .arg(slopScan).arg(CORNER_IN_MM, 0, 'f', 1));
    label->setWordWrap(true);
    layout->addWidget(label);
    QFormLayout *form = new QFormLayout();
    QComboBox *mode = new QComboBox();
    mode->addItem(QString("Measure the slop"));
    mode->addItem(measured.isEmpty() ? QString("Check a measurement (none yet for this scan)")
                                     : QString("Check measurement %1: corner centred, slop corrected").arg(measured));
    qobject_cast<QStandardItemModel *>(mode->model())->item(1)->setEnabled(!measured.isEmpty());
    form->addRow(QString("Mode"), mode);
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
    form->addRow(QString("Rounds of the clock"), rounds);
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
        const bool check = mode->currentIndex() == 1;
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
    connect(mode, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, refresh);
    refresh();
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    slopVerify = mode->currentIndex() == 1;
    const int k = slopVerify ? measuredCorner : corner->currentIndex();
    if (k < 0 || !fits(k, distance->value(), slopVerify)) {
        return;
    }
    if (slopVerify) {
        const QJsonObject fx = measurement["fit_x"].toObject(), fy = measurement["fit_y"].toObject();
        slopFitX[0] = fx["constant"].toDouble();
        slopFitX[1] = fx["per_last_x_sign"].toDouble();
        slopFitX[2] = fx["per_last_y_sign"].toDouble();
        slopFitY[0] = fy["constant"].toDouble();
        slopFitY[1] = fy["per_last_x_sign"].toDouble();
        slopFitY[2] = fy["per_last_y_sign"].toDouble();
        slopRimRadius = measurement["rim_radius_mm"].toDouble();
        slopMeasured = measured;
    }
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
    slopAtSingle = railWidget->reportedMM(0);
    slopAtPair = railWidget->reportedMM(2);
    slopLastX = 0;
    slopLastY = 0;
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
    if (!sloping || !homeDialog || raw16.empty() || !singleRail.isKnown() || !pairRail.isKnown()) {
        return;
    }
    const double ZOOM_HALF_MM = 9.0;
    const double now = guideClock.elapsed() / 1000.0;
    const double pair = pairRail.estimate(now), single = singleRail.estimate(now);
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const bool found = slopSeenValid && slopSeenHole;
    const cv::Point2d hole = (found ? slopSeenAperture : slopAperture) + ax * pair + ay * single;
    cv::Point2d centre, edge;
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

    // the i1's hole: a bullseye where its rim was found, else a grey ring where it is expected
    if (found) {
        const double r = (slopSeenRadius > 0.0) ? slopSeenRadius : 4.9;
        for (double f : { 1.0, 2.0 / 3.0, 1.0 / 3.0 }) {
            drawCircle(hole, r * f, green, f == 1.0 ? 2 : 1);
        }
        cv::Point c;
        if (project(hole, &c)) {
            cv::circle(colour, c, 3 * 16, green, cv::FILLED, cv::LINE_AA, 4);
        }
        drawLine(hole - cv::Point2d(r, 0), hole + cv::Point2d(r, 0), green, 1);
        drawLine(hole - cv::Point2d(0, r), hole + cv::Point2d(0, r), green, 1);
        if (slopSeen.atValid) {
            cv::Point k;
            if (project(slopSeen.at, &k) && project(hole, &c)) {
                cv::line(colour, c, k, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 4);
            }
            const cv::Point2d d = hole - slopSeen.at;
            cv::putText(colour, QString("hole from corner: %1, %2 mm").arg(d.x, 0, 'f', 3).arg(d.y, 0, 'f', 3).toStdString(), cv::Point(10, 46),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1, cv::LINE_AA);
        }
    } else {
        drawCircle(hole, 4.9, grey160, 1);
    }

    QString title = QString("%1 corner").arg(slopCorner);
    if (!slopShots.isEmpty()) {
        title += QString(", last stop from %1 o'clock").arg(slopShots.last().hour);
    }
    cv::putText(colour, title.toStdString(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    if (slopSeenValid && !slopSeenHole && !slopSeenWhy.isEmpty()) {
        cv::putText(colour, slopSeenWhy.left(64).toStdString(), cv::Point(10, 66), cv::FONT_HERSHEY_SIMPLEX, 0.4, cyan, 1, cv::LINE_AA);
    }
    cv::putText(colour, "green bullseye: i1 hole   magenta: corner   cyan: page edges", cv::Point(10, size - 28), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(colour, QString("%1 mm across").arg(2 * ZOOM_HALF_MM, 0, 'f', 0).toStdString(), cv::Point(10, size - 10), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    homeDialog->setZoom(QImage(colour.data, colour.cols, colour.rows, (int)colour.step, QImage::Format_BGR888).copy());
}

// out to the hour's start, or back in to the spot
void LAUGantryMainWindow::slopNextMove()
{
    const int total = 12 * slopRounds;
    const int round = slopStep / 12, hour = slopStep % 12 + 1;
    double single = slopSingle, pair = slopPair;
    if (!slopAtCentre) {
        const cv::Point2d d = hourDirection(hour) * slopDistance;
        single += d.x;
        pair += d.y;
        homeDialog->setMessage(QString("Round %1 of %2: out to %3 o'clock...").arg(round + 1).arg(slopRounds).arg(hour));
    } else if (slopVerify) {
        // the slop this approach will leave, from the sign each slider's last move will have, added in advance; the
        // correction can flip a small move's sign (12 and 6 o'clock in X), so settle on signs that agree with it
        int sx = signOf(single - slopAtSingle), sy = signOf(pair - slopAtPair);
        for (int iteration = 0; iteration < 3; iteration++) {
            const int lx = sx ? sx : slopLastX, ly = sy ? sy : slopLastY;
            single = slopSingle + slopFitX[0] + slopFitX[1] * lx + slopFitX[2] * ly;
            pair = slopPair + slopFitY[0] + slopFitY[1] * lx + slopFitY[2] * ly;
            sx = signOf(single - slopAtSingle);
            sy = signOf(pair - slopAtPair);
        }
        homeDialog->setMessage(QString("Round %1 of %2: onto the corner from %3 o'clock, %4 mm X and %5 mm Y added for the slop...")
                                   .arg(round + 1).arg(slopRounds).arg(hour).arg(single - slopSingle, 0, 'f', 3).arg(pair - slopPair, 0, 'f', 3));
    } else {
        homeDialog->setMessage(QString("Round %1 of %2: back to the spot from %3 o'clock...").arg(round + 1).arg(slopRounds).arg(hour));
    }
    homeDialog->setProgress(slopStep, total);
    const int sx = signOf(single - slopAtSingle), sy = signOf(pair - slopAtPair);
    slopLastX = sx ? sx : slopLastX;
    slopLastY = sy ? sy : slopLastY;
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
        slopAtCentre = true;
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

    // the correction against the sign of each slider's last move: c = c0 + cx * lastX + cy * lastY, for X and for Y
    const int n = good.count();
    cv::Mat A(n, 3, CV_64F), bx(n, 1, CV_64F), by(n, 1, CV_64F);
    for (int i = 0; i < n; i++) {
        const cv::Point2d c = correctionOf(good[i]);
        A.at<double>(i, 0) = 1.0;
        A.at<double>(i, 1) = good[i].lastX;
        A.at<double>(i, 2) = good[i].lastY;
        bx.at<double>(i) = c.x;
        by.at<double>(i) = c.y;
    }
    cv::Mat fx, fy;
    cv::solve(A, bx, fx, cv::DECOMP_SVD);
    cv::solve(A, by, fy, cv::DECOMP_SVD);
    double rx = 0.0, ry = 0.0;
    for (int i = 0; i < n; i++) {
        const cv::Point2d c = correctionOf(good[i]);
        const double ex = c.x - (fx.at<double>(0) + fx.at<double>(1) * good[i].lastX + fx.at<double>(2) * good[i].lastY);
        const double ey = c.y - (fy.at<double>(0) + fy.at<double>(1) * good[i].lastX + fy.at<double>(2) * good[i].lastY);
        rx += ex * ex / n;
        ry += ey * ey / n;
    }
    rx = std::sqrt(rx);
    ry = std::sqrt(ry);

    // ---- the results folder
    const QString folder = slopFolder;
    QFile csv(QDir(folder).filePath(QString("slop.csv")));
    if (csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "round,hour,from_x_mm,from_y_mm,last_x_sign,last_y_sign,found,hole_board_x_mm,hole_board_y_mm,corner_board_x_mm,corner_board_y_mm,"
               "correction_x_mm,correction_y_mm,note\n";
        const auto mm = [](double v) { return QString::number(v, 'f', 4); };
        for (const SlopShot &s : slopShots) {
            const cv::Point2d d = hourDirection(s.hour) * slopDistance;
            const cv::Point2d c = s.ok ? correctionOf(s) : cv::Point2d(0, 0);
            out << s.round << ',' << s.hour << ',' << QString::number(d.x, 'f', 3) << ',' << QString::number(d.y, 'f', 3) << ',' << s.lastX << ','
                << s.lastY << ',' << (s.ok ? 1 : 0) << ',' << mm(s.hole.x) << ',' << mm(s.hole.y) << ',' << mm(s.corner.x) << ',' << mm(s.corner.y)
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
        slopVerify ? QString("CHECK of %1: slider correction still needed (mm) with the slop corrected, by hour").arg(slopMeasured)
                   : QString("Slider correction to add (mm), by the hour the hole came back from"),
        QString("%1 corner of scan %2, %3 mm approach, %4 of %5 shots found").arg(slopCorner, slopScan).arg(slopDistance, 0, 'f', 1).arg(n).arg(slopShots.count()),
        QString("X: %1 %2 %3 x lastX %4 %5 x lastY   (rms left %6)").arg(fx.at<double>(0), 0, 'f', 3).arg(fx.at<double>(1) < 0 ? "-" : "+")
            .arg(std::abs(fx.at<double>(1)), 0, 'f', 3).arg(fx.at<double>(2) < 0 ? "-" : "+").arg(std::abs(fx.at<double>(2)), 0, 'f', 3).arg(rx, 0, 'f', 3),
        QString("Y: %1 %2 %3 x lastX %4 %5 x lastY   (rms left %6)").arg(fy.at<double>(0), 0, 'f', 3).arg(fy.at<double>(1) < 0 ? "-" : "+")
            .arg(std::abs(fy.at<double>(1)), 0, 'f', 3).arg(fy.at<double>(2) < 0 ? "-" : "+").arg(std::abs(fy.at<double>(2)), 0, 'f', 3).arg(ry, 0, 'f', 3),
        QString("slop between +X and -X approaches: %1 mm").arg(2.0 * std::abs(fx.at<double>(1)), 0, 'f', 3),
    };
    if (slopVerify) {
        lines << QString("hole's centre from the corner: %1 mm rms, %2 mm worst").arg(missRms, 0, 'f', 3).arg(missWorst, 0, 'f', 3);
    }
    for (int i = 0; i < lines.count(); i++) {
        cv::putText(plot, lines[i].toStdString(), cv::Point(14, 24 + 20 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv::LINE_AA);
    }
    const QString plotPath = QDir(folder).filePath(QString("slop_polar.png"));
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
    json["fit_x"] = QJsonObject{ { "constant", fx.at<double>(0) }, { "per_last_x_sign", fx.at<double>(1) }, { "per_last_y_sign", fx.at<double>(2) }, { "rms_left", rx } };
    json["fit_y"] = QJsonObject{ { "constant", fy.at<double>(0) }, { "per_last_x_sign", fy.at<double>(1) }, { "per_last_y_sign", fy.at<double>(2) }, { "rms_left", ry } };
    json["x_slop_mm"] = 2.0 * std::abs(fx.at<double>(1));
    json["y_slop_mm"] = 2.0 * std::abs(fy.at<double>(2));
    json["hours"] = hours;
    json["mode"] = slopVerify ? QString("check") : QString("measure");
    json["corner_index"] = (int)QStringList{ QString("top-left"), QString("top-right"), QString("bottom-right"), QString("bottom-left") }.indexOf(slopCorner);
    json["rim_radius_mm"] = slopVerify ? slopRimRadius : rimRadius;
    json["mean_hole_from_corner_board_mm"] = QJsonArray{ measuredMean.x, measuredMean.y };
    if (slopVerify) {
        json["checked"] = slopMeasured;
        json["applied_fit_x"] = QJsonArray{ slopFitX[0], slopFitX[1], slopFitX[2] };
        json["applied_fit_y"] = QJsonArray{ slopFitY[0], slopFitY[1], slopFitY[2] };
        json["miss_rms_mm"] = missRms;
        json["miss_worst_mm"] = missWorst;
    } else {
        // where the sliders put the corner in the middle of the hole, before the slop: the spot, less the average offset
        const cv::Point2d m = -measuredMean;
        json["corner_centred_single_pair_mm"] = QJsonArray{ slopSingle + (ax.x * m.y - m.x * ax.y) / det, slopPair + (m.x * ay.y - ay.x * m.y) / det };
    }
    QFile jsonFile(QDir(folder).filePath(QString("slop.json")));
    if (jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        jsonFile.write(QJsonDocument(json).toJson());
    }

    if (slopVerify) {
        endSlop(QString("Checked %1: with the slop corrected, the hole's centre landed %2 mm rms (worst %3 mm) from the corner over %4 of %5 "
                        "approaches, with %6 mm of X slop left. Results are in %7")
                    .arg(slopMeasured).arg(missRms, 0, 'f', 3).arg(missWorst, 0, 'f', 3).arg(n).arg(slopShots.count())
                    .arg(2.0 * std::abs(fx.at<double>(1)), 0, 'f', 3).arg(QDir::toNativeSeparators(folder)));
        QDesktopServices::openUrl(QUrl::fromLocalFile(plotPath));
        return;
    }
    endSlop(QString("Slop measured from %1 of %2 approaches: %3 mm in X between +X and -X moves, %4 mm in Y between +Y and -Y; %5 mm left unexplained. "
                    "Results are in %6")
                .arg(n).arg(slopShots.count()).arg(2.0 * std::abs(fx.at<double>(1)), 0, 'f', 3).arg(2.0 * std::abs(fy.at<double>(2)), 0, 'f', 3)
                .arg(std::hypot(rx, ry), 0, 'f', 3).arg(QDir::toNativeSeparators(folder)));
    QDesktopServices::openUrl(QUrl::fromLocalFile(plotPath));
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
