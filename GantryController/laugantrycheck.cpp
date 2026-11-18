/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Check page corners: after Fine-tune page corners, the sliders go back to the four corners it found (page_sliders.json),
// round after round, every move with the same one-way approach as the fine-tune and the patches.  At each visit the
// hole is first seen on the plain paper beside the corner -- where the hole really is, then -- and over the corner the
// camera finds the corner and measures how far it is from the hole's centre.  The misses say how well the fine-tune
// put each corner in the hole, and their spread over the rounds how well the sliders return to it.
//
#include "laugantrymainwindow.h"

#include <cmath>
#include <QDir>
#include <QFile>
#include <QTimer>
#include <QMap>
#include <QDateTime>
#include <QSettings>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QMessageBox>
#include <QTextStream>
#include <QInputDialog>
#include <QJsonDocument>
#include <QImage>
#include <opencv2/imgproc.hpp>

namespace
{
    const double CHECK_INSETS_MM[] = { 12.0, 20.0, 30.0 };   // plain paper in from both edges, as the fine-tune uses
    const int CHECK_SETTLE_MS = 500;
    const int CHECK_FRAME_TIMEOUT_MS = 15000;
    const double CHECK_ZOOM_HALF_MM = 8.0;   // the close-up around the hole, each way
    const double CHECK_ZOOM_RES = 0.04;      // mm a pixel
    const double HOLE_RADIUS_MM = 4.9;       // the i1 holder's 9.8 mm hole
    const double FALLBACK_WINDOW_MM = 4.5;   // the paper seen through the hole, when it couldn't be seen on plain paper

    double rms(const QList<cv::Point2d> &points, cv::Point2d *mean)
    {
        *mean = cv::Point2d(0, 0);
        if (points.isEmpty()) {
            return 0.0;
        }
        for (const cv::Point2d &p : points) {
            *mean += p * (1.0 / points.count());
        }
        double sum = 0.0;
        for (const cv::Point2d &p : points) {
            const cv::Point2d d = p - *mean;
            sum += d.dot(d);
        }
        return std::sqrt(sum / points.count());
    }
}

void LAUGantryMainWindow::onCornerCheckRequested()
{
    const QString title = QString("Check Page Corners");
    if (scanning || measuring || merging || returningHome || tuning || edgeSloping || pageMoving || checking || railWidget->isHoming()) {
        cameraWidget->showMessage(QString("Wait until the sliders and the merge are finished, then try again."));
        return;
    }
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        QMessageBox::information(this, title, QString("Tick Enable Sliders first, so the sliders can move to the page's corners."));
        return;
    }

    // the newest fine-tuned page
    QDir base(scanBase);
    checkFolder.clear();
    for (const QString &name : base.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        if (QFile::exists(base.filePath(name + QString("/mosaic/page_sliders.json")))) {
            checkFolder = base.filePath(name + QString("/mosaic"));
            break;
        }
    }
    if (checkFolder.isEmpty()) {
        QMessageBox::information(this, title, QString("No page has been fine-tuned yet. Scan the platen image, then choose Fine-tune page corners."));
        return;
    }
    const auto readJson = [this](const QString &name) {
        QFile file(QDir(checkFolder).filePath(name));
        return file.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject();
    };
    const QJsonObject sliders = readJson(QString("page_sliders.json")), page = readJson(QString("page.json")), tuned = readJson(QString("corner_tune.json"));
    const QJsonArray sliderCorners = sliders["corners"].toArray(), boardCorners = page["corners_board_mm"].toArray(), h = sliders["page_to_single_pair"].toArray();
    const QJsonArray size = sliders["page_size_mm"].toArray();
    if (sliderCorners.count() != 4 || boardCorners.count() != 4 || h.count() != 3 || size.count() != 2) {
        QMessageBox::warning(this, title, QString("The fine-tuned page in %1 couldn't be read. Fine-tune the page corners again.").arg(QDir::toNativeSeparators(checkFolder)));
        return;
    }
    holeView.load(QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString());
    if (!holeView.isValid()) {
        QMessageBox::warning(this, title, QString("The platen calibration can't be read, so the camera can't see the hole. Run Calibrate platen with the i1 off."));
        return;
    }
    cv::Matx33d H;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            H(r, c) = h[r].toArray().at(c).toDouble();
        }
    }
    const double width = size[0].toDouble(), height = size[1].toDouble();
    double sLow = 0.0, sHigh = 0.0, pLow = 0.0, pHigh = 0.0;
    const bool limits = railWidget->limitRange(0, &sLow, &sHigh) && railWidget->limitRange(2, &pLow, &pHigh);

    // the hole each corner was centred with, else the mean of what the fine-tune saw
    cv::Point2d meanSeen = holeView.calibratedAperture();
    const QJsonArray seen = tuned["hole_seen_at_rails_zero_mm"].toArray();
    if (!seen.isEmpty()) {
        meanSeen = cv::Point2d(0, 0);
        for (const QJsonValue &v : seen) {
            meanSeen += cv::Point2d(v.toArray().at(0).toDouble(), v.toArray().at(1).toDouble()) * (1.0 / seen.count());
        }
    }
    QMap<QString, cv::Point2d> tunedHoles;
    for (const QJsonValue &v : tuned["corners"].toArray()) {
        const QJsonArray a = v.toObject()["hole_beside_it_at_rails_zero_mm"].toArray();
        if (a.count() == 2) {
            tunedHoles[v.toObject()["corner"].toString()] = cv::Point2d(a[0].toDouble(), a[1].toDouble());
        }
    }

    // page mm of each corner, clockwise from the top-left, and the way into the page from it
    const cv::Point2d pageAt[4] = { { 0.0, 0.0 }, { width, 0.0 }, { width, height }, { 0.0, height } };
    const cv::Point2d inward[4] = { { 1.0, 1.0 }, { -1.0, 1.0 }, { -1.0, -1.0 }, { 1.0, -1.0 } };
    checkCorners.clear();
    for (int k = 0; k < 4; k++) {
        const QJsonObject o = sliderCorners[k].toObject();
        CheckCorner c;
        c.name = o["corner"].toString();
        c.single = o["single_pair_mm"].toArray().at(0).toDouble();
        c.pair = o["single_pair_mm"].toArray().at(1).toDouble();
        c.board = cv::Point2d(boardCorners[k].toArray().at(0).toDouble(), boardCorners[k].toArray().at(1).toDouble());
        c.tunedHoleKnown = tunedHoles.contains(c.name);
        c.tunedHole = c.tunedHoleKnown ? tunedHoles[c.name] : meanSeen;
        for (double inset : CHECK_INSETS_MM) {
            const cv::Point2d p = pageAt[k] + inward[k] * inset;
            const cv::Vec3d r = H * cv::Vec3d(p.x, p.y, 1.0);
            const double single = r[0] / r[2], pair = r[1] / r[2];
            if (!limits || (single >= sLow && single <= sHigh && pair >= pLow && pair <= pHigh)) {
                c.insets << cv::Point2d(single, pair);
            }
        }
        if (limits && (c.single < sLow || c.single > sHigh || c.pair < pLow || c.pair > pHigh)) {
            QMessageBox::information(this, title, QString("The page's %1 corner (X %2 mm, Y %3 mm) is outside the limit sliders. Move them out and try again.")
                                                      .arg(c.name).arg(c.single, 0, 'f', 2).arg(c.pair, 0, 'f', 2));
            return;
        }
        checkCorners << c;
    }

    QSettings settings;
    bool ok = false;
    const int rounds = QInputDialog::getInt(this, title,
                                            QString("The sliders will visit the page's four corners, as Fine-tune page corners left them, this many times. "
                                                    "At each visit the camera sees the hole on plain paper beside the corner, then measures how far the "
                                                    "corner is from the hole's centre.\n\nTake the i1 Pro out of its holder first. Rounds:"),
                                            settings.value(QString("LAUGantryMainWindow::checkRounds"), 3).toInt(), 1, 20, 1, &ok);
    if (!ok) {
        return;
    }
    settings.setValue(QString("LAUGantryMainWindow::checkRounds"), rounds);

    if (!checkTimer) {
        checkTimer = new QTimer(this);
        checkTimer->setSingleShot(true);
        connect(checkTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onCheckTimer);
    }
    checking = true;
    checkCancelling = false;
    checkWaitingFrame = false;
    checkRounds = rounds;
    checkRound = 0;
    checkIndex = 0;
    checkVisits.clear();
    resetSlopTracking();
    cameraWidget->lockScanActions(true);
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(title);
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, &LAUGantryMainWindow::onCheckAbort);
    homeDialog->open();
    checkNextVisit();
}

// to the plain paper beside the corner of this visit
void LAUGantryMainWindow::checkNextVisit()
{
    CheckCorner &c = checkCorners[checkIndex];
    checkVisit = CheckVisit();
    checkVisit.round = checkRound;
    checkVisit.corner = checkIndex;
    checkWindow.release();
    homeDialog->setMessage(QString("Round %1 of %2: the page's %3 corner. Seeing the hole beside it...").arg(checkRound + 1).arg(checkRounds).arg(c.name));
    homeDialog->setProgress(checkRound * 4 + checkIndex, checkRounds * 4);
    if (c.insets.isEmpty()) {
        checkVisit.holeWhy = QString("no plain paper beside it is within the limit sliders");
        checkStage = CheckToCorner;
        moveRailsSlop(c.single, c.pair, QString("round %1 %2: corner").arg(checkRound + 1).arg(c.name));
        return;
    }
    checkStage = CheckToInset;
    checkInsetTry = std::min(c.insetUsed, (int)c.insets.count() - 1);
    checkVisit.inset = checkInsetTry;
    moveRailsSlop(c.insets[checkInsetTry].x, c.insets[checkInsetTry].y, QString("round %1 %2: plain paper %3").arg(checkRound + 1).arg(c.name).arg(checkInsetTry + 1));
}

void LAUGantryMainWindow::onCheckMoveFinished(bool ok, const QString &message)
{
    if (checkCancelling) {
        finishCheck(QString("Checking was aborted."));
        return;
    }
    if (!ok) {
        finishCheck(QString("The sliders stopped: %1").arg(message));
        return;
    }
    checkTimer->start(CHECK_SETTLE_MS);      // then a frame
}

void LAUGantryMainWindow::onCheckTimer()
{
    if (!checking) {
        return;
    }
    if (checkWaitingFrame) {
        checkWaitingFrame = false;
        finishCheck(QString("The camera stopped delivering frames."));
        return;
    }
    checkWaitingFrame = true;
    checkTimer->start(CHECK_FRAME_TIMEOUT_MS);
    cameraWidget->requestScanFrame();
}

void LAUGantryMainWindow::onCheckFrame(const cv::Mat &raw16)
{
    if (!checking || !checkWaitingFrame) {
        return;
    }
    checkWaitingFrame = false;
    checkTimer->stop();
    if (checkCancelling) {
        finishCheck(QString("Checking was aborted."));
        return;
    }
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);      // Mono10 -> 8 bits
    CheckCorner &c = checkCorners[checkIndex];
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const double single = railWidget->reportedMM(0), pair = railWidget->reportedMM(2);

    if (checkStage == CheckToInset) {
        LAUHoleView::Hole hole;
        QString why;
        if (holeView.findHole(gray, pair, single, c.tunedHole + ax * pair + ay * single, &hole, &why)) {
            checkVisit.holeFound = true;
            checkVisit.inset = checkInsetTry;
            checkVisit.hole = hole.centre - ax * pair - ay * single;
            checkWindow = hole.window;
            c.insetUsed = checkInsetTry;
            showCheckZoom(gray, hole.centre, nullptr, QString("hole beside the %1 corner").arg(c.name));
        } else if (checkInsetTry + 1 < c.insets.count()) {
            checkInsetTry++;                 // print under the hole, most likely: further in
            moveRailsSlop(c.insets[checkInsetTry].x, c.insets[checkInsetTry].y,
                          QString("round %1 %2: plain paper %3").arg(checkRound + 1).arg(c.name).arg(checkInsetTry + 1));
            return;
        } else {
            checkVisit.holeWhy = why;
            showCheckZoom(gray, c.tunedHole + ax * pair + ay * single, nullptr, QString("hole not seen: %1").arg(why).left(70));
        }
        checkStage = CheckToCorner;
        homeDialog->setMessage(QString("Round %1 of %2: the page's %3 corner. Measuring it in the hole...").arg(checkRound + 1).arg(checkRounds).arg(c.name));
        moveRailsSlop(c.single, c.pair, QString("round %1 %2: corner").arg(checkRound + 1).arg(c.name));
        return;
    }

    // over the corner: the hole as seen beside it on this visit (else as the fine-tune had it), and the corner
    checkVisit.single = single;
    checkVisit.pair = pair;
    const cv::Point2d aperture = checkVisit.holeFound ? checkVisit.hole : c.tunedHole;
    const cv::Point2d hole = aperture + ax * pair + ay * single;
    if (checkWindow.empty()) {
        const int n = (int)std::lround(2.0 * LAUHoleView::WINDOW_HALF / LAUHoleView::RES) + 1;
        checkWindow = cv::Mat::zeros(n, n, CV_8U);
        cv::circle(checkWindow, cv::Point(n / 2, n / 2), (int)std::lround(FALLBACK_WINDOW_MM / LAUHoleView::RES), cv::Scalar(255), cv::FILLED);
    }
    const cv::Point2d along1 = checkCorners[(checkIndex + 1) % 4].board - c.board, along2 = checkCorners[(checkIndex + 3) % 4].board - c.board;
    LAUHoleView::Corner found;
    QString why;
    checkVisit.found = holeView.findCorner(gray, pair, single, hole, checkWindow, c.board, along1, along2, &found, &why);
    if (checkVisit.found) {
        checkVisit.miss = found.at - hole;
        checkVisit.missTuned = found.at - (c.tunedHole + ax * pair + ay * single);
        const double det = ax.x * ay.y - ay.x * ax.y;
        checkVisit.missPair = (checkVisit.miss.x * ay.y - ay.x * checkVisit.miss.y) / det;
        checkVisit.missSingle = (ax.x * checkVisit.miss.y - checkVisit.miss.x * ax.y) / det;
    } else {
        checkVisit.why = why;
    }
    showCheckZoom(gray, hole, &found,
                  checkVisit.found ? QString("%1: corner %2 mm from the hole's centre").arg(c.name).arg(cv::norm(checkVisit.miss), 0, 'f', 3)
                                   : QString("%1: %2").arg(c.name, why).left(70));
    checkVisits << checkVisit;

    if (++checkIndex >= checkCorners.count()) {
        checkIndex = 0;
        if (++checkRound >= checkRounds) {
            finishCheck(QString());
            return;
        }
    }
    checkNextVisit();
}

// the paper around the hole, square-on, with the hole as a bullseye and what was found of the corner
void LAUGantryMainWindow::showCheckZoom(const cv::Mat &gray, const cv::Point2d &hole, const LAUHoleView::Corner *corner, const QString &text)
{
    if (!homeDialog) {
        return;
    }
    const double pair = railWidget->reportedMM(2), single = railWidget->reportedMM(0);
    cv::Mat L = holeView.ortho(gray, pair, single, hole, CHECK_ZOOM_HALF_MM, CHECK_ZOOM_RES), grey, colour;
    L.convertTo(grey, CV_8U);
    cv::cvtColor(grey, colour, cv::COLOR_GRAY2BGR);
    const int n = colour.cols;
    const auto px = [&](const cv::Point2d &board) {
        return cv::Point((int)std::lround((board.x - hole.x + CHECK_ZOOM_HALF_MM) / CHECK_ZOOM_RES),
                         (int)std::lround((board.y - hole.y + CHECK_ZOOM_HALF_MM) / CHECK_ZOOM_RES));
    };
    const double perMM = 1.0 / CHECK_ZOOM_RES;
    const cv::Point mid(n / 2, n / 2);
    // the bullseye: the hole's outline, rings every mm, and its centre
    const cv::Scalar green(0, 220, 0), magenta(255, 0, 255), cyan(255, 255, 0);
    cv::circle(colour, mid, (int)std::lround(HOLE_RADIUS_MM * perMM), green, 2, cv::LINE_AA);
    for (int r = 1; r <= 3; r++) {
        cv::circle(colour, mid, (int)std::lround(r * perMM), green, 1, cv::LINE_AA);
    }
    cv::line(colour, mid - cv::Point(12, 0), mid + cv::Point(12, 0), green, 1, cv::LINE_AA);
    cv::line(colour, mid - cv::Point(0, 12), mid + cv::Point(0, 12), green, 1, cv::LINE_AA);
    if (corner) {
        for (int e = 0; e < 2; e++) {
            for (const cv::Point2d &p : corner->edgePoints[e]) {
                cv::circle(colour, px(p), 1, cyan, cv::FILLED);
            }
            if (corner->edgeFitted[e]) {
                cv::line(colour, px(corner->edgeAt[e] - corner->edgeDir[e] * 20.0), px(corner->edgeAt[e] + corner->edgeDir[e] * 20.0), cyan, 1, cv::LINE_AA);
            }
        }
        if (corner->atValid) {
            const cv::Point at = px(corner->at);
            cv::line(colour, at - cv::Point(10, 10), at + cv::Point(10, 10), magenta, 2, cv::LINE_AA);
            cv::line(colour, at - cv::Point(10, -10), at + cv::Point(10, -10), magenta, 2, cv::LINE_AA);
            cv::line(colour, mid, at, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
    }
    cv::putText(colour, text.toStdString(), cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(colour, QString("green hole (rings 1 mm)  cyan edges  magenta corner  %1 mm across").arg(2 * CHECK_ZOOM_HALF_MM, 0, 'f', 0).toStdString(),
                cv::Point(10, n - 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    homeDialog->setZoom(QImage(colour.data, colour.cols, colour.rows, (int)colour.step, QImage::Format_BGR888).copy());
}

void LAUGantryMainWindow::onCheckAbort()
{
    if (!checking) {
        return;
    }
    checkCancelling = true;
    if (railWidget->isHoming()) {
        railWidget->abortHome();             // onCheckMoveFinished finishes up
        return;
    }
    if (checkWaitingFrame) {
        return;                              // onCheckFrame finishes up
    }
    checkTimer->stop();
    finishCheck(QString("Checking was aborted."));
}

// what the rounds say, per corner: the log, a summary, and a table for the user
void LAUGantryMainWindow::finishCheck(const QString &stopped)
{
    const QString stamp = QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss"));
    QFile csv(QDir(checkFolder).filePath(QString("corner_check_%1.csv").arg(stamp)));
    if (!checkVisits.isEmpty() && csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "# the corner less the hole's centre, board mm; miss_single/pair: the slider move that would close it. hole: seen on plain "
               "paper beside the corner on this visit, rails at zero; tuned: the hole the fine-tune centred the corner with\n";
        out << "visit,round,corner,hole_found,inset,hole_x_mm,hole_y_mm,reported_single_mm,reported_pair_mm,corner_found,miss_x_mm,miss_y_mm,miss_mm,"
               "miss_single_mm,miss_pair_mm,miss_vs_tuned_hole_mm,note\n";
        for (int k = 0; k < checkVisits.count(); k++) {
            const CheckVisit &v = checkVisits[k];
            out << (k + 1) << "," << (v.round + 1) << "," << checkCorners[v.corner].name << "," << (v.holeFound ? 1 : 0) << "," << (v.inset + 1) << ",";
            out << (v.holeFound ? QString("%1,%2").arg(v.hole.x, 0, 'f', 4).arg(v.hole.y, 0, 'f', 4) : QString(",")) << ",";
            out << QString::number(v.single, 'f', 4) << "," << QString::number(v.pair, 'f', 4) << "," << (v.found ? 1 : 0) << ",";
            if (v.found) {
                out << QString::number(v.miss.x, 'f', 4) << "," << QString::number(v.miss.y, 'f', 4) << "," << QString::number(cv::norm(v.miss), 'f', 4) << ","
                    << QString::number(v.missSingle, 'f', 4) << "," << QString::number(v.missPair, 'f', 4) << "," << QString::number(cv::norm(v.missTuned), 'f', 4) << ",";
            } else {
                out << ",,,,,,";
            }
            const QString note = !v.why.isEmpty() ? v.why : v.holeWhy;
            out << "\"" << QString(note).replace(QChar('"'), QChar('\'')) << "\"\n";
        }
        csv.close();
    }

    // per corner: the mean miss (how well the fine-tune centred it), its spread (how well the sliders return), and
    // the spread of the hole's own sightings beside it
    QString rows;
    QJsonArray json;
    double worstMean = 0.0, worstSpread = 0.0;
    for (int k = 0; k < checkCorners.count(); k++) {
        QList<cv::Point2d> misses, missesTuned, holes;
        QList<cv::Point2d> sliderMisses;
        int visits = 0;
        for (const CheckVisit &v : checkVisits) {
            if (v.corner != k) {
                continue;
            }
            visits++;
            if (v.found) {
                misses << v.miss;
                missesTuned << v.missTuned;
                sliderMisses << cv::Point2d(v.missSingle, v.missPair);
            }
            if (v.holeFound) {
                holes << v.hole;
            }
        }
        if (visits == 0) {
            continue;
        }
        cv::Point2d meanMiss, meanTuned, meanHole, meanSlider;
        const double spread = rms(misses, &meanMiss), holeSpread = rms(holes, &meanHole);
        rms(missesTuned, &meanTuned);
        rms(sliderMisses, &meanSlider);
        double worst = 0.0;
        for (const cv::Point2d &m : misses) {
            worst = std::max(worst, cv::norm(m));
        }
        QJsonObject o;
        o["corner"] = checkCorners[k].name;
        o["visits"] = visits;
        o["corner_found"] = (int)misses.count();
        o["hole_found"] = (int)holes.count();
        if (!misses.isEmpty()) {
            worstMean = std::max(worstMean, cv::norm(meanMiss));
            worstSpread = std::max(worstSpread, spread);
            o["mean_miss_board_mm"] = QJsonArray{ meanMiss.x, meanMiss.y };
            o["mean_miss_mm"] = cv::norm(meanMiss);
            o["mean_miss_single_pair_mm"] = QJsonArray{ meanSlider.x, meanSlider.y };
            o["worst_miss_mm"] = worst;
            o["repeatability_rms_mm"] = spread;
            o["mean_miss_vs_tuned_hole_mm"] = cv::norm(meanTuned);
        }
        if (!holes.isEmpty()) {
            o["hole_rms_mm"] = holeSpread;
            o["hole_mean_from_tuned_hole_mm"] = cv::norm(meanHole - checkCorners[k].tunedHole);
        }
        json.append(o);
        rows += misses.isEmpty()
                    ? QString("<tr><td>%1</td><td align=right>0 of %2</td><td colspan=5>the corner wasn't found</td></tr>").arg(checkCorners[k].name).arg(visits)
                    : QString("<tr><td>%1</td><td align=right>%2 of %3</td><td align=right>%4</td><td align=right>%5, %6</td><td align=right>%7</td>"
                              "<td align=right>%8</td><td align=right>%9</td></tr>")
                          .arg(checkCorners[k].name).arg(misses.count()).arg(visits)
                          .arg(cv::norm(meanMiss), 0, 'f', 3).arg(meanSlider.x, 0, 'f', 3).arg(meanSlider.y, 0, 'f', 3)
                          .arg(worst, 0, 'f', 3).arg(spread, 0, 'f', 3)
                          .arg(holes.isEmpty() ? QString("-") : QString::number(holeSpread, 'f', 3));
    }
    QJsonObject record;
    record["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    record["rounds_planned"] = checkRounds;
    record["visits"] = (int)checkVisits.count();
    record["stopped"] = stopped;
    record["approach"] = QString("every move ends X alone, moving right, over its last %1 mm").arg(SLOP_APPROACH_MM, 0, 'f', 1);
    record["corners"] = json;
    record["log"] = csv.fileName();
    QFile summary(QDir(checkFolder).filePath(QString("corner_check_%1.json").arg(stamp)));
    if (!checkVisits.isEmpty() && summary.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        summary.write(QJsonDocument(record).toJson());
    }
    writeSlopMoves(QDir(checkFolder).filePath(QString("corner_check_%1_moves.csv").arg(stamp)));

    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    if (checkVisits.isEmpty()) {
        endCheck(stopped.isEmpty() ? QString("No corner was visited.") : stopped);
        return;
    }
    QString text = stopped.isEmpty() ? QString("<p>%1 rounds of the four corners.</p>").arg(checkRounds)
                                     : QString("<p>%1 Stopped after %2 visits.</p>").arg(stopped.toHtmlEscaped()).arg(checkVisits.count());
    text += QString("<table cellspacing=6><tr><th align=left>Corner</th><th>Found</th><th>Mean miss</th><th>Slider X, Y to fix</th><th>Worst</th>"
                    "<th>Spread (rms)</th><th>Hole spread</th></tr>%1</table>").arg(rows);
    text += QString("<p>All in mm. <b>Mean miss</b>: how far the corner is from the hole's centre on average, which is how well the fine-tune "
                    "centred it. <b>Spread</b>: how much that changes from round to round, which is how well the sliders return. "
                    "<b>Hole spread</b>: the same for the hole seen beside the corner.</p>");
    text += QString("<p>Logged in %1 as corner_check_%2.*</p>").arg(QDir::toNativeSeparators(checkFolder).toHtmlEscaped(), stamp);
    QMessageBox box(QMessageBox::Information, QString("Check Page Corners"), text, QMessageBox::Ok, this);
    box.setTextFormat(Qt::RichText);
    box.exec();
    endCheck(QString("Checked the page corners: worst mean miss %1 mm, worst spread %2 mm. Logged in %3")
                 .arg(worstMean, 0, 'f', 3).arg(worstSpread, 0, 'f', 3).arg(QDir::toNativeSeparators(csv.fileName())));
}

void LAUGantryMainWindow::endCheck(const QString &message)
{
    checking = false;
    checkCancelling = false;
    checkWaitingFrame = false;
    if (checkTimer) {
        checkTimer->stop();
    }
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->lockScanActions(false);
    cameraWidget->showMessage(message);
}
