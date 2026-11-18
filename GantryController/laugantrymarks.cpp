/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Check side marks: a chart with registration diamonds down its sides (the i1 iSis charts) is on the platen and has
// been scanned (Scan platen image) and, best, fine-tuned (Fine-tune page corners).  The diamonds are found in the
// merged picture, turned into slider positions the way the patches are (LAUPlatenGeometry::railsFor, with the fine-
// tune's correction), and each is visited once with the one-way approach the patches use.  A diamond sits in plain
// margin, so the hole (the disc of paper it shows, print and all) and the diamond are both found in the same frame:
// how far the diamond is from the hole's centre is how far the i1 would land from where it was sent.
//
#include "laugantrymainwindow.h"
#include "lauimageinspectdialog.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <QDir>
#include <QFile>
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
#include <QTextStream>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QDialogButtonBox>
#include <QImage>
#include <opencv2/imgproc.hpp>

namespace
{
    const double MARK_PITCH_MIN_MM = 4.0, MARK_PITCH_MAX_MM = 9.0;   // neighbouring diamonds (6 mm on the iSis charts)
    const double MARK_AREA_MIN_MM2 = 2.0, MARK_AREA_MAX_MM2 = 14.0;
    const double MARK_SIZE_MIN_MM = 1.5, MARK_SIZE_MAX_MM = 7.0;   // the iSis diamonds are 5 x 2.5 mm
    const double MARK_DIAMOND_FIT = 0.6;     // overlap (intersection over union) with a diamond filling the blob's box
    const int MARK_MIN_IN_COLUMN = 8;
    const int MARK_SETTLE_MS = 500;
    const int MARK_FRAME_TIMEOUT_MS = 15000;

    double median(std::vector<double> v)
    {
        if (v.empty()) {
            return 0.0;
        }
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }
}

// The diamonds in the merged picture: small dark blobs about half their bounding box, in straight columns at an even
// pitch.  page: the page's corners in the picture, if one was found (only blobs on it are kept).
bool LAUGantryMainWindow::findSideMarks(const cv::Mat &mosaic, const std::vector<cv::Point2d> &page, QList<MarkStop> *stops, cv::Mat *check, QString *why)
{
    const cv::Matx23d &M = markGeometry.pictureToBoard;
    const double mmPerPixel = std::hypot(M(0, 0), M(1, 0));
    cv::Mat mask = cv::Mat::zeros(mosaic.size(), CV_8U);
    if (page.size() == 4) {
        std::vector<cv::Point> poly;
        for (const cv::Point2d &p : page) {
            poly.emplace_back((int)std::lround(p.x), (int)std::lround(p.y));
        }
        cv::fillConvexPoly(mask, poly, cv::Scalar(255));
        const int in = (int)std::lround(2.0 * 1.0 / mmPerPixel) | 1;   // clear of the paper's own edge
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(in, in)));
    } else {
        mask.setTo(255);
    }
    cv::Mat soft, bw;
    cv::GaussianBlur(mosaic, soft, cv::Size(0, 0), 0.6);
    const int block = (int)std::lround(10.0 / mmPerPixel) | 1;
    cv::adaptiveThreshold(soft, bw, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, block, 25);
    bw &= mask;
    cv::Mat lab, stats, centroids;
    const int count = cv::connectedComponentsWithStats(bw, lab, stats, centroids);
    struct Blob
    {
        cv::Point2d picture, board;
        double area = 0.0;
    };
    std::vector<Blob> blobs;
    for (int k = 1; k < count; k++) {
        const int l = stats.at<int>(k, cv::CC_STAT_LEFT), t = stats.at<int>(k, cv::CC_STAT_TOP);
        const int wp = stats.at<int>(k, cv::CC_STAT_WIDTH), hp = stats.at<int>(k, cv::CC_STAT_HEIGHT);
        const double w = wp * mmPerPixel, h = hp * mmPerPixel;
        const double area = stats.at<int>(k, cv::CC_STAT_AREA) * mmPerPixel * mmPerPixel;
        if (area < MARK_AREA_MIN_MM2 || area > MARK_AREA_MAX_MM2 || std::min(w, h) < MARK_SIZE_MIN_MM || std::max(w, h) > MARK_SIZE_MAX_MM) {
            continue;
        }
        // a diamond: its corners at the middles of its box's sides
        const cv::Mat blob = (lab(cv::Rect(l, t, wp, hp)) == k);
        cv::Mat ideal = cv::Mat::zeros(blob.size(), CV_8U);
        const std::vector<cv::Point> rhombus{ { (int)std::lround(wp / 2.0 - 0.5), 0 }, { wp - 1, (int)std::lround(hp / 2.0 - 0.5) },
                                              { (int)std::lround(wp / 2.0 - 0.5), hp - 1 }, { 0, (int)std::lround(hp / 2.0 - 0.5) } };
        cv::fillConvexPoly(ideal, rhombus, cv::Scalar(255));
        const double overlap = cv::countNonZero(blob & ideal) / std::max(1.0, (double)cv::countNonZero(blob | ideal));
        if (overlap < MARK_DIAMOND_FIT) {
            continue;
        }
        Blob b;
        b.picture = cv::Point2d(centroids.at<double>(k, 0), centroids.at<double>(k, 1));
        b.board = markGeometry.boardOf(b.picture);
        b.area = area;
        blobs.push_back(b);
    }
    if (check) {
        cv::cvtColor(mosaic, *check, cv::COLOR_GRAY2BGR);
        for (const Blob &b : blobs) {
            cv::circle(*check, cv::Point((int)std::lround(b.picture.x), (int)std::lround(b.picture.y)), 4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
    }

    // the columns' direction: most neighbouring pairs lie along it
    double c2 = 0.0, s2 = 0.0;
    int pairs = 0;
    for (size_t i = 0; i < blobs.size(); i++) {
        for (size_t j = i + 1; j < blobs.size(); j++) {
            const cv::Point2d d = blobs[j].board - blobs[i].board;
            const double length = cv::norm(d);
            if (length >= MARK_PITCH_MIN_MM && length <= MARK_PITCH_MAX_MM) {
                const double a = 2.0 * std::atan2(d.y, d.x);
                c2 += std::cos(a);
                s2 += std::sin(a);
                pairs++;
            }
        }
    }
    if (pairs < MARK_MIN_IN_COLUMN) {
        *why = QString("Only %1 diamond-shaped marks were found on the page, not enough to make a column.").arg(blobs.size());
        return false;
    }
    const double angle = 0.5 * std::atan2(s2, c2);
    const cv::Point2d along(std::cos(angle), std::sin(angle)), across(-std::sin(angle), std::cos(angle));

    // the columns: blobs grouped by where they sit across that direction
    std::vector<int> order(blobs.size());
    for (size_t i = 0; i < blobs.size(); i++) {
        order[i] = (int)i;
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) { return blobs[a].board.dot(across) < blobs[b].board.dot(across); });
    std::vector<std::vector<int>> groups;
    for (size_t i = 0; i < order.size(); i++) {
        if (i == 0 || blobs[order[i]].board.dot(across) - blobs[order[i - 1]].board.dot(across) > 2.0) {
            groups.emplace_back();
        }
        groups.back().push_back(order[i]);
    }
    std::vector<std::vector<int>> columns;
    for (std::vector<int> &g : groups) {
        if ((int)g.size() < MARK_MIN_IN_COLUMN) {
            continue;
        }
        std::sort(g.begin(), g.end(), [&](int a, int b) { return blobs[a].board.dot(along) < blobs[b].board.dot(along); });
        // an even pitch: most gaps the same
        std::vector<double> gaps;
        for (size_t i = 1; i < g.size(); i++) {
            gaps.push_back(blobs[g[i]].board.dot(along) - blobs[g[i - 1]].board.dot(along));
        }
        const double pitch = median(gaps);
        int even = 0;
        for (double gap : gaps) {
            even += (std::abs(gap - pitch) < 0.15 * pitch);
        }
        if (pitch >= MARK_PITCH_MIN_MM && pitch <= MARK_PITCH_MAX_MM && even >= 0.7 * gaps.size()) {
            columns.push_back(g);
        }
    }
    if (columns.empty()) {
        *why = QString("%1 diamond-shaped marks were found, but none of them line up in a column at an even pitch.").arg(blobs.size());
        return false;
    }
    // the diamonds run down the outside of the chart: the outermost columns (a patch grid can leave diamond-like
    // pieces in columns between them)
    std::sort(columns.begin(), columns.end(), [&](const std::vector<int> &a, const std::vector<int> &b) {
        return blobs[a[0]].board.dot(across) < blobs[b[0]].board.dot(across);
    });
    if (columns.size() > 2) {
        columns = { columns.front(), columns.back() };
    }

    // stops: down one column and back up the other
    std::vector<double> areas;
    stops->clear();
    for (int c = 0; c < (int)columns.size(); c++) {
        std::vector<int> g = columns[c];
        if (c % 2) {
            std::reverse(g.begin(), g.end());
        }
        for (int i = 0; i < (int)g.size(); i++) {
            const Blob &b = blobs[g[i]];
            MarkStop s;
            s.column = c;
            s.index = (c % 2) ? (int)g.size() - 1 - i : i;
            s.picture = b.picture;
            s.board = b.board;
            markGeometry.railsFor(b.board, &s.single, &s.pair, true);
            stops->append(s);
            areas.push_back(b.area);
            if (check) {
                cv::circle(*check, cv::Point((int)std::lround(b.picture.x), (int)std::lround(b.picture.y)), 9, c ? cv::Scalar(255, 160, 0) : cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
            }
        }
    }
    markAreaMM2 = median(areas);
    return true;
}

void LAUGantryMainWindow::onMarkCheckRequested()
{
    const QString title = QString("Check Side Marks");
    if (scanning || measuring || merging || returningHome || tuning || edgeSloping || pageMoving || checking || markChecking || railWidget->isHoming()) {
        cameraWidget->showMessage(QString("Wait until the sliders and the merge are finished, then try again."));
        return;
    }
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        QMessageBox::information(this, title, QString("Tick Enable Sliders first, so the sliders can move to the marks."));
        return;
    }

    // the newest merged picture of whatever is on the platen
    QDir base(scanBase);
    markFolder.clear();
    for (const QString &name : base.entryList(QStringList{ QString("*_image") }, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed)) {
        if (QFile::exists(base.filePath(name + QString("/mosaic/platen_mosaic.png")))) {
            markFolder = base.filePath(name + QString("/mosaic"));
            break;
        }
    }
    if (markFolder.isEmpty()) {
        QMessageBox::information(this, title, QString("Nothing on the platen has been scanned yet. Put the chart on the platen and choose Scan platen image first."));
        return;
    }
    const QString scanName = QFileInfo(QFileInfo(markFolder).absolutePath()).fileName();
    QString error;
    if (!markGeometry.load(markFolder, &error) || !markGeometry.hasRails()) {
        QMessageBox::warning(this, title, error.isEmpty() ? QString("Scan %1 doesn't know where the i1's hole is: run Calibrate platen with the i1 off, then scan again.").arg(scanName) : error);
        return;
    }
    holeView.load(QSettings().value(QString("LAUGantryMainWindow::bareScanFolder")).toString());
    if (!holeView.isValid()) {
        QMessageBox::warning(this, title, QString("The platen calibration can't be read, so the camera can't see the hole. Run Calibrate platen with the i1 off."));
        return;
    }
    const QImage picture = QImage(QDir(markFolder).filePath(QString("platen_mosaic.png"))).convertToFormat(QImage::Format_Grayscale8);
    if (picture.isNull()) {
        QMessageBox::warning(this, title, QString("Couldn't open the merged picture of scan %1.").arg(scanName));
        return;
    }
    cv::Mat mosaic(picture.height(), picture.width(), CV_8UC1);
    for (int r = 0; r < picture.height(); r++) {
        memcpy(mosaic.ptr(r), picture.constScanLine(r), picture.width());
    }
    const auto readJson = [this](const QString &name) {
        QFile file(QDir(markFolder).filePath(name));
        return file.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject();
    };
    std::vector<cv::Point2d> page;
    for (const QJsonValue &v : readJson(QString("page.json"))["corners_picture_px"].toArray()) {
        page.emplace_back(v.toArray().at(0).toDouble(), v.toArray().at(1).toDouble());
    }
    const QJsonObject tuned = readJson(QString("corner_tune.json"));
    const bool fineTuned = !tuned.isEmpty();
    markAperture = holeView.calibratedAperture();
    const QJsonArray seen = tuned["hole_seen_at_rails_zero_mm"].toArray();
    if (!seen.isEmpty()) {
        markAperture = cv::Point2d(0, 0);
        for (const QJsonValue &v : seen) {
            markAperture += cv::Point2d(v.toArray().at(0).toDouble(), v.toArray().at(1).toDouble()) * (1.0 / seen.count());
        }
    }

    cameraWidget->showMessage(QString("Finding the diamonds down the chart's sides in the merged picture..."));
    cv::Mat check;
    QList<MarkStop> stops;
    if (!findSideMarks(mosaic, page, &stops, &check, &error)) {
        QMessageBox::information(this, title, error);
        cameraWidget->showMessage(error);
        return;
    }
    double sLow = 0.0, sHigh = 0.0, pLow = 0.0, pHigh = 0.0;
    const bool limits = railWidget->limitRange(0, &sLow, &sHigh) && railWidget->limitRange(2, &pLow, &pHigh);
    int outOfReach = 0;
    markStops.clear();
    for (const MarkStop &s : stops) {
        if (limits && (s.single < sLow || s.single > sHigh || s.pair < pLow || s.pair > pHigh)) {
            outOfReach++;
            continue;
        }
        markStops << s;
    }
    if (markStops.isEmpty()) {
        QMessageBox::information(this, title, QString("All %1 diamonds found are outside the limit sliders.").arg(stops.count()));
        return;
    }
    int columns = 0;
    for (const MarkStop &s : stops) {
        columns = std::max(columns, s.column + 1);
    }

    // the user checks what was found before the sliders go
    QDialog confirm(this);
    confirm.setWindowTitle(title);
    confirm.resize(1000, 860);
    QVBoxLayout *layout = new QVBoxLayout(&confirm);
    cv::Mat rgb;
    cv::cvtColor(check, rgb, cv::COLOR_BGR2RGB);
    layout->addWidget(new LAUImageInspectView(QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).copy()), 1);
    QString text = QString("Found %1 diamonds in %2 column%3 of scan %4 (green and orange; red: dark marks that aren't in a column).")
                       .arg(stops.count()).arg(columns).arg(columns > 1 ? "s" : "").arg(scanName);
    if (outOfReach) {
        text += QString(" %1 are outside the limit sliders and are left out.").arg(outOfReach);
    }
    text += fineTuned ? QString("\n\nThe sliders go to each with the page corners' fine-tune correction, as the patches would.")
                      : QString("\n\nThis scan hasn't been fine-tuned (Fine-tune page corners), so the sliders go by the platen calibration alone.");
    text += QString("\n\nTake the i1 Pro out of its holder first. Each diamond is visited once; the camera measures how far it is from the hole's centre.");
    QLabel *label = new QLabel(text);
    label->setWordWrap(true);
    layout->addWidget(label);
    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(QString("Visit them"));
    connect(buttons, &QDialogButtonBox::accepted, &confirm, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &confirm, &QDialog::reject);
    layout->addWidget(buttons);
    if (confirm.exec() != QDialog::Accepted) {
        cameraWidget->showMessage(QString("The diamonds were found but not visited."));
        return;
    }

    if (!markTimer) {
        markTimer = new QTimer(this);
        markTimer->setSingleShot(true);
        connect(markTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onMarkTimer);
    }
    markChecking = true;
    markCancelling = false;
    markWaitingFrame = false;
    markIndex = 0;
    markVisits.clear();
    resetSlopTracking();
    cameraWidget->lockScanActions(true);
    railBox->setEnabled(false);
    homeDialog = new LAUHomeDialog(this);
    homeDialog->setWindowTitle(title);
    connect(homeDialog, &LAUHomeDialog::emitAbort, this, &LAUGantryMainWindow::onMarkAbort);
    homeDialog->open();
    markNextVisit();
}

void LAUGantryMainWindow::markNextVisit()
{
    const MarkStop &s = markStops[markIndex];
    homeDialog->setMessage(QString("Diamond %1 of %2 (column %3, number %4)...").arg(markIndex + 1).arg(markStops.count()).arg(s.column + 1).arg(s.index + 1));
    homeDialog->setProgress(markIndex, markStops.count());
    moveRailsSlop(s.single, s.pair, QString("column %1 diamond %2").arg(s.column + 1).arg(s.index + 1));
}

void LAUGantryMainWindow::onMarkMoveFinished(bool ok, const QString &message)
{
    if (markCancelling) {
        finishMarks(QString("Checking was aborted."));
        return;
    }
    if (!ok) {
        finishMarks(QString("The sliders stopped: %1").arg(message));
        return;
    }
    markTimer->start(MARK_SETTLE_MS);
}

void LAUGantryMainWindow::onMarkTimer()
{
    if (!markChecking) {
        return;
    }
    if (markWaitingFrame) {
        markWaitingFrame = false;
        finishMarks(QString("The camera stopped delivering frames."));
        return;
    }
    markWaitingFrame = true;
    markTimer->start(MARK_FRAME_TIMEOUT_MS);
    cameraWidget->requestScanFrame();
}

void LAUGantryMainWindow::onMarkFrame(const cv::Mat &raw16)
{
    if (!markChecking || !markWaitingFrame) {
        return;
    }
    markWaitingFrame = false;
    markTimer->stop();
    if (markCancelling) {
        finishMarks(QString("Checking was aborted."));
        return;
    }
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);      // Mono10 -> 8 bits
    const MarkStop &s = markStops[markIndex];
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    MarkVisit v;
    v.stop = markIndex;
    v.single = railWidget->reportedMM(0);
    v.pair = railWidget->reportedMM(2);
    LAUHoleView::Hole hole;
    QString why;
    const cv::Point2d guess = markAperture + ax * v.pair + ay * v.single;
    if (holeView.findHole(gray, v.pair, v.single, guess, &hole, &why, nullptr, true)) {
        v.holeFound = true;
        v.hole = hole.centre;
        markAperture = hole.aperture;        // the next one's guess
        double area = 0.0;
        if (holeView.findMark(gray, v.pair, v.single, hole, s.board, 0.4 * markAreaMM2, 2.5 * markAreaMM2, &v.mark, &area, &v.why)) {
            v.found = true;
            v.areaMM2 = area;
            v.miss = v.mark - v.hole;
            const double det = ax.x * ay.y - ay.x * ax.y;
            v.missPair = (v.miss.x * ay.y - ay.x * v.miss.y) / det;
            v.missSingle = (ax.x * v.miss.y - v.miss.x * ax.y) / det;
        }
    } else {
        v.holeWhy = why;
    }
    markVisits << v;
    showCheckZoom(gray, v.holeFound ? v.hole : guess, nullptr,
                  v.found ? QString("diamond %1 mm from the hole's centre").arg(cv::norm(v.miss), 0, 'f', 3)
                          : QString(v.holeFound ? v.why : QString("hole not found: %1").arg(v.holeWhy)).left(70),
                  v.found ? &v.mark : nullptr);

    if (++markIndex >= markStops.count()) {
        finishMarks(QString());
        return;
    }
    markNextVisit();
}

void LAUGantryMainWindow::onMarkAbort()
{
    if (!markChecking) {
        return;
    }
    markCancelling = true;
    if (railWidget->isHoming()) {
        railWidget->abortHome();             // onMarkMoveFinished finishes up
        return;
    }
    if (markWaitingFrame) {
        return;                              // onMarkFrame finishes up
    }
    markTimer->stop();
    finishMarks(QString("Checking was aborted."));
}

// per column and overall: the mean miss, its spread, the worst, and how it changes along the column
void LAUGantryMainWindow::finishMarks(const QString &stopped)
{
    const QString stamp = QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss"));
    const QString stem = QDir(markFolder).filePath(QString("marks_check_%1").arg(stamp));
    QFile csv(stem + QString(".csv"));
    if (!markVisits.isEmpty() && csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "# the diamond less the hole's centre (board mm), both seen in the same frame; miss_single/pair: the slider move that would close it\n";
        out << "visit,column,diamond,board_x_mm,board_y_mm,target_single_mm,target_pair_mm,reported_single_mm,reported_pair_mm,hole_found,hole_x_mm,hole_y_mm,"
               "diamond_found,diamond_x_mm,diamond_y_mm,diamond_area_mm2,miss_x_mm,miss_y_mm,miss_mm,miss_single_mm,miss_pair_mm,note\n";
        for (int k = 0; k < markVisits.count(); k++) {
            const MarkVisit &v = markVisits[k];
            const MarkStop &s = markStops[v.stop];
            out << (k + 1) << "," << (s.column + 1) << "," << (s.index + 1) << "," << QString::number(s.board.x, 'f', 4) << "," << QString::number(s.board.y, 'f', 4) << ","
                << QString::number(s.single, 'f', 4) << "," << QString::number(s.pair, 'f', 4) << "," << QString::number(v.single, 'f', 4) << ","
                << QString::number(v.pair, 'f', 4) << "," << (v.holeFound ? 1 : 0) << ",";
            out << (v.holeFound ? QString("%1,%2").arg(v.hole.x, 0, 'f', 4).arg(v.hole.y, 0, 'f', 4) : QString(",")) << "," << (v.found ? 1 : 0) << ",";
            if (v.found) {
                out << QString::number(v.mark.x, 'f', 4) << "," << QString::number(v.mark.y, 'f', 4) << "," << QString::number(v.areaMM2, 'f', 3) << ","
                    << QString::number(v.miss.x, 'f', 4) << "," << QString::number(v.miss.y, 'f', 4) << "," << QString::number(cv::norm(v.miss), 'f', 4) << ","
                    << QString::number(v.missSingle, 'f', 4) << "," << QString::number(v.missPair, 'f', 4) << ",";
            } else {
                out << ",,,,,,,,";
            }
            out << "\"" << QString(v.why.isEmpty() ? v.holeWhy : v.why).replace(QChar('"'), QChar('\'')) << "\"\n";
        }
        csv.close();
    }
    writeSlopMoves(stem + QString("_moves.csv"));

    int columns = 0;
    for (const MarkStop &s : markStops) {
        columns = std::max(columns, s.column + 1);
    }
    QString rows;
    QJsonArray json;
    double worstAll = 0.0;
    for (int c = -1; c < columns; c++) {   // -1: all of them
        std::vector<cv::Point2d> sliderMiss;
        std::vector<double> along, missMM;
        int visits = 0;
        for (const MarkVisit &v : markVisits) {
            const MarkStop &s = markStops[v.stop];
            if (c >= 0 && s.column != c) {
                continue;
            }
            visits++;
            if (v.found) {
                sliderMiss.emplace_back(v.missSingle, v.missPair);
                missMM.push_back(cv::norm(v.miss));
                along.push_back(s.index);   // diamond number along the column
            }
        }
        if (visits == 0) {
            continue;
        }
        QJsonObject o;
        o["column"] = (c < 0) ? QString("all") : QString::number(c + 1);
        o["visits"] = visits;
        o["found"] = (int)missMM.size();
        QString row = QString("<tr><td>%1</td><td align=right>%2 of %3</td>").arg(c < 0 ? QString("<b>all</b>") : QString("column %1").arg(c + 1)).arg(missMM.size()).arg(visits);
        if (!missMM.empty()) {
            cv::Point2d mean(0, 0);
            for (const cv::Point2d &m : sliderMiss) {
                mean += m * (1.0 / sliderMiss.size());
            }
            double ss = 0.0, worst = 0.0;
            for (size_t i = 0; i < sliderMiss.size(); i++) {
                const cv::Point2d d = sliderMiss[i] - mean;
                ss += d.dot(d);
                worst = std::max(worst, missMM[i]);
            }
            const double rmsAbout = std::sqrt(ss / sliderMiss.size());
            const double meanMM = std::accumulate(missMM.begin(), missMM.end(), 0.0) / missMM.size();
            // the trend along the column, slider mm per diamond, for each rail
            double slopeS = 0.0, slopeP = 0.0;
            if (c >= 0 && along.size() > 2) {
                double ma = 0.0;
                for (double a : along) {
                    ma += a / along.size();
                }
                double saa = 0.0, sas = 0.0, sap = 0.0;
                for (size_t i = 0; i < along.size(); i++) {
                    saa += (along[i] - ma) * (along[i] - ma);
                    sas += (along[i] - ma) * (sliderMiss[i].x - mean.x);
                    sap += (along[i] - ma) * (sliderMiss[i].y - mean.y);
                }
                if (saa > 0.0) {
                    slopeS = sas / saa;
                    slopeP = sap / saa;
                }
            }
            if (c < 0) {
                worstAll = worst;
            }
            o["mean_miss_single_pair_mm"] = QJsonArray{ mean.x, mean.y };
            o["mean_miss_mm"] = meanMM;
            o["worst_miss_mm"] = worst;
            o["spread_about_mean_rms_mm"] = rmsAbout;
            if (c >= 0) {
                o["trend_single_pair_mm_per_diamond"] = QJsonArray{ slopeS, slopeP };
            }
            row += QString("<td align=right>%1</td><td align=right>%2, %3</td><td align=right>%4</td><td align=right>%5</td><td align=right>%6</td></tr>")
                       .arg(meanMM, 0, 'f', 3).arg(mean.x, 0, 'f', 3).arg(mean.y, 0, 'f', 3).arg(rmsAbout, 0, 'f', 3).arg(worst, 0, 'f', 3)
                       .arg(c < 0 ? QString("-") : QString("%1, %2").arg(slopeS * 10.0, 0, 'f', 3).arg(slopeP * 10.0, 0, 'f', 3));
        } else {
            row += QString("<td colspan=5>none found</td></tr>");
        }
        rows += row;
        json.append(o);
    }
    QJsonObject record;
    record["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    record["scan"] = QFileInfo(QFileInfo(markFolder).absolutePath()).fileName();
    record["stopped"] = stopped;
    record["diamond_area_mm2_in_picture"] = markAreaMM2;
    record["approach"] = QString("every move ends X alone, moving right, over its last %1 mm").arg(SLOP_APPROACH_MM, 0, 'f', 1);
    record["summary"] = json;
    record["log"] = csv.fileName();
    QFile summary(stem + QString(".json"));
    if (!markVisits.isEmpty() && summary.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        summary.write(QJsonDocument(record).toJson());
    }

    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    markChecking = false;
    markCancelling = false;
    markWaitingFrame = false;
    if (markTimer) {
        markTimer->stop();
    }
    railBox->setEnabled(true);
    cameraWidget->lockScanActions(false);
    if (markVisits.isEmpty()) {
        cameraWidget->showMessage(stopped.isEmpty() ? QString("No diamond was visited.") : stopped);
        return;
    }
    QString text = stopped.isEmpty() ? QString("<p>Visited all %1 diamonds.</p>").arg(markVisits.count())
                                     : QString("<p>%1 Stopped after %2 of %3 diamonds.</p>").arg(stopped.toHtmlEscaped()).arg(markVisits.count()).arg(markStops.count());
    text += QString("<table cellspacing=6><tr><th align=left></th><th>Found</th><th>Mean miss</th><th>Mean slider X, Y</th><th>Spread (rms)</th><th>Worst</th>"
                    "<th>Trend X, Y per 10</th></tr>%1</table>").arg(rows);
    text += QString("<p>All in mm. <b>Miss</b>: how far the diamond is from the hole's centre, both seen in the same frame, so how far the i1 "
                    "would land from where it was sent. <b>Mean slider X, Y</b>: the move that would close the average miss. <b>Spread</b>: "
                    "how much the misses vary about that. <b>Trend</b>: how the miss changes along the column, per 10 diamonds.</p>");
    text += QString("<p>Logged in %1 as %2.*</p>").arg(QDir::toNativeSeparators(markFolder).toHtmlEscaped(), QFileInfo(stem).fileName());
    QMessageBox box(QMessageBox::Information, QString("Check Side Marks"), text, QMessageBox::Ok, this);
    box.setTextFormat(Qt::RichText);
    box.exec();
    cameraWidget->showMessage(QString("Checked the side marks: worst miss %1 mm. Logged in %2").arg(worstAll, 0, 'f', 3).arg(QDir::toNativeSeparators(csv.fileName())));
}
