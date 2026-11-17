/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Measure X slop on the page's edge.  Slop is taken up when the head changes direction, not while it keeps moving the
// same way, so it can be measured without any model: put the hole's centre exactly on a straight page edge across the
// X travel, arriving once moving left and once moving right, and the two slider positions differ by the slop.
//
//   Learn a side:  out 1" past the edge on that side, back to 1/8" short of it in one move, then creep toward the edge
//                  in steps that always fall short (a share of the miss the camera sees), never reversing, until the
//                  hole's centre is on the edge.  That X is where the hole lands arriving from that side.
//   Predict:       then alternately from each side, out 1" and back in ONE move straight to that side's X; the camera's
//                  miss refines it (each side's X is the mean of where the hole was on the edge), until single moves
//                  land on the edge within tolerance from both sides.
//
// X is the single rail.  Arriving moving left (-X) and moving right (+X) give X_L and X_R; the slop is X_R - X_L, and
// a target found without regard to direction is reached by adding lastX * (X_R - X_L) / 2 to its midpoint.
//
#include "laugantrymainwindow.h"
#include "lauimageinspectdialog.h"

#include <cmath>
#include <algorithm>
#include <QDir>
#include <QFile>
#include <QImage>
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
    const double CREEP_SHARE = 0.7;          // each creep covers this share of the miss, so it never overshoots
    const double SMALLEST_STEP_MM = 0.005;   // a creep smaller than this is made as the whole miss
    const int MAX_CREEPS = 25;
    const int MAX_RESTARTS = 3;              // overshot while creeping: start that side again
    const int HITS_NEEDED = 2;               // single moves in a row within tolerance, from each side

    cv::Point2d unitOf(const cv::Point2d &v)
    {
        return v * (1.0 / std::max(cv::norm(v), 1e-12));
    }

    int sideIndex(int side)                  // side +1 comes from the right moving left: X_L, index 0
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
    const QString title = QString("Measure X Slop on the Page Edge");
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
    QLabel *label = new QLabel(QString("The hole's centre is put exactly on the page's left edge (or its right edge, if the left can't be reached), "
                                       "arriving once moving left and once moving right. Each side starts 1\" past the edge, comes back to 1/8\" "
                                       "short of it in one move, then creeps toward it without ever reversing. Then single moves from each side, "
                                       "straight to where that side put the hole on the edge, are refined until they land on it.\n\n"
                                       "Take the i1 Pro out of its holder first so the camera sees the paper through the hole, and don't move the page."));
    label->setWordWrap(true);
    layout->addWidget(label);
    QFormLayout *form = new QFormLayout();
    QDoubleSpinBox *away = new QDoubleSpinBox(), *shortOf = new QDoubleSpinBox(), *tolerance = new QDoubleSpinBox(), *tries = new QDoubleSpinBox();
    away->setRange(5.0, 60.0);
    away->setSuffix(QString(" mm"));
    away->setValue(settings.value(QString("LAUGantryMainWindow::edgeAway"), 25.4).toDouble());
    shortOf->setRange(0.5, 10.0);
    shortOf->setDecimals(3);
    shortOf->setSuffix(QString(" mm"));
    shortOf->setValue(settings.value(QString("LAUGantryMainWindow::edgeShort"), 3.175).toDouble());
    tolerance->setRange(0.005, 0.5);
    tolerance->setDecimals(3);
    tolerance->setSingleStep(0.005);
    tolerance->setSuffix(QString(" mm"));
    tolerance->setValue(settings.value(QString("LAUGantryMainWindow::edgeTolerance"), 0.02).toDouble());
    tries->setRange(2, 20);
    tries->setDecimals(0);
    tries->setValue(settings.value(QString("LAUGantryMainWindow::edgeTries"), 8).toInt());
    form->addRow(QString("Start past the edge"), away);
    form->addRow(QString("First stop short of it"), shortOf);
    form->addRow(QString("On the edge within"), tolerance);
    form->addRow(QString("Single moves, at most, per side"), tries);
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
    settings.setValue(QString("LAUGantryMainWindow::edgeShort"), shortOf->value());
    settings.setValue(QString("LAUGantryMainWindow::edgeTolerance"), tolerance->value());
    settings.setValue(QString("LAUGantryMainWindow::edgeTries"), (int)tries->value());
    edgeAway = away->value();
    edgeShort = shortOf->value();
    edgeTolerance = tolerance->value();
    edgeTries = (int)tries->value();

    // the page edges that cross the X travel (most nearly square to the single rail), the left one first
    std::vector<cv::Point2d> P;
    cv::Point2d middle(0, 0);
    for (int k = 0; k < 4; k++) {
        P.emplace_back(corners[k].toArray().at(0).toDouble(), corners[k].toArray().at(1).toDouble());
        middle += P.back() * 0.25;
    }
    const cv::Point2d xDir = unitOf(holeView.boardAxis(1));   // board direction of +X (the single rail)
    std::vector<int> crossing{ 0, 1, 2, 3 };
    std::sort(crossing.begin(), crossing.end(), [&](int a, int b) {
        return std::abs(unitOf(P[(a + 1) % 4] - P[a]).dot(xDir)) < std::abs(unitOf(P[(b + 1) % 4] - P[b]).dot(xDir));
    });
    crossing.resize(2);
    std::vector<double> xOf(4, 0.0), yOf(4, 0.0);
    for (int k : crossing) {
        slopGeometry.railsFor((P[k] + P[(k + 1) % 4]) * 0.5, &xOf[k], &yOf[k], true);
    }
    std::sort(crossing.begin(), crossing.end(), [&](int a, int b) { return xOf[a] < xOf[b]; });
    int chosen = -1;
    for (int k : crossing) {
        const double margin = edgeAway + 2.0;
        if (slopGeometry.reachable(xOf[k] - margin, yOf[k]) && slopGeometry.reachable(xOf[k] + margin, yOf[k])) {
            chosen = k;
            break;
        }
    }
    if (chosen < 0) {
        QMessageBox::warning(this, title, QString("Neither page edge across the X travel can be reached %1 mm to both sides with the limit sliders "
                                                  "where they are. Start closer to the edge, or widen the limit sliders.").arg(edgeAway, 0, 'f', 1));
        return;
    }
    edgeName = (chosen == crossing[0]) ? QString("left") : QString("right");
    edgeAlong = P[(chosen + 1) % 4] - P[chosen];
    edgeIntoPaper = middle - (P[chosen] + P[(chosen + 1) % 4]) * 0.5;
    edgeY = yOf[chosen];
    edgeModelX = xOf[chosen];

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
    edgeFolder = QDir(scanBase).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) + QString("_edgeslop"));
    QDir().mkpath(edgeFolder);

    if (!edgeTimer) {
        edgeTimer = new QTimer(this);
        edgeTimer->setSingleShot(true);
        connect(edgeTimer, &QTimer::timeout, this, &LAUGantryMainWindow::onEdgeTimer);
    }
    edgeSloping = true;
    edgeCancelling = false;
    edgeWaitingFrame = false;
    edgeOnEdge[0].clear();
    edgeOnEdge[1].clear();
    edgeHits[0] = edgeHits[1] = 0;
    edgeSingles[0] = edgeSingles[1] = 0;
    edgeLog = QJsonArray();
    edgeStops.clear();
    edgeSide = +1;                           // learn the right side first: from the paper, moving left onto the edge
    edgeLearning = true;
    edgeRestarts = 0;
    slopSeenValid = false;
    slopSeenHole = false;
    slopSeenWhy.clear();
    slopZoomTitle = QString("%1 edge").arg(edgeName);
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
            endEdgeSlop(QString("Measuring the X slop was cancelled."));
        }
    });
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitLiveFrame, homeDialog, [this](cv::Mat raw16) { showSlopZoom(raw16); });
    homeDialog->open();
    edgeStartSide();
}

// where this side puts the hole on the edge: what it has measured so far, else what the other side has, else the model
double LAUGantryMainWindow::edgeEstimate(int side) const
{
    const QList<double> &own = edgeOnEdge[sideIndex(side)], &other = edgeOnEdge[sideIndex(-side)];
    const QList<double> &use = !own.isEmpty() ? own : other;
    if (use.isEmpty()) {
        return edgeModelX;
    }
    double sum = 0.0;
    for (double x : use) {
        sum += x;
    }
    return sum / use.count();
}

void LAUGantryMainWindow::edgeMoveTo(double x, EdgeStage stage, const QString &message)
{
    edgeStage = stage;
    edgeX = x;
    homeDialog->setMessage(message);
    railWidget->moveRails(x, edgeY);         // answers with emitHomeFinished -> onEdgeMoveFinished
}

// out past the edge on this side
void LAUGantryMainWindow::edgeStartSide()
{
    const double x = edgeEstimate(edgeSide) + edgeSide * edgeAway;
    edgeCreeps = 0;
    edgeMoveTo(x, EdgeOut, QString("%1 %2: out %3 mm past the edge...").arg(edgeLearning ? QString("Learning") : QString("Single move"))
                               .arg(sideName(edgeSide)).arg(edgeAway, 0, 'f', 1));
}

void LAUGantryMainWindow::onEdgeMoveFinished(bool ok, const QString &message)
{
    if (edgeCancelling) {
        endEdgeSlop(QString("Measuring the X slop was cancelled."));
        return;
    }
    if (!ok) {
        endEdgeSlop(QString("Measuring the X slop stopped: %1").arg(message));
        return;
    }
    if (edgeStage == EdgeOut) {
        if (edgeLearning) {
            edgeMoveTo(edgeEstimate(edgeSide) + edgeSide * edgeShort, EdgeShort,
                       QString("Learning %1: to %2 mm short of the edge...").arg(sideName(edgeSide)).arg(edgeShort, 0, 'f', 3));
        } else {
            edgeMoveTo(edgeEstimate(edgeSide), EdgeSingle, QString("Single move %1: straight to where it lands on the edge, X %2 mm...")
                                                                .arg(sideName(edgeSide)).arg(edgeEstimate(edgeSide), 0, 'f', 3));
        }
        return;
    }
    edgeTimer->start(SETTLE_MS);             // then a frame of the edge
}

void LAUGantryMainWindow::onEdgeTimer()
{
    if (!edgeSloping) {
        return;
    }
    if (edgeCancelling) {
        endEdgeSlop(QString("Measuring the X slop was cancelled."));
        return;
    }
    if (edgeWaitingFrame) {
        edgeWaitingFrame = false;
        endEdgeSlop(QString("Measuring the X slop stopped: the camera stopped delivering frames."));
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
        endEdgeSlop(QString("Measuring the X slop was cancelled."));
        return;
    }
    cv::Mat gray;
    raw16.convertTo(gray, CV_8U, 0.25);      // Mono10 -> 8 bits, as the scans are saved
    const double pair = railWidget->reportedMM(2), single = railWidget->reportedMM(0);
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const cv::Point2d guess = slopAperture + ax * pair + ay * single;
    LAUHoleView::Corner edge;
    cv::Point2d hole;
    QString why;
    cv::Mat check;
    double radius = 0.0;
    bool found = holeView.findHoleAtEdge(gray, pair, single, guess, edgeAlong, edgeIntoPaper, &hole, &edge, &why, &check, &radius);
    slopSeen = edge;
    slopSeenValid = true;
    slopSeenHole = found;
    slopSeenWhy = why;
    slopSeenRadius = radius;
    if (found) {
        slopSeenAperture = hole - ax * pair - ay * single;
        slopAperture = slopSeenAperture;     // look there next time
    }
    const int stop = edgeStops.count() + 1;
    if (!check.empty()) {
        QImage(check.data, check.cols, check.rows, (int)check.step, QImage::Format_BGR888)
            .save(QDir(edgeFolder).filePath(QString("stop_%1.png").arg(stop, 3, 10, QChar('0'))));
    }

    // the X move that puts the hole's centre on the edge: n . (hole + ay * dX - edge) = 0
    double miss = 0.0;
    if (found) {
        const cv::Point2d n(-edge.edgeDir[0].y, edge.edgeDir[0].x);
        const double across = n.dot(ay);
        found = std::abs(across) > 1e-6;
        if (found) {
            miss = -n.dot(hole - edge.edgeAt[0]) / across;
        } else {
            why = QString("The edge runs along the X travel, so X can't put the hole on it.");
        }
    }
    EdgeStop s;
    s.stage = edgeStage;
    s.side = edgeSide;
    s.commanded = edgeX;
    s.reported = single;
    s.found = found;
    s.miss = miss;
    s.note = why;
    edgeStops << s;
    slopZoomNote = found ? QString("hole's centre to the edge: %1 mm in X").arg(miss, 0, 'f', 3) : QString();

    if (!found) {
        endEdgeSlop(QString("Measuring the X slop stopped: the camera couldn't find the hole on the edge (%1). Its view is saved as stop_%2.png in %3")
                        .arg(why).arg(stop, 3, 10, QChar('0')).arg(QDir::toNativeSeparators(edgeFolder)));
        return;
    }
    const int i = sideIndex(edgeSide);
    const double onEdge = single + miss;     // where this side's arrival would have put the hole on the edge

    if (edgeLearning) {
        if (std::abs(miss) < edgeTolerance) {
            edgeOnEdge[i] << onEdge;         // learned: the hole is on the edge, arrived from this side
            edgeNextSide();
            return;
        }
        if (miss * edgeSide > 0.0) {
            // past the edge (the miss points back the way it came): start this side again rather than reverse
            if (++edgeRestarts > MAX_RESTARTS) {
                endEdgeSlop(QString("Measuring the X slop stopped: creeping %1 overshot the edge %2 times.").arg(sideName(edgeSide)).arg(edgeRestarts));
                return;
            }
            edgeStartSide();
            return;
        }
        if (++edgeCreeps > MAX_CREEPS) {
            endEdgeSlop(QString("Measuring the X slop stopped: %1 creeps %2 didn't reach the edge.").arg(MAX_CREEPS).arg(sideName(edgeSide)));
            return;
        }
        const double step = (std::abs(miss) * CREEP_SHARE < SMALLEST_STEP_MM) ? miss : miss * CREEP_SHARE;
        edgeMoveTo(edgeX + step, EdgeCreep, QString("Learning %1: creeping onto the edge, %2 mm to go (creep %3)...")
                                                 .arg(sideName(edgeSide)).arg(std::abs(miss), 0, 'f', 3).arg(edgeCreeps));
        return;
    }

    // a single move: how far it landed from the edge refines this side's X
    edgeSingles[i]++;
    edgeHits[i] = (std::abs(miss) < edgeTolerance) ? edgeHits[i] + 1 : 0;
    edgeOnEdge[i] << onEdge;
    edgeNextSide();
}

// on to the other side, or finished
void LAUGantryMainWindow::edgeNextSide()
{
    edgeRestarts = 0;
    if (edgeLearning) {
        if (edgeOnEdge[1].isEmpty()) {
            edgeSide = -1;                   // learn the left side too
            edgeStartSide();
            return;
        }
        edgeLearning = false;                // both learned: single moves, alternating
        edgeSide = +1;
        edgeStartSide();
        return;
    }
    const bool done = (edgeHits[0] >= HITS_NEEDED && edgeHits[1] >= HITS_NEEDED) || (edgeSingles[0] >= edgeTries && edgeSingles[1] >= edgeTries);
    if (done) {
        finishEdgeSlop();
        return;
    }
    edgeSide = -edgeSide;
    if (edgeSingles[sideIndex(edgeSide)] >= edgeTries || edgeHits[sideIndex(edgeSide)] >= HITS_NEEDED) {
        edgeSide = -edgeSide;                // that side is done: keep at this one
    }
    edgeStartSide();
}

void LAUGantryMainWindow::finishEdgeSlop()
{
    const double xl = edgeEstimate(+1), xr = edgeEstimate(-1);
    const double slop = xr - xl;
    const bool converged = edgeHits[0] >= HITS_NEEDED && edgeHits[1] >= HITS_NEEDED;

    // ---- the log
    QFile csv(QDir(edgeFolder).filePath(QString("edge_slop.csv")));
    if (csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "stop,stage,arriving,commanded_x_mm,reported_x_mm,found,miss_x_mm,on_edge_x_mm,note\n";
        const QStringList stages{ QString("out"), QString("short"), QString("creep"), QString("single") };
        for (int k = 0; k < edgeStops.count(); k++) {
            const EdgeStop &s = edgeStops[k];
            out << k + 1 << ',' << stages.value((int)s.stage) << ',' << (s.side > 0 ? "moving_left" : "moving_right") << ','
                << QString::number(s.commanded, 'f', 4) << ',' << QString::number(s.reported, 'f', 4) << ',' << (s.found ? 1 : 0) << ','
                << QString::number(s.miss, 'f', 4) << ',' << QString::number(s.reported + s.miss, 'f', 4) << ",\""
                << QString(s.note).replace(QChar('"'), QChar('\'')) << "\"\n";
        }
    }

    // ---- the plot: each stop's miss, by stop, the two sides in their own colours, the tolerance shaded
    const int width = 1000, height = 640, left = 80, right = 30, top = 120, bottom = 60;
    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    double largest = edgeTolerance * 2.0;
    for (const EdgeStop &s : edgeStops) {
        largest = std::max(largest, std::abs(s.miss));
    }
    largest = std::min(largest, std::max(edgeShort * 1.1, edgeTolerance * 2.0));
    const int count = std::max(2, (int)edgeStops.count());
    const auto toPlot = [&](int k, double miss) {
        const double x = left + (width - left - right) * (k / (double)(count - 1));
        const double y = top + (height - top - bottom) * 0.5 * (1.0 - std::clamp(miss / largest, -1.0, 1.0));
        return cv::Point((int)std::lround(x), (int)std::lround(y));
    };
    const cv::Scalar ink(40, 40, 40), grid(215, 215, 215), leftColour(200, 90, 20), rightColour(30, 120, 230);
    cv::rectangle(plot, toPlot(0, edgeTolerance), toPlot(count - 1, -edgeTolerance), cv::Scalar(225, 245, 225), cv::FILLED);
    cv::line(plot, toPlot(0, 0.0), toPlot(count - 1, 0.0), grid, 1, cv::LINE_AA);
    for (double v : { largest, largest / 2.0, -largest / 2.0, -largest }) {
        cv::line(plot, toPlot(0, v), toPlot(count - 1, v), grid, 1, cv::LINE_AA);
        cv::putText(plot, QString("%1").arg(v, 0, 'f', 3).toStdString(), toPlot(0, v) + cv::Point(-70, 4), cv::FONT_HERSHEY_SIMPLEX, 0.4, ink, 1, cv::LINE_AA);
    }
    cv::Point previous[2];
    bool havePrevious[2] = { false, false };
    for (int k = 0; k < edgeStops.count(); k++) {
        const EdgeStop &s = edgeStops[k];
        const int i = sideIndex(s.side);
        const cv::Scalar colour = (s.side > 0) ? leftColour : rightColour;
        const cv::Point p = toPlot(k, s.miss);
        if (havePrevious[i]) {
            cv::line(plot, previous[i], p, colour, 1, cv::LINE_AA);
        }
        previous[i] = p;
        havePrevious[i] = true;
        if (s.stage == EdgeSingle) {
            cv::circle(plot, p, 6, colour, cv::FILLED, cv::LINE_AA);
        } else {
            cv::circle(plot, p, 5, colour, 1, cv::LINE_AA);
        }
    }
    const QStringList lines{
        QString("X slop on the page's %1 edge, scan %2: miss from the edge at each stop (mm in X)").arg(edgeName, edgeScan),
        QString("arriving moving left  X_L = %1 mm      arriving moving right  X_R = %2 mm").arg(xl, 0, 'f', 3).arg(xr, 0, 'f', 3),
        QString("slop X_R - X_L = %1 mm     %2").arg(slop, 0, 'f', 3)
            .arg(converged ? QString("single moves land within %1 mm from both sides").arg(edgeTolerance, 0, 'f', 3)
                           : QString("NOT converged within %1 single moves a side").arg(edgeTries)),
        QString("open circles: learning (short stop, creeps); filled: single moves.  blue: moving left   orange: moving right"),
    };
    for (int k = 0; k < lines.count(); k++) {
        cv::putText(plot, lines[k].toStdString(), cv::Point(14, 24 + 22 * k), cv::FONT_HERSHEY_SIMPLEX, 0.52, ink, 1, cv::LINE_AA);
    }
    const QString plotPath = QDir(edgeFolder).filePath(QString("edge_slop.png"));
    QImage(plot.data, plot.cols, plot.rows, (int)plot.step, QImage::Format_BGR888).save(plotPath);

    QJsonObject json;
    json["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["scan"] = edgeScan;
    json["edge"] = edgeName;
    json["y_mm"] = edgeY;
    json["model_x_mm"] = edgeModelX;
    json["x_arriving_moving_left_mm"] = xl;
    json["x_arriving_moving_right_mm"] = xr;
    json["x_slop_mm"] = slop;
    json["per_last_x_sign_mm"] = 0.5 * slop;   // add lastX * this to the midpoint of the two
    json["converged"] = converged;
    json["tolerance_mm"] = edgeTolerance;
    json["away_mm"] = edgeAway;
    json["short_mm"] = edgeShort;
    json["single_moves"] = QJsonArray{ edgeSingles[0], edgeSingles[1] };
    QJsonArray onLeft, onRight;
    for (double x : edgeOnEdge[0]) {
        onLeft.append(x);
    }
    for (double x : edgeOnEdge[1]) {
        onRight.append(x);
    }
    json["on_edge_x_moving_left_mm"] = onLeft;
    json["on_edge_x_moving_right_mm"] = onRight;
    QFile jsonFile(QDir(edgeFolder).filePath(QString("edge_slop.json")));
    if (jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        jsonFile.write(QJsonDocument(json).toJson());
    }

    endEdgeSlop(QString("X slop on the %1 edge: %2 mm (arriving moving left the hole is on the edge at X %3 mm, moving right at X %4 mm). %5 "
                        "Results are in %6")
                    .arg(edgeName).arg(slop, 0, 'f', 3).arg(xl, 0, 'f', 3).arg(xr, 0, 'f', 3)
                    .arg(converged ? QString("Single moves now land within %1 mm from both sides.").arg(edgeTolerance, 0, 'f', 3)
                                   : QString("Single moves didn't all land within %1 mm.").arg(edgeTolerance, 0, 'f', 3))
                    .arg(QDir::toNativeSeparators(edgeFolder)));
    const QImage shown(plotPath);
    if (!shown.isNull()) {
        LAUImageInspectDialog *inspect = new LAUImageInspectDialog(shown, QString("X Slop"), this);
        inspect->setAttribute(Qt::WA_DeleteOnClose);
        inspect->show();
    }
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
