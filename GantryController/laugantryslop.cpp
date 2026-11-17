/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Measure slop: the i1's holder can turn a little on its bracket, so where its hole ends up depends on the way the
// sliders were moving when they stopped.  The camera rides on the carriage, not the holder, so it sees that directly:
// over plain paper the hole is a bright disc, and LAUHoleView::findHole says where its centre is.
//
// One spot on plain paper, in from a corner of the last page found, is visited from each of the twelve hours of a
// clock (in slider space: 12 o'clock is +Y, the rail pair; 3 o'clock is +X, the single rail), a few rounds over.  For
// each hour the sliders go out to that hour, the given distance away, then back to the spot, settle, and the hole is
// found.  The hole's offset from its average is turned into the correction the sliders need, in slider mm, and
// plotted around a clock face; a fit of that correction to the sign of each slider's last move gives the slop.
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
    const auto fits = [&](int k, double in, double out) {
        double single = 0.0, pair = 0.0;
        spotFor(k, in, &single, &pair);
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
    QLabel *label = new QLabel(QString("The sliders bring the i1's hole back to one spot on plain paper beside a corner of the page found in scan %1, "
                                       "from each of the twelve hours of a clock, and the camera sees where the hole lands each time.\n\n"
                                       "Take the i1 Pro out of its holder first so the camera sees the paper through the hole, and don't move the page.")
                                   .arg(slopScan));
    label->setWordWrap(true);
    layout->addWidget(label);
    QFormLayout *form = new QFormLayout();
    QComboBox *corner = new QComboBox();
    corner->addItems(names);
    corner->setCurrentIndex(std::clamp(settings.value(QString("LAUGantryMainWindow::slopCorner"), 0).toInt(), 0, 3));
    QDoubleSpinBox *inset = new QDoubleSpinBox(), *distance = new QDoubleSpinBox(), *rounds = new QDoubleSpinBox();
    inset->setRange(8.0, 60.0);
    inset->setSuffix(QString(" mm"));
    inset->setValue(settings.value(QString("LAUGantryMainWindow::slopInset"), 20.0).toDouble());
    distance->setRange(0.5, 50.0);
    distance->setSuffix(QString(" mm"));
    distance->setValue(settings.value(QString("LAUGantryMainWindow::slopDistance"), 10.0).toDouble());
    rounds->setRange(1, 10);
    rounds->setDecimals(0);
    rounds->setValue(settings.value(QString("LAUGantryMainWindow::slopRounds"), 3).toInt());
    form->addRow(QString("Corner"), corner);
    form->addRow(QString("Spot, in from both edges"), inset);
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
        int first = -1;
        for (int k = 0; k < 4; k++) {
            const bool ok = fits(k, inset->value(), distance->value());
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
                                 : QString("Every corner is too close to the edge of the travel for this spot and approach. "
                                           "Move the spot further in, approach from closer, or widen the limit sliders."));
        none->setVisible(first < 0);
    };
    connect(inset, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dialog, refresh);
    connect(distance, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dialog, refresh);
    refresh();
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    const int k = corner->currentIndex();
    if (!fits(k, inset->value(), distance->value())) {
        return;
    }
    settings.setValue(QString("LAUGantryMainWindow::slopCorner"), k);
    settings.setValue(QString("LAUGantryMainWindow::slopInset"), inset->value());
    settings.setValue(QString("LAUGantryMainWindow::slopDistance"), distance->value());
    settings.setValue(QString("LAUGantryMainWindow::slopRounds"), (int)rounds->value());
    slopCorner = names[k];
    slopDistance = distance->value();
    slopRounds = (int)rounds->value();

    spotFor(k, inset->value(), &slopSingle, &slopPair);

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
    homeDialog->open();
    slopNextMove();
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
    cv::Point2d aperture = holeView.calibratedAperture();
    int seen = 0;
    cv::Point2d sum(0, 0);
    for (const SlopShot &s : slopShots) {
        if (s.ok) {
            sum += s.aperture;
            seen++;
        }
    }
    if (seen) {
        aperture = sum * (1.0 / seen);
    }
    LAUHoleView::Hole hole;
    QString why;
    shot.ok = holeView.findHole(gray, pair, single, aperture + holeView.boardAxis(0) * pair + holeView.boardAxis(1) * single, &hole, &why);
    shot.aperture = hole.aperture;
    shot.note = why;
    slopShots << shot;

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

    // each shot's hole offset from the average, as the slider move that would undo it: ax * dPair + ay * dSingle = -offset
    cv::Point2d mean(0, 0);
    for (const SlopShot &s : good) {
        mean += s.aperture * (1.0 / good.count());
    }
    const cv::Point2d ax = holeView.boardAxis(0), ay = holeView.boardAxis(1);
    const double det = ax.x * ay.y - ay.x * ax.y;
    const auto correctionOf = [&](const SlopShot &s) {
        const cv::Point2d m = mean - s.aperture;
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
    const QString folder = QDir(scanBase).filePath(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss")) + QString("_slop"));
    QDir().mkpath(folder);
    QFile csv(QDir(folder).filePath(QString("slop.csv")));
    if (csv.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream out(&csv);
        out << "round,hour,from_x_mm,from_y_mm,last_x_sign,last_y_sign,found,hole_at_rails_zero_board_x_mm,hole_at_rails_zero_board_y_mm,"
               "correction_x_mm,correction_y_mm,note\n";
        for (const SlopShot &s : slopShots) {
            const cv::Point2d d = hourDirection(s.hour) * slopDistance;
            const cv::Point2d c = s.ok ? correctionOf(s) : cv::Point2d(0, 0);
            out << s.round << ',' << s.hour << ',' << QString::number(d.x, 'f', 3) << ',' << QString::number(d.y, 'f', 3) << ',' << s.lastX << ','
                << s.lastY << ',' << (s.ok ? 1 : 0) << ',' << QString::number(s.aperture.x, 'f', 4) << ',' << QString::number(s.aperture.y, 'f', 4)
                << ',' << (s.ok ? QString::number(c.x, 'f', 4) : QString()) << ',' << (s.ok ? QString::number(c.y, 'f', 4) : QString()) << ",\""
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
    const QStringList lines{
        QString("Slider correction to add (mm), by the hour the hole came back from"),
        QString("%1 corner of scan %2, %3 mm approach, %4 of %5 shots found").arg(slopCorner, slopScan).arg(slopDistance, 0, 'f', 1).arg(n).arg(slopShots.count()),
        QString("X: %1 %2 %3 x lastX %4 %5 x lastY   (rms left %6)").arg(fx.at<double>(0), 0, 'f', 3).arg(fx.at<double>(1) < 0 ? "-" : "+")
            .arg(std::abs(fx.at<double>(1)), 0, 'f', 3).arg(fx.at<double>(2) < 0 ? "-" : "+").arg(std::abs(fx.at<double>(2)), 0, 'f', 3).arg(rx, 0, 'f', 3),
        QString("Y: %1 %2 %3 x lastX %4 %5 x lastY   (rms left %6)").arg(fy.at<double>(0), 0, 'f', 3).arg(fy.at<double>(1) < 0 ? "-" : "+")
            .arg(std::abs(fy.at<double>(1)), 0, 'f', 3).arg(fy.at<double>(2) < 0 ? "-" : "+").arg(std::abs(fy.at<double>(2)), 0, 'f', 3).arg(ry, 0, 'f', 3),
        QString("slop between +X and -X approaches: %1 mm").arg(2.0 * std::abs(fx.at<double>(1)), 0, 'f', 3),
    };
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
    QFile jsonFile(QDir(folder).filePath(QString("slop.json")));
    if (jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        jsonFile.write(QJsonDocument(json).toJson());
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
