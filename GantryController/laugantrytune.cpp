/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// Fine-tune page corners: the rails put the i1 holder's hole over each corner of the page found by the last scan,
// the user nudges them until the corner sits in the middle of the hole and confirms it, and the rails' positions at
// the confirmed corners give a correction from the model's rail positions to the true ones.  It is saved with that
// scan's platen_mosaic.json, so LAUPlatenGeometry::railsFor applies it to every point of that picture.
//
#include "laugantrymainwindow.h"
#include "laucornertunedialog.h"

#include <cmath>
#include <complex>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QMessageBox>
#include <QPushButton>
#include <QJsonDocument>

namespace
{
    const double LARGE_NUDGE_MM = 5.0;       // more than this at a corner suggests the wrong corner was centred
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

    // as the merged picture shows them: page.json lists them clockwise from the picture's top-left
    const QStringList names{ QString("top-left"), QString("top-right"), QString("bottom-right"), QString("bottom-left") };
    tuneCorners.clear();
    QStringList unreachable;
    for (int k = 0; k < 4; k++) {
        TuneCorner c;
        c.name = names[k];
        c.board = cv::Point2d(corners[k].toArray().at(0).toDouble(), corners[k].toArray().at(1).toDouble());
        tuneGeometry.railsFor(c.board, &c.plannedSingle, &c.plannedPair, false);
        tuneGeometry.railsFor(c.board, &c.startSingle, &c.startPair, true);
        c.single = c.startSingle;
        c.pair = c.startPair;
        c.reachable = tuneGeometry.reachable(c.startSingle, c.startPair);
        if (!c.reachable) {
            unreachable << c.name;
        }
        tuneCorners << c;
    }
    if (unreachable.count() == 4) {
        QMessageBox::warning(this, title, QString("None of the page's corners is within the i1's reach. Move the page toward the middle of the platen and scan it again."));
        return;
    }

    QString text = QString("The sliders will put the i1's hole over each corner of the %1 x %2 mm page found in scan %3, one corner at a time.\n\n"
                           "Before starting, take the i1 Pro out of its holder so the camera sees the paper through the hole, and don't move the page.\n\n"
                           "At each corner, zoom the live picture on the hole and use the arrows to move the paper until its corner is in the middle "
                           "of the hole. Then press Centered to go on to the next corner.")
                       .arg(page["width_mm"].toDouble(), 0, 'f', 1).arg(page["length_mm"].toDouble(), 0, 'f', 1).arg(scanName);
    if (!unreachable.isEmpty()) {
        text += QString("\n\nOut of the i1's reach, so skipped: the %1 corner%2.").arg(unreachable.join(QString(", "))).arg(unreachable.count() > 1 ? "s" : "");
    }
    if (QMessageBox::information(this, title, text, QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok) != QMessageBox::Ok) {
        return;
    }

    tuning = true;
    tuneCancelling = false;
    tuneIndex = -1;
    cameraWidget->lockScanActions(true);
    railBox->setEnabled(false);              // the dialog's arrows move the sliders now
    tuneDialog = new LAUCornerTuneDialog(this);
    connect(tuneDialog, &LAUCornerTuneDialog::emitNudge, this, &LAUGantryMainWindow::onTuneNudge);
    connect(tuneDialog, &LAUCornerTuneDialog::emitCentered, this, &LAUGantryMainWindow::onTuneCentered);
    connect(tuneDialog, &LAUCornerTuneDialog::emitSkip, this, &LAUGantryMainWindow::onTuneSkip);
    connect(tuneDialog, &LAUCornerTuneDialog::emitCancel, this, &LAUGantryMainWindow::onTuneCancel);
    tuneDialog->show();
    nextTuneCorner();
}

void LAUGantryMainWindow::nextTuneCorner()
{
    do {
        tuneIndex++;
    } while (tuneIndex < tuneCorners.count() && !tuneCorners[tuneIndex].reachable);
    if (tuneIndex >= tuneCorners.count()) {
        finishTune();
        return;
    }
    goToTuneCorner();
}

void LAUGantryMainWindow::goToTuneCorner()
{
    const TuneCorner &c = tuneCorners[tuneIndex];
    tuneDialog->setBusy(true);
    tuneDialog->setMessage(QString("Moving the hole to the page's %1 corner...").arg(c.name));
    tuneDialog->setNudge(QString());
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
    c.single = railWidget->reportedMM(0);
    c.pair = railWidget->reportedMM(2);

    int number = 0, total = 0;
    for (int k = 0; k < tuneCorners.count(); k++) {
        if (tuneCorners[k].reachable) {
            total++;
            number += (k <= tuneIndex);
        }
    }
    tuneDialog->setBusy(false);
    tuneDialog->setMessage(QString("<b>Corner %1 of %2: the page's %3 corner.</b><br><br>In the live picture, the corner of the paper should be "
                                   "in the middle of the i1's hole. Use the arrows to move the paper until it is, then press <b>Centered</b>.")
                               .arg(number).arg(total).arg(c.name));
    const double ds = c.single - c.startSingle, dp = c.pair - c.startPair;
    tuneDialog->setNudge((std::abs(ds) < 0.005 && std::abs(dp) < 0.005)
                             ? QString("Not nudged yet.")
                             : QString("Nudged so far:\nsingle rail %1 mm\nrail pair %2 mm").arg(ds, 0, 'f', 2).arg(dp, 0, 'f', 2));
}

// Each arrow moves the paper that way in the live picture by the chosen step, measured on the paper
void LAUGantryMainWindow::onTuneNudge(int right, int down)
{
    if (!tuning || railWidget->isHoming()) {
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
    if (!tuning || railWidget->isHoming()) {
        return;
    }
    tuneCorners[tuneIndex].confirmed = true;
    nextTuneCorner();
}

void LAUGantryMainWindow::onTuneSkip()
{
    if (!tuning || railWidget->isHoming()) {
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
    endTune(QString("Fine-tuning the page corners was cancelled. Nothing was changed."));
}

// The correction: from the model's rail positions to the confirmed ones.  Four or three corners give an affine map
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
        endTune(QString("No corner was confirmed, so nothing was changed."));
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
        if (!c.confirmed) {
            rows += QString("<tr><td>%1</td><td colspan=2>%2</td></tr>").arg(c.name, c.reachable ? QString("skipped") : QString("out of reach"));
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
        o["nudge_mm"] = nudge;
        o["left_after_fit_mm"] = left;
        json.append(o);
        rows += QString("<tr><td>%1</td><td align=right>%2 mm</td><td align=right>%3 mm</td></tr>").arg(c.name).arg(nudge, 0, 'f', 2).arg(left, 0, 'f', 2);
    }
    const QString scanName = QFileInfo(QFileInfo(tuneFolder).absolutePath()).fileName();
    QString text = QString("<p>%1 of %2 corners centred.</p><table cellspacing=6><tr><th align=left>Corner</th><th>Nudged by</th><th>Left after the fit</th></tr>%3</table>")
                       .arg(used.count()).arg(tuneCorners.count()).arg(rows);
    if (worstNudge > LARGE_NUDGE_MM) {
        text += QString("<p><b>One corner needed more than %1 mm.</b> Check that the page's own corner was centred, not another mark.</p>").arg(LARGE_NUDGE_MM, 0, 'f', 0);
    }
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
    QJsonObject record;
    record["time"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    record["corners"] = json;
    record["fix_single_pair"] = QJsonArray{ QJsonArray{ F(0, 0), F(0, 1), F(0, 2) }, QJsonArray{ F(1, 0), F(1, 1), F(1, 2) } };
    record["fit"] = (used.count() >= 3) ? QString("affine") : (used.count() == 2) ? QString("similarity") : QString("offset");
    record["worst_nudge_mm"] = worstNudge;
    record["worst_left_after_fit_mm"] = worstLeft;
    QFile out(QDir(tuneFolder).filePath(QString("corner_tune.json")));
    const bool saved = tuneGeometry.save(tuneFolder) && out.open(QIODevice::WriteOnly | QIODevice::Truncate) && out.write(QJsonDocument(record).toJson()) > 0;
    endTune(saved ? QString("Saved the page-corner correction with %1: corners needed up to %2 mm, %3 mm left after the fit.")
                        .arg(QDir::toNativeSeparators(tuneFolder)).arg(worstNudge, 0, 'f', 2).arg(worstLeft, 0, 'f', 2)
                  : QString("Couldn't save the page-corner correction in %1").arg(QDir::toNativeSeparators(tuneFolder)));
}

void LAUGantryMainWindow::endTune(const QString &message)
{
    tuning = false;
    tuneCancelling = false;
    if (tuneDialog) {
        tuneDialog->deleteLater();
        tuneDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->lockScanActions(false);
    cameraWidget->showMessage(message);
}
