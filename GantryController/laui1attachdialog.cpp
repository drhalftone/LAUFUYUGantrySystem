/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laui1attachdialog.h"
#include "lauglimageview.h"

#include <cmath>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <opencv2/imgproc.hpp>

namespace
{
    const int REFERENCE_FRAMES = 8;              // averaged into the empty cradle's look
    const double ATTACHED_CHANGE = 0.15;         // the cradle region differs from empty by this share of its brightness
    const double STEADY_CHANGE = 0.03;           // and from the frame before by less than this
    const int STEADY_FRAMES = 8;                 // for this many frames in a row
    const int SAMPLE_STEP = 4;
}

LAUi1AttachDialog::LAUi1AttachDialog(const cv::Mat &cradle, QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Attach the i1 Pro"));
    setModal(true);
    resize(900, 720);

    QVBoxLayout *layout = new QVBoxLayout(this);
    view = new LAUImageView();
    layout->addWidget(view, 1);
    status = new QLabel();
    status->setWordWrap(true);
    QFont f = status->font();
    f.setPointSize(f.pointSize() + 3);
    status->setFont(f);
    layout->addWidget(status);

    QHBoxLayout *buttons = new QHBoxLayout();
    QPushButton *overrideButton = new QPushButton(QString("Measure without the camera's confirmation..."));
    connect(overrideButton, &QPushButton::clicked, this, &LAUi1AttachDialog::onOverride);
    okButton = new QPushButton(QString("OK - start measuring"));
    okButton->setEnabled(false);
    okButton->setDefault(true);
    connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
    QPushButton *cancelButton = new QPushButton(QString("Cancel"));
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    buttons->addWidget(overrideButton);
    buttons->addStretch();
    buttons->addWidget(okButton);
    buttons->addWidget(cancelButton);
    layout->addLayout(buttons);

    mask = cradle.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    LAUViewOverlayLayer outline;
    outline.color = QColor(255, 200, 0);
    outline.width = 2.0;
    outline.label = QString("i1 cradle");
    for (const std::vector<cv::Point> &c : contours) {
        QPolygonF polygon;
        for (const cv::Point &p : c) {
            polygon << QPointF(p.x, p.y);
        }
        polygon << polygon.first();
        outline.paths << polygon;
    }
    LAUViewOverlay overlay;
    overlay.layers << outline;
    view->setOverlay(overlay);

    setStatus(QString("Looking at the empty cradle..."), QString("gray"));
}

void LAUi1AttachDialog::setStatus(const QString &text, const QString &color)
{
    status->setText(text);
    status->setStyleSheet(QString("color: %1;").arg(color));
}

void LAUi1AttachDialog::onFrame(cv::Mat raw16)
{
    if (raw16.empty()) {
        return;
    }
    view->setFrame(raw16);

    if (samples.empty()) {
        cv::Mat fitted = mask;
        if (fitted.size() != raw16.size()) {
            cv::resize(mask, fitted, raw16.size(), 0, 0, cv::INTER_NEAREST);
        }
        for (int r = 0; r < fitted.rows; r += SAMPLE_STEP) {
            for (int c = 0; c < fitted.cols; c += SAMPLE_STEP) {
                if (fitted.at<uchar>(r, c)) {
                    samples.push_back(r * raw16.cols + c);
                }
            }
        }
        if (samples.empty()) {
            setStatus(QString("The platen calibration doesn't say where the cradle is, so the camera can't check for the i1."), QString("#c62828"));
            return;
        }
        reference.assign(samples.size(), 0.0);
    }

    std::vector<double> now(samples.size());
    const quint16 *pixels = raw16.ptr<quint16>(0);
    for (size_t k = 0; k < samples.size(); k++) {
        now[k] = pixels[samples[k]];
    }

    if (referenceFrames < REFERENCE_FRAMES) {
        for (size_t k = 0; k < now.size(); k++) {
            reference[k] += now[k] / REFERENCE_FRAMES;
        }
        if (++referenceFrames == REFERENCE_FRAMES) {
            setStatus(QString("Now attach the i1 Pro to the carriage, in its cradle."), QString("black"));
        }
        previous = now;
        return;
    }

    double level = 0.0, fromEmpty = 0.0, fromLast = 0.0;
    for (size_t k = 0; k < now.size(); k++) {
        level += reference[k];
        fromEmpty += std::abs(now[k] - reference[k]);
        fromLast += std::abs(now[k] - previous[k]);
    }
    level = std::max(level, 1.0);
    lastChange = fromEmpty / level;
    const bool steady = fromLast / level < STEADY_CHANGE;
    previous = now;

    steadyFrames = (lastChange > ATTACHED_CHANGE && steady) ? steadyFrames + 1 : 0;
    const bool seen = steadyFrames >= STEADY_FRAMES || (attached && lastChange > ATTACHED_CHANGE);
    if (seen != attached) {
        attached = seen;
        okButton->setEnabled(attached);
        if (attached) {
            setStatus(QString("The camera sees the i1 Pro in its cradle. Press OK to start measuring."), QString("#2e7d32"));
        } else {
            setStatus(QString("Now attach the i1 Pro to the carriage, in its cradle."), QString("black"));
        }
    } else if (!attached && lastChange > ATTACHED_CHANGE) {
        setStatus(QString("Something is in the cradle: waiting for it to hold still..."), QString("#ef6c00"));
    }
}

void LAUi1AttachDialog::onOverride()
{
    const QMessageBox::StandardButton answer = QMessageBox::question(this, QString("Attach the i1 Pro"),
        QString("The camera hasn't confirmed that the i1 Pro is in its cradle.\n\nMeasure anyway? Readings taken without it seated will be wrong."),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer == QMessageBox::Yes) {
        accept();
    }
}
