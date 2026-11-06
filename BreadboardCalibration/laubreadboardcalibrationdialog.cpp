/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laubreadboardcalibrationdialog.h"

#include <QDir>
#include <QUrl>
#include <QPainter>
#include <QDateTime>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDesktopServices>
#include <QStandardPaths>

void LAUImageView::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.fillRect(rect(), Qt::black);
    if (image.isNull()) {
        p.setPen(Qt::gray);
        p.drawText(rect(), Qt::AlignCenter, QString("Waiting for the camera..."));
        return;
    }
    const QSize s = image.size().scaled(size(), Qt::KeepAspectRatio);
    const QRect r(QPoint((width() - s.width()) / 2, (height() - s.height()) / 2), s);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.drawImage(r, image);
}

LAUBreadboardCalibrationDialog::LAUBreadboardCalibrationDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Breadboard Camera Calibration"));
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint);
    resize(1280, 940);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    // which picture to show
    QHBoxLayout *views = new QHBoxLayout();
    viewButtons = new QButtonGroup(this);
    const QStringList names = { QString("Live camera"), QString("Holes found"), QString("Lens corrected"),
                                QString("Lines into the distance"), QString("Vanishing point") };
    for (int n = 0; n < names.count(); n++) {
        QPushButton *b = new QPushButton(names.at(n));
        b->setCheckable(true);
        b->setEnabled(n == ViewLive);
        b->setMinimumHeight(32);
        viewButtons->addButton(b, n);
        views->addWidget(b);
    }
    viewButtons->button(ViewLive)->setChecked(true);
    connect(viewButtons, &QButtonGroup::idClicked, this, &LAUBreadboardCalibrationDialog::onViewChanged);
    layout->addLayout(views);

    view = new LAUImageView();
    layout->addWidget(view, 1);

    camera = new QLabel(QString("Connecting to the camera..."));
    camera->setStyleSheet(QString("color: gray;"));
    layout->addWidget(camera);

    status = new QLabel(QString("Point the camera at the breadboard and press Calibrate."));
    status->setWordWrap(true);
    QFont f = status->font();
    f.setPointSize(f.pointSize() + 3);
    status->setFont(f);
    layout->addWidget(status);

    QHBoxLayout *buttons = new QHBoxLayout();
    calibrateButton = new QPushButton(QString("Calibrate"));
    calibrateButton->setMinimumSize(180, 44);
    QFont bf = calibrateButton->font();
    bf.setPointSize(bf.pointSize() + 3);
    bf.setBold(true);
    calibrateButton->setFont(bf);
    calibrateButton->setEnabled(false);
    connect(calibrateButton, &QPushButton::clicked, this, &LAUBreadboardCalibrationDialog::onCalibrateClicked);
    folderButton = new QPushButton(QString("Open results folder"));
    folderButton->setMinimumHeight(44);
    folderButton->setEnabled(false);
    connect(folderButton, &QPushButton::clicked, this, &LAUBreadboardCalibrationDialog::onOpenFolderClicked);
    QPushButton *closeButton = new QPushButton(QString("Close"));
    closeButton->setMinimumHeight(44);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    buttons->addWidget(calibrateButton);
    buttons->addWidget(folderButton);
    buttons->addStretch();
    buttons->addWidget(closeButton);
    layout->addLayout(buttons);

    // calibration runs in its own thread so the live picture keeps moving
    calibration = new LAUBreadboardCalibration();
    calibration->moveToThread(&calibrationThread);
    connect(&calibrationThread, &QThread::finished, calibration, &QObject::deleteLater);
    connect(this, &LAUBreadboardCalibrationDialog::emitCalibrate, calibration, &LAUBreadboardCalibration::onCalibrate, Qt::QueuedConnection);
    connect(calibration, &LAUBreadboardCalibration::emitResult, this, &LAUBreadboardCalibrationDialog::onResult, Qt::QueuedConnection);
    calibrationThread.start();

    grabber = new LAUBaslerGrabber(this);
    connect(grabber, &LAUBaslerGrabber::emitPreview, this, &LAUBreadboardCalibrationDialog::onPreview, Qt::QueuedConnection);
    connect(grabber, &LAUBaslerGrabber::emitAveragedFrame, this, &LAUBreadboardCalibrationDialog::onAveragedFrame, Qt::QueuedConnection);
    connect(grabber, &LAUBaslerGrabber::emitError, this, &LAUBreadboardCalibrationDialog::onCameraError, Qt::QueuedConnection);
    connect(grabber, &LAUBaslerGrabber::emitCameraName, this, [this](QString name) {
        camera->setText(QString("Camera: %1").arg(name));
    });
    grabber->start();
}

LAUBreadboardCalibrationDialog::~LAUBreadboardCalibrationDialog()
{
    grabber->stop();
    calibrationThread.quit();
    calibrationThread.wait();
}

void LAUBreadboardCalibrationDialog::setStatus(const QString &text)
{
    status->setText(text);
}

void LAUBreadboardCalibrationDialog::onPreview(QImage image, double exposureMs)
{
    liveImage = image;
    if (!busy) {
        calibrateButton->setEnabled(true);
    }
    camera->setText(camera->text().section(QString("   "), 0, 0) + QString("   exposure %1 ms").arg(exposureMs, 0, 'f', 1));
    if (viewButtons->checkedId() == ViewLive) {
        view->setImage(liveImage);
    }
}

void LAUBreadboardCalibrationDialog::onCameraError(QString message)
{
    camera->setText(message);
    setStatus(QString("The camera isn't available. Check its USB cable, and that no other program is using it, "
                      "then reopen this window."));
    calibrateButton->setEnabled(false);
}

void LAUBreadboardCalibrationDialog::onCalibrateClicked()
{
    busy = true;
    calibrateButton->setEnabled(false);
    setStatus(QString("Hold still — taking the pictures..."));
    grabber->requestAverage(8);
}

void LAUBreadboardCalibrationDialog::onAveragedFrame(cv::Mat raw16, double exposureMs)
{
    setStatus(QString("Finding the holes and measuring the lens..."));
    emit emitCalibrate(raw16, exposureMs);
}

void LAUBreadboardCalibrationDialog::onResult(LAUBreadboardCalibrationResult r)
{
    busy = false;
    calibrateButton->setEnabled(true);
    if (!r.ok) {
        setStatus(r.message);
        return;
    }
    result = r;
    const QString base = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString folder = QDir(base).filePath(QString("BreadboardCalibration/%1").arg(QDateTime::currentDateTime().toString(QString("yyyyMMdd_HHmmss"))));
    QString err;
    if (LAUBreadboardCalibration::save(result, folder, &err)) {
        savedFolder = folder;
        folderButton->setEnabled(true);
        setStatus(result.message + QString("\nSaved to %1").arg(QDir::toNativeSeparators(folder)));
    } else {
        setStatus(result.message + QString("\nCouldn't save the results: %1").arg(err));
    }
    for (int n = ViewHoles; n <= ViewVanishing; n++) {
        viewButtons->button(n)->setEnabled(true);
    }
    viewButtons->button(ViewCorrected)->setChecked(true);
    showView();
}

void LAUBreadboardCalibrationDialog::onViewChanged(int)
{
    showView();
}

void LAUBreadboardCalibrationDialog::showView()
{
    switch (viewButtons->checkedId()) {
    case ViewHoles:
        view->setImage(result.holes);
        break;
    case ViewCorrected:
        view->setImage(result.undistorted);
        break;
    case ViewLines:
        view->setImage(result.lines);
        break;
    case ViewVanishing:
        view->setImage(result.vanishing);
        break;
    default:
        view->setImage(liveImage);
    }
}

void LAUBreadboardCalibrationDialog::onOpenFolderClicked()
{
    if (!savedFolder.isEmpty()) {
        QDesktopServices::openUrl(QUrl::fromLocalFile(savedFolder));
    }
}
