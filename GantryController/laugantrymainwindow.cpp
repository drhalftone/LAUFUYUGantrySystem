/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laugantrymainwindow.h"

#include <QProgressBar>
#include <QVBoxLayout>

LAUHomeDialog::LAUHomeDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QString("Calibrate"));
    setModal(true);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setSpacing(12);
    label = new QLabel(QString("Moving the sliders to their home position..."));
    label->setAlignment(Qt::AlignHCenter);
    layout->addWidget(label);

    // a range of 0..0 makes the bar sweep left to right over and over until the dialog closes
    QProgressBar *bar = new QProgressBar();
    bar->setRange(0, 0);
    bar->setTextVisible(false);
    bar->setMinimumWidth(360);
    layout->addWidget(bar);

    abortButton = new QPushButton(QString("Abort"));
    abortButton->setMinimumHeight(44);
    connect(abortButton, &QPushButton::clicked, this, &LAUHomeDialog::reject);
    layout->addWidget(abortButton);
}

void LAUHomeDialog::reject()
{
    abortButton->setEnabled(false);
    label->setText(QString("Stopping the sliders..."));
    emit emitAbort();
}

void LAUHomeDialog::finish()
{
    QDialog::accept();
    deleteLater();
}

LAUGantryMainWindow::LAUGantryMainWindow(QWidget *parent) : QMainWindow(parent)
{
#ifdef FUYU_SIMULATE
    setWindowTitle(QString("Gantry Controller  —  SIMULATION (no rail hardware)"));
#else
    setWindowTitle(QString("Gantry Controller"));
#endif

    QWidget *central = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(central);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);
    setCentralWidget(central);

    QGroupBox *cameraBox = new QGroupBox(QString("Camera"));
    QVBoxLayout *cameraLayout = new QVBoxLayout(cameraBox);
    cameraWidget = new LAUBreadboardCalibrationWidget();
    cameraLayout->addWidget(cameraWidget);
    layout->addWidget(cameraBox, 1);          // the camera takes all the spare height

    //   FMC4030 SDK axis 0 = X, the single rail of the H
    //   FMC4030 SDK axis 2 = Y, the tied-together rail pair (both rails share one controller axis)
    QList<int> axes;
    axes << 0 << 2;

    railBox = new QGroupBox(QString("Enable Sliders"));
    railBox->setCheckable(true);
    railBox->setChecked(false);
    QVBoxLayout *railLayout = new QVBoxLayout(railBox);
    railWidget = new LAUMultiVelmexWidget(axes, nullptr, false);
    railLayout->addWidget(railWidget);
    connect(railBox, &QGroupBox::toggled, this, &LAUGantryMainWindow::onRailsEnabled);
    layout->addWidget(railBox, 0);

    cameraWidget->enableHomeMove();
    connect(cameraWidget, &LAUBreadboardCalibrationWidget::emitGoHome, this, &LAUGantryMainWindow::onGoHome);
    connect(railWidget, &LAUMultiVelmexWidget::emitHomeFinished, this, &LAUGantryMainWindow::onHomeFinished);
}

void LAUGantryMainWindow::onGoHome()
{
    if (!railBox->isChecked() || !railWidget->isControllerStarted()) {
        cameraWidget->onHomeFinished(false, QString("Tick Enable Sliders first, so the sliders can move to their home position."));
        return;
    }

    railWidget->goToHome();
    if (railWidget->isHoming()) {
        railBox->setEnabled(false);          // hands off the sliders until they get there
        homeDialog = new LAUHomeDialog(this);
        connect(homeDialog, &LAUHomeDialog::emitAbort, railWidget, &LAUMultiVelmexWidget::abortHome);
        homeDialog->open();
    }
}

void LAUGantryMainWindow::onHomeFinished(bool ok, QString message)
{
    if (homeDialog) {
        homeDialog->finish();
        homeDialog = nullptr;
    }
    railBox->setEnabled(true);
    cameraWidget->onHomeFinished(ok, message);
}

void LAUGantryMainWindow::onRailsEnabled(bool state)
{
    if (!state || railWidget->isControllerStarted()) {
        return;
    }

    // the dialog saves the IP/port to the QSettings keys the controller reads when it opens the FMC4030
    LAUFuyuConnectDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        railWidget->connectToController();
    } else {
        railBox->setChecked(false);
    }
}
