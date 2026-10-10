#include "laueyeonedialog.h"
#include "lauaboutdialog.h"

#include <QDebug>
#include <QMenu>
#include <QFile>
#include <QMenuBar>
#include <QSettings>
#include <QFileInfo>
#include <QMessageBox>
#include <QTextStream>
#include <QStandardPaths>

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
LAUEyeOneDialog::LAUEyeOneDialog(QWidget *parent) : QDialog(parent), device(nullptr), customPlot(nullptr), statusLabel(nullptr), calibrateButton(nullptr), tableWidget(nullptr), contextMenu(nullptr)
{
    this->setLayout(new QVBoxLayout());
    this->layout()->setContentsMargins(6, 6, 6, 6);
    this->setWindowTitle(QString("Dr. Lau's Eye-One Tool"));
    this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    tableWidget = new LAUSpectralTableWidget(this);
    tableWidget->setWindowFlags(Qt::Tool);

    QMenuBar *menuBar = new QMenuBar(this);
    menuBar->setNativeMenuBar(true);

    QMenu *menu = new QMenu(QString("Help"));
    menu->addAction(QString("About"), QKeySequence(Qt::CTRL | Qt::Key_A), this, SLOT(onActionAboutBox()));
    menuBar->addMenu(menu);
    this->layout()->setMenuBar(menuBar);

    // DEVICE STATUS
    QGroupBox *deviceBox = new QGroupBox(QString("i1Pro (SDK %1)").arg(LAUi1ProDevice::sdkVersion()));
    deviceBox->setLayout(new QVBoxLayout());
    deviceBox->layout()->setContentsMargins(6, 6, 6, 6);
    statusLabel = new QLabel();
    deviceBox->layout()->addWidget(statusLabel);
    this->layout()->addWidget(deviceBox);

    QGroupBox *box = new QGroupBox(QString("Measurement"));
    box->setLayout(new QVBoxLayout());
    box->layout()->setContentsMargins(6, 6, 6, 6);
    this->layout()->addWidget(box);

    // CREATE A MENU ACTION FOR DISPLAYING THE SCAN HISTORY
    contextMenu = new QMenu();
    QAction *action = new QAction(QString("Show scan history..."), nullptr);
    action->setCheckable(false);
    connect(action, SIGNAL(triggered()), this, SLOT(onContextMenuTriggered()));
    contextMenu->addAction(action);

    customPlot = new QCustomPlot();
    connect(customPlot, SIGNAL(mousePress(QMouseEvent *)), this, SLOT(onMousePressEvent(QMouseEvent *)));
    customPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    customPlot->setMinimumSize(640, 200);
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue));
    customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20)));
    customPlot->xAxis->setLabel("wavelength (nm)");
    customPlot->yAxis->setLabel("reflectance");
    customPlot->xAxis->setRange(LAUColor::wavelength(0), LAUColor::wavelength(LAU_SPECTRUM_SIZE - 1));
    customPlot->yAxis->setRange(0.0, 1.0);
    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    box->layout()->addWidget(customPlot);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
    this->layout()->addWidget(buttonBox);

    calibrateButton = new QPushButton(QString("Calibrate"));
    buttonBox->addButton(calibrateButton, QDialogButtonBox::ActionRole);
    connect(calibrateButton, SIGNAL(clicked()), this, SLOT(onCalibrateButtonClicked()));

    // SDK EVENTS ARRIVE ON THE SDK'S THREAD; QUEUE THEM ONTO THE GUI THREAD
    device = new LAUi1ProDevice(this);
    device->setMode(LAUi1ProDevice::ModeSpot);
    connect(device, &LAUi1ProDevice::buttonPressed, this, &LAUEyeOneDialog::onButtonClicked, Qt::QueuedConnection);
    connect(device, &LAUi1ProDevice::deviceArrived, this, &LAUEyeOneDialog::onDeviceConnected, Qt::QueuedConnection);
    connect(device, &LAUi1ProDevice::deviceDeparted, this, &LAUEyeOneDialog::onDeviceDisconnected, Qt::QueuedConnection);

    // ASSUME A DEVICE IS AVAILABLE AND TRY TO CONNECT IT
    onDeviceConnected();
    onUpdatePlot();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
LAUEyeOneDialog::~LAUEyeOneDialog()
{
    if (contextMenu) {
        delete contextMenu;
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUEyeOneDialog::onActionAboutBox()
{
    LAUAboutDialog dialog(this);
    dialog.exec();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::updateStatus()
{
    if (!device->isOpen()) {
        statusLabel->setText(errorString.isEmpty() ? QString("No device connected.") : errorString);
        calibrateButton->setEnabled(false);
        return;
    }

    int seconds = device->secondsUntilCalibrationExpires();
    QString calibration = (seconds > 0) ? QString("calibrated, expires in %1 min").arg(seconds / 60) : QString("not calibrated");
    statusLabel->setText(QString("Serial %1, revision %2, zebra sensor: %3\nSpot mode %4.")
                         .arg(device->serialNumber())
                         .arg(device->hardwareRevision())
                         .arg(device->hasZebraRulerSensor() ? QString("yes") : QString("no"))
                         .arg(calibration));
    calibrateButton->setEnabled(true);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::onDeviceDisconnected()
{
    device->close();
    errorString = QString("Device disconnected.");
    updateStatus();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::onDeviceConnected()
{
    if (device->open()) {
        errorString.clear();
    } else {
        errorString = device->error();
    }
    updateStatus();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::onCalibrateButtonClicked()
{
    int ret = QMessageBox::question(this, QString("Calibration"),
                                    QString("This will calibrate the i1Pro in spot mode.\n\n"
                                            "Please ensure:\n"
                                            "\xe2\x80\xa2 The device is seated on its white calibration tile\n"
                                            "\xe2\x80\xa2 The white tile is clean\n\n"
                                            "Start calibration now?"),
                                    QMessageBox::Yes | QMessageBox::No);
    if (ret != QMessageBox::Yes) {
        return;
    }

    if (device->calibrate()) {
        errorString.clear();
        QMessageBox::information(this, QString("Calibration"), QString("Calibration completed successfully.\n\nYou can now take measurements by pressing the device button."));
    } else {
        errorString = device->error();
        QMessageBox::critical(this, QString("Calibration Error"), QString("Unable to calibrate the device.\n\n%1").arg(errorString));
    }
    updateStatus();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::onButtonClicked()
{
    LAUColorPatch reading;
    if (!device->measureSpot(reading)) {
        errorString = device->error();
        qCritical() << "Measurement failed:" << errorString;
        if (!device->isCalibrated()) {
            QMessageBox::warning(this, QString("Measurement"), QString("The device must be calibrated before measuring.\n\nPlease press Calibrate first."));
        }
        return;
    }

    patch = reading;
    errorString.clear();
    onUpdatePlot();
    updateStatus();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::onUpdatePlot()
{
    QVector<double> x(LAU_SPECTRUM_SIZE), y(LAU_SPECTRUM_SIZE);
    double yMin = 0.0, yMax = 1.0;
    for (int n = 0; n < LAU_SPECTRUM_SIZE; n++) {
        x[n] = static_cast<double>(LAUColor::wavelength(n));
        y[n] = static_cast<double>(patch.spectrum[n]);
        yMin = qMin(yMin, y[n]);
        yMax = qMax(yMax, y[n]);
    }

    customPlot->graph(0)->setData(x, y);
    customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 8));
    customPlot->yAxis->setRange(yMin, yMax);
    customPlot->yAxis2->setRange(yMin, yMax);
    customPlot->replot();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void LAUEyeOneDialog::accept()
{
    QSettings settings;
    QString directory = settings.value("LAUEyeOneDialog::lastUsedDirectory", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    QString filename = QFileDialog::getSaveFileName(this, QString("Save measurement to disk (*.csv)"), directory, QString("*.csv"));
    if (filename.isNull()) {
        return;
    }
    if (!filename.toLower().endsWith(QString(".csv"))) {
        filename = QString("%1.csv").arg(filename);
    }
    settings.setValue(QString("LAUEyeOneDialog::lastUsedDirectory"), QFileInfo(filename).absolutePath());

    QFile file(filename);
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream textStream(&file);
        for (int n = 0; n < LAU_SPECTRUM_SIZE; n++) {
            textStream << LAUColor::wavelength(n) << ", " << static_cast<double>(patch.spectrum[n]) << Qt::endl;
        }
        file.close();
    }
    QDialog::accept();
}
