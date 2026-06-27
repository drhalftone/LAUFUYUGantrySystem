/*********************************************************************************
 *                                                                               *
 * Copyright (c) 2017, Dr. Daniel L. Lau                                         *
 * All rights reserved.                                                          *
 *                                                                               *
 * Redistribution and use in source and binary forms, with or without            *
 * modification, are permitted provided that the following conditions are met:   *
 * 1. Redistributions of source code must retain the above copyright             *
 *    notice, this list of conditions and the following disclaimer.              *
 * 2. Redistributions in binary form must reproduce the above copyright          *
 *    notice, this list of conditions and the following disclaimer in the        *
 *    documentation and/or other materials provided with the distribution.       *
 * 3. All advertising materials mentioning features or use of this software      *
 *    must display the following acknowledgement:                                *
 *    This product includes software developed by the <organization>.            *
 * 4. Neither the name of the <organization> nor the                             *
 *    names of its contributors may be used to endorse or promote products       *
 *    derived from this software without specific prior written permission.      *
 *                                                                               *
 * THIS SOFTWARE IS PROVIDED BY Dr. Daniel L. Lau ''AS IS'' AND ANY              *
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED     *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE        *
 * DISCLAIMED. IN NO EVENT SHALL Dr. Daniel L. Lau BE LIABLE FOR ANY             *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;  *
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND   *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                  *
 *                                                                               *
 *********************************************************************************/

#include "lauvelmexwidget.h"

#include <QDebug>
#include <QtMath>
#include <QLabel>
#include <QThread>
#include <QSettings>
#include <QBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QApplication>
#include <QInputDialog>
#include <QProgressDialog>
#include <QRegularExpression>
#include <QRegularExpressionValidator>

int LAUVelmexWidget::unitIndex = 1;   // 0 = inches, 1 = millimetres (default: mm, FMC4030 is mm-native)
bool LAUVelmexController::velmexControllerIsWaitingWhileBusy = false;
bool LAUVelmexController::velmexRailHasReachedLimitSwitch = false;
QVector4D LAUVelmexWidget::scannerPosition;

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexUserPathOffsetDialog::LAUVelmexUserPathOffsetDialog(int chn, QWidget *parent) : QDialog(parent), numChannels(chn), xSpinBox(nullptr), ySpinBox(nullptr), zSpinBox(nullptr), wSpinBox(nullptr)
{
    this->setLayout(new QVBoxLayout());
    this->layout()->setContentsMargins(6,6,6,6);
    this->setWindowTitle(QString("User Path Offset"));

    QGroupBox *box = new QGroupBox(QString("Offset"));
    box->setLayout(new QFormLayout());
    box->layout()->setContentsMargins(6,6,6,6);
    box->setFixedWidth(400);
    ((QFormLayout*)(box->layout()))->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    if (numChannels > 0){
        xSpinBox = new QDoubleSpinBox();
        xSpinBox->setMinimum(-10.0);
        xSpinBox->setMaximum(10.0);
        xSpinBox->setDecimals(5);
        xSpinBox->setSingleStep(0.1);
        xSpinBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        ((QFormLayout*)(box->layout()))->addRow(QString("X offset:"), xSpinBox);
    }

    if (numChannels > 1){
        ySpinBox = new QDoubleSpinBox();
        ySpinBox->setMinimum(-10.0);
        xSpinBox->setMaximum(10.0);
        ySpinBox->setDecimals(5);
        ySpinBox->setSingleStep(0.1);
        ySpinBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        ((QFormLayout*)(box->layout()))->addRow(QString("Y offset:"), ySpinBox);
    }

    if (numChannels > 2){
        zSpinBox = new QDoubleSpinBox();
        zSpinBox->setMinimum(-10.0);
        zSpinBox->setMaximum(10.0);
        zSpinBox->setDecimals(5);
        zSpinBox->setSingleStep(0.1);
        zSpinBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        ((QFormLayout*)(box->layout()))->addRow(QString("Z offset:"), zSpinBox);
    }

    if (numChannels > 3){
        wSpinBox = new QDoubleSpinBox();
        wSpinBox->setMinimum(-10.0);
        wSpinBox->setMaximum(10.0);
        wSpinBox->setDecimals(5);
        wSpinBox->setSingleStep(0.1);
        wSpinBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        ((QFormLayout*)(box->layout()))->addRow(QString("W offset:"), wSpinBox);
    }
    this->layout()->addWidget(box);

    QSettings settings;
    xSpinBox->setValue(settings.value("LAUUserPathOffsetDialog::xSpinBox", 0.0).toDouble());
    ySpinBox->setValue(settings.value("LAUUserPathOffsetDialog::ySpinBox", 0.0).toDouble());
    zSpinBox->setValue(settings.value("LAUUserPathOffsetDialog::zSpinBox", 0.0).toDouble());
    wSpinBox->setValue(settings.value("LAUUserPathOffsetDialog::wSpinBox", 0.0).toDouble());

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
    this->layout()->addWidget(buttonBox);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexUserPathOffsetDialog::accept()
{
    QSettings settings;
    settings.setValue("LAUUserPathOffsetDialog::xSpinBox", xSpinBox->value());
    settings.setValue("LAUUserPathOffsetDialog::ySpinBox", ySpinBox->value());
    settings.setValue("LAUUserPathOffsetDialog::zSpinBox", zSpinBox->value());
    settings.setValue("LAUUserPathOffsetDialog::wSpinBox", wSpinBox->value());

    QDialog::accept();
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexDialog::LAUVelmexDialog(int dims, QDialog *parent) : QDialog(parent), velmexWidget(nullptr), velmexMultiWidget(nullptr)
{
    this->setLayout(new QVBoxLayout());
#ifdef Q_OS_WIN
    this->layout()->setContentsMargins(6, 6, 6, 6);
#else
    this->layout()->setContentsMargins(6, 6, 6, 6);
#endif
    this->setWindowTitle(QString("Velmex Rail Controller"));

    if (dims == 1){
        velmexWidget = new LAUVelmexWidget(0, nullptr, parent);
        connect(velmexWidget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onRelayTriggerScanner(float,int,int)), Qt::QueuedConnection);
        this->layout()->addWidget(velmexWidget);
    } else {
        velmexMultiWidget = new LAUMultiVelmexWidget(dims, parent);
        connect(velmexMultiWidget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onRelayTriggerScanner(float,int,int)), Qt::QueuedConnection);
        this->layout()->addWidget(velmexMultiWidget);
    }
    ((QVBoxLayout *)(this->layout()))->addStretch();

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    this->layout()->addWidget(buttonBox);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexDialog::LAUVelmexDialog(QList<int> channels, QDialog *parent) : QDialog(parent), velmexWidget(nullptr), velmexMultiWidget(nullptr)
{
    this->setLayout(new QVBoxLayout());
#ifdef Q_OS_WIN
    this->layout()->setContentsMargins(6, 6, 6, 6);
#else
    this->layout()->setContentsMargins(6, 6, 6, 6);
#endif
    this->setWindowTitle(QString("Velmex Rail Controller"));

    if (channels.count() == 1){
        velmexWidget = new LAUVelmexWidget(channels.first(), nullptr, parent);
        connect(velmexWidget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onRelayTriggerScanner(float,int,int)), Qt::QueuedConnection);
        this->layout()->addWidget(velmexWidget);
    } else {
        velmexMultiWidget = new LAUMultiVelmexWidget(channels, parent);
        connect(velmexMultiWidget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onRelayTriggerScanner(float,int,int)), Qt::QueuedConnection);
        this->layout()->addWidget(velmexMultiWidget);
    }
    ((QVBoxLayout *)(this->layout()))->addStretch();

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    this->layout()->addWidget(buttonBox);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexDialog::LAUVelmexDialog(LAUMultiVelmexWidget *widget, QDialog *parent) : QDialog(parent), velmexWidget(nullptr), velmexMultiWidget(widget)
{
    this->setLayout(new QVBoxLayout());
#ifdef Q_OS_WIN
    this->layout()->setContentsMargins(6, 6, 6, 6);
#else
    this->layout()->setContentsMargins(6, 6, 6, 6);
#endif
    this->setWindowTitle(QString("Velmex Rail Controller"));

    if (velmexMultiWidget){
        this->layout()->addWidget(velmexMultiWidget);
    } else {
        velmexMultiWidget = new LAUMultiVelmexWidget(0, parent);
        this->layout()->addWidget(velmexMultiWidget);
    }
    ((QVBoxLayout *)(this->layout()))->addStretch();

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    this->layout()->addWidget(buttonBox);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexDialog::LAUVelmexDialog(LAUVelmexWidget *widget, QDialog *parent) : QDialog(parent), velmexWidget(widget), velmexMultiWidget(nullptr)
{
    this->setLayout(new QVBoxLayout());
#ifdef Q_OS_WIN
    this->layout()->setContentsMargins(6, 6, 6, 6);
#else
    this->layout()->setContentsMargins(6, 6, 6, 6);
#endif
    this->setWindowTitle(QString("Velmex Rail Controller"));

    if (velmexWidget){
        this->layout()->addWidget(velmexWidget);
    } else {
        velmexWidget = new LAUVelmexWidget(0, nullptr, parent);
        this->layout()->addWidget(velmexWidget);
    }
    ((QVBoxLayout *)(this->layout()))->addStretch();

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    this->layout()->addWidget(buttonBox);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUMultiVelmexWidget::LAUMultiVelmexWidget(int dims, QWidget *parent) : QWidget(parent), scanUserPathFlag(false), employSerpentineRasterFlag(false), controllerThread(nullptr), controller(nullptr), progressDialog(nullptr)
{
    this->setLayout(new QVBoxLayout());
    this->layout()->setContentsMargins(0, 0, 0, 0);
    this->setWindowTitle(QString("Velmex Rail Controller"));

    if (dims < 0){
        bool ok = false;
        QSettings settings;
        dims = settings.value(QString("LAUMultiVelmexWidget::dimensions"), 1).toInt();
        dims = QInputDialog::getInt(nullptr, QString("Velmex Axis"), QString("Select number of axes."), dims, 1, 6, 1, &ok);
        if (ok){
            settings.setValue(QString("LAUMultiVelmexWidget::dimensions"), dims);
        } else {
            dims = -1;
        }
    }

    QList<int> channels;
    for (int n = 0; n < dims; n++) {
        channels << n;
    }

    controller = new LAUVelmexController(channels);
    if (controller){
        // SINCE WE CREATED THE CONTROLLER, WE SHOULD HANDLE ANY ERROR MESSAGES
        connect(this, SIGNAL(emitSetVelocity(int,int)), controller, SLOT(onSetVelocity(int,int)));
        connect(controller, SIGNAL(emitError(QString)), this, SLOT(onError(QString)), Qt::QueuedConnection);

        controllerExistsFlag = true;
        for (int n = 0; n < dims; n++) {
            LAUVelmexWidget *widget = new LAUVelmexWidget(n, controller);
            widget->enableScanButton(dims == 1);   // 1 axis: in-group scan; >1 axis: use the 2-D scan below
            widget->enableAutoCentering(false);
            widget->setMinimumWidth(600);
            this->layout()->addWidget(widget);

            // CONNECT THE SLIDER POSITION SIGNAL TO THIS WIDGET
            connect(widget, SIGNAL(emitSliderPosition(float)), this, SLOT(onUpdateSliderPosition(float)), Qt::QueuedConnection);

            widgets.prepend(widget);
        }

        controllerThread = new QThread();
        connect(controllerThread, SIGNAL(started()), controller, SLOT(onStart()));
        connect(controllerThread, SIGNAL(finished()), controller, SLOT(onFinish()));
        connect(controllerThread, SIGNAL(finished()), controller, SLOT(deleteLater()));
        connect(controller, SIGNAL(destroyed()), controllerThread, SLOT(deleteLater()));
        connect(controllerThread, SIGNAL(destroyed()), this, SLOT(onControllerDeleted()));
        controller->moveToThread(controllerThread);
        controllerThread->start();
    }

    // The bottom (2-D) scan button exists only when there is more than one axis;
    // a single-axis window scans from its own group-box Scan button instead.
    if (dims > 1) {
        QPushButton *button = new QPushButton(QString("Scan"));
        button->setToolTip(QString("Run a 2-D raster scan across all axes."));
        connect(button, SIGNAL(clicked()), this, SLOT(onScanButton_clicked()));
        this->layout()->addWidget(button);
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUMultiVelmexWidget::LAUMultiVelmexWidget(QList<int> channels, QWidget *parent) : QWidget(parent), scanUserPathFlag(false), employSerpentineRasterFlag(false), controller(nullptr), progressDialog(nullptr)
{
    this->setLayout(new QVBoxLayout());
#ifdef Q_OS_WIN
    this->layout()->setContentsMargins(6, 6, 6, 6);
#else
    this->layout()->setContentsMargins(6, 6, 6, 6);
#endif
    this->setWindowTitle(QString("Velmex Rail Controller"));

    controller = new LAUVelmexController(channels);
    if (controller){
        // SINCE WE CREATED THE CONTROLLER, WE SHOULD HANDLE ANY ERROR MESSAGES
        connect(this, SIGNAL(emitSetVelocity(int,int)), controller, SLOT(onSetVelocity(int,int)));
        connect(controller, SIGNAL(emitError(QString)), this, SLOT(onError(QString)), Qt::QueuedConnection);

        controllerExistsFlag = true;
        for (int n = 0; n < channels.count(); n++) {
            LAUVelmexWidget *widget = new LAUVelmexWidget(channels.at(n), controller);
            widget->enableScanButton(channels.count() == 1);   // 1 axis: in-group scan; >1 axis: use the 2-D scan below
            widget->enableAutoCentering(false);
            widget->setMinimumWidth(600);
            this->layout()->addWidget(widget);

            // CONNECT THE SLIDER POSITION SIGNAL TO THIS WIDGET
            connect(widget, SIGNAL(emitSliderPosition(float)), this, SLOT(onUpdateSliderPosition(float)), Qt::QueuedConnection);

            widgets.prepend(widget);
        }

        controllerThread = new QThread();
        connect(controllerThread, SIGNAL(started()), controller, SLOT(onStart()));
        connect(controllerThread, SIGNAL(finished()), controller, SLOT(onFinish()));
        connect(controllerThread, SIGNAL(finished()), controller, SLOT(deleteLater()));
        connect(controller, SIGNAL(destroyed()), controllerThread, SLOT(deleteLater()));
        connect(controllerThread, SIGNAL(destroyed()), this, SLOT(onControllerDeleted()));
        controller->moveToThread(controllerThread);
        controllerThread->start();
    }
    ((QVBoxLayout *)(this->layout()))->addStretch();

    // The bottom (2-D) scan button exists only when there is more than one axis;
    // a single-axis window scans from its own group-box Scan button instead.
    if (channels.count() > 1) {
        QPushButton *button = new QPushButton(QString("Scan"));
        button->setToolTip(QString("Run a 2-D raster scan across all axes."));
        connect(button, SIGNAL(clicked()), this, SLOT(onScanButton_clicked()));
        this->layout()->addWidget(button);
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUMultiVelmexWidget::~LAUMultiVelmexWidget()
{
    if (controllerThread) {
        controllerThread->quit();
    } else if (controller) {
        controller->deleteLater();
    }

    // WAIT UNTIL WE KNOW CONTROLLER AND CONTROLLER THREAD HAS BEEN DELETED BEFORE MOVING ON
    while (controllerExistsFlag){
        qApp->processEvents();
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
int LAUMultiVelmexWidget::getDimensions()
{
    // SEE HOW MANY AXES THE USER WANTS BEFORE WE CREATE A WIDGET
    bool ok = false;
    QSettings settings;
    int dims = settings.value(QString("LAUMultiVelmexWidget::dimensions"), 1).toInt();
    dims = QInputDialog::getInt(nullptr, QString("Velmex Axis"), QString("Select number of axes."), dims, 1, 6, 1, &ok);
    if (ok){
        settings.setValue(QString("LAUMultiVelmexWidget::dimensions"), dims);
    } else {
        dims = -1;
    }
    return(dims);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::scanUserPath(QList<QVector4D> points)
{
    // PUT THIS WIDGET INTO SCANNING USER PATH MODE
    for (int n = 0; n < points.count(); n++){
        QVector4D point = points.at(n);
        for (int c = 0; c < widgets.count(); c++){
            // SET THE VELOCITY
            emit emitSetVelocity(widgets.at(c)->dim(), widgets.at(c)->scanSpeed());

            // SET THE DEFAULT 4D VECTOR COORDINATE
            if (widgets.at(c)->dim() == 0){
                point.setX((float)qMin(widgets.at(c)->maximum(), qMax(widgets.at(c)->minimum(), (double)point.x())));
            } else if (widgets.at(c)->dim() == 1){
                point.setY((float)qMin(widgets.at(c)->maximum(), qMax(widgets.at(c)->minimum(), (double)point.y())));
            } else if (widgets.at(c)->dim() == 2){
                point.setZ((float)qMin(widgets.at(c)->maximum(), qMax(widgets.at(c)->minimum(), (double)point.z())));
            } else if (widgets.at(c)->dim() == 3){
                point.setW((float)qMin(widgets.at(c)->maximum(), qMax(widgets.at(c)->minimum(), (double)point.w())));
            }
        }
        scanUserPathPoints << point;
    }

    if (scanUserPathPoints.isEmpty() == false){
        scanUserPathFlag = true;
        disableLimitSliders(scanUserPathFlag);
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::enableLimitSliders(bool state)
{
    for (int n = 0; n < widgets.count(); n++) {
        widgets.at(n)->enableLimitSliders(state);
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::initiateScanning()
{
    // DISNABLE THIS WIDGET
    this->setEnabled(false);

    // SET SCANNING PARAMETERS
    scanState = LAUVelmexWidget::ScanStateIterateCapture;

    // CLEAR THE SCAN STEP COORDINATES
    scannerPosition.clear();
    numberOfScansSteps.clear();
    currentScanStep.clear();

    // CLEAR THE LIST OF ENABLED WIDGET INDICES
    enabledWidgetsList.clear();

    // CONNECT ALL VALID WIDGETS
    int lastEnabledWidget = -1;
    int enabledWidgetCount = -1;
    for (int n = 0; n < widgets.count(); n++) {
        LAUVelmexWidget *widget = widgets.at(n);

        // INITIALIZE THE SCANNER POSITION VECTOR IN CASE A WIDGET IS DISABLED
        if (widget->dim() == 0){
            LAUVelmexWidget::scannerPosition.setX((float)widget->position());
        } else if (widget->dim() == 1){
            LAUVelmexWidget::scannerPosition.setY((float)widget->position());
        } else if (widget->dim() == 2){
            LAUVelmexWidget::scannerPosition.setZ((float)widget->position());
        } else if (widget->dim() == 3){
            LAUVelmexWidget::scannerPosition.setW((float)widget->position());
        }

        // IF THE WIDGET IS ENABLED, PREPARE IT FOR SCANNING
        if (widget->isEnabled()){
            widget->enableLeftToRightRaster(true);
            enabledWidgetsList << widget->dim();
            lastEnabledWidget = n;
            enabledWidgetCount++;

            if (enabledWidgetCount == 0){
                connect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerA(float,int,int)), Qt::QueuedConnection);
                connect(this, SIGNAL(emitTriggerScannerA(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)), Qt::QueuedConnection);
            } else if (enabledWidgetCount == 1){
                connect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerB(float,int,int)), Qt::QueuedConnection);
                connect(this, SIGNAL(emitTriggerScannerB(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)), Qt::QueuedConnection);
            } else if (enabledWidgetCount == 2){
                connect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerC(float,int,int)), Qt::QueuedConnection);
                connect(this, SIGNAL(emitTriggerScannerC(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)), Qt::QueuedConnection);
            } else if (enabledWidgetCount == 3){
                connect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerD(float,int,int)), Qt::QueuedConnection);
                connect(this, SIGNAL(emitTriggerScannerD(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)), Qt::QueuedConnection);
            } else if (enabledWidgetCount == 4){
                connect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerE(float,int,int)), Qt::QueuedConnection);
                connect(this, SIGNAL(emitTriggerScannerE(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)), Qt::QueuedConnection);
            } else if (enabledWidgetCount == 5){
                connect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerF(float,int,int)), Qt::QueuedConnection);
                connect(this, SIGNAL(emitTriggerScannerF(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)), Qt::QueuedConnection);
            }

            // KEEP A LIST OF SCANNING PARAMETERS FOR THE ENABLED WIDGETS
            currentScanStep << -1;
            scannerPosition << 0.0f;
            numberOfScansSteps << widgets.at(n)->scanSteps();
        }
    }

    // TELL THE MOST SIGNIFICANT ENABLED AXES TO INITIATE A SCAN
    widgets.at(lastEnabledWidget)->initiateScan();
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::completeScanning(bool centerRailsFlag)
{
    // USER HAS CANCELLED SCANNING SO PASS THE SIGNAL DOWN TO ALL WIDGETS
    scanState = LAUVelmexWidget::ScanStateNoState;

    // DISCONNECT ALL VALID WIDGETS
    int enabledWidgetCount = -1;
    for (int n = 0; n < widgets.count(); n++) {
        LAUVelmexWidget *widget = widgets.at(n);
        if (widget->isEnabled()){
            widget->enableLeftToRightRaster(true);
            enabledWidgetCount++;
            if (enabledWidgetCount == 0){
                disconnect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerA(float,int,int)));
                disconnect(this, SIGNAL(emitTriggerScannerA(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)));
            } else if (enabledWidgetCount == 1){
                disconnect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerB(float,int,int)));
                disconnect(this, SIGNAL(emitTriggerScannerB(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)));
            } else if (enabledWidgetCount == 2){
                disconnect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerC(float,int,int)));
                disconnect(this, SIGNAL(emitTriggerScannerC(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)));
            } else if (enabledWidgetCount == 3){
                disconnect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerD(float,int,int)));
                disconnect(this, SIGNAL(emitTriggerScannerD(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)));
            } else if (enabledWidgetCount == 4){
                disconnect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerE(float,int,int)));
                disconnect(this, SIGNAL(emitTriggerScannerE(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)));
            } else if (enabledWidgetCount == 5){
                disconnect(widget, SIGNAL(emitTriggerScanner(float,int,int)), this, SLOT(onTriggerScannerF(float,int,int)));
                disconnect(this, SIGNAL(emitTriggerScannerF(float,int,int)), widget, SLOT(onTriggerScanner(float,int,int)));
            }
        }
    }

    // CENTER THE RAILS
    if (centerRailsFlag){
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                widgets.at(k)->onCenterButton_clicked();
            }
        }
    }

    // RE-ENABLE THIS WIDGET
    this->setEnabled(true);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onScanButton_clicked()
{
    // MAKE SURE WE HAVE A CONNECTED CONTROLLER BEFORE INITIATING A SCAN
    if (controller->isConnected()){
        // CHECK ALL THE WIDGETS TO SEE IF THERE IS A SINGLE WIDGET THAT IS ENABLED
        bool anyWidgetsEnabledFlag = false;
        for (int n = 0; n < widgets.count(); n++) {
            if (widgets.at(n)->isEnabled()){
                anyWidgetsEnabledFlag = true;
                break;
            }
        }

        // MAKE SURE WE FOUND AT LEAST ONE ENABLED WIDGET OTHERWISE IGNORE THIS BUTTON CLICK
        if (anyWidgetsEnabledFlag == false){
            QMessageBox::warning(this, QString("Calibration Widget"), QString("No axes are enabled for scanning."), QMessageBox::Ok);
            return;
        }

        // MAKE SURE THE USER TURNED OFF THE SCREEN SAVER SO THAT WE CAN CONTROL
        // THE BACKLIGHT DISPLAY OTHERWISE CANCEL THE SCAN
        int ret = QMessageBox::warning(this, QString("Calibration Widget"), QString("Please disable the screen saver before continuing."), QMessageBox::Ok | QMessageBox::Cancel);
        if (ret == QMessageBox::Ok) {
            initiateScanning();
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onUpdateSliderPosition(float val)
{
    // MAKE SURE RAILS HAVE COME TO A COMPLETE STOP
    if (isMoving() == false){
        if (scanUserPathFlag){
            // SEE IF WE HAVE ANY POINTS TO PROCESS
            if (scanUserPathPoints.isEmpty()){
                // DELETE THE PROGRESS DIALOG
                if (progressDialog){
                    progressDialog->setValue(2 * progressDialog->maximum());
                    progressDialog->deleteLater();
                    progressDialog = nullptr;
                }

                // TAKE THIS WIDGET OUT OF SCANNING USER PATH MODE
                scanUserPathFlag = false;

                // RE-ENABLE LIMIT SLIDERS
                disableLimitSliders(scanUserPathFlag);

                // RESET THE SCAN VELOCITY
                for (int c = 0; c < widgets.count(); c++){
                    emit emitSetVelocity(c);
                }

                // DISABLE THIS WIDGET SO THE USER CAN'T MESS UP THE SLIDERS DURING SCANNING
                this->setEnabled(true);
            } else {
                // SEE IF WE NEED TO CREATE A PROGRESS DIALOG SO USER CAN CANCEL SCANNING
                if (progressDialog == nullptr){
                    progressDialog = new QProgressDialog(QString("Scanning..."), QString("Abort"), 0, 2 * scanUserPathPoints.count(), this, Qt::Sheet);
                    progressDialog->setModal(Qt::WindowModal);
                    progressDialog->show();
                }

                if (progressDialog->wasCanceled()){
                    // SINCE USER CANCELLED SCANNING, LET'S CLEAR THE SCAN PATH AND THEN RECURIVELY CALL THIS FUNCTION TO DELETE THE PROGRESS BAR
                    scanUserPathPoints.clear();
                    onUpdateSliderPosition(val);
                } else {
                    // UPDATE THE PROGRESS DIALOG BASED ON THE NUMBER OF POINTS IN OUR SCAN PATH
                    progressDialog->setValue(2 * progressDialog->maximum() - 2 * scanUserPathPoints.count() + 1);

                    // GRAB THE FIRST POINT IN THE LIST
                    if (scanUserPathPoints.isEmpty()){
                        onUpdateSliderPosition(val);
                        return;
                    }
                    QVector4D point = scanUserPathPoints.first();

                    // SEE IF FIRST AXIS IS AT FIRST COORDINATE OF FIRST POSITION IN LIST OTHERWISE MOVE TO SECOND COORDINATE AND SO ON
                    double deltaX = qAbs(position(0) - point.x());
                    if (deltaX > 0.010){
                        onGoToPosition(point.x(), 0);
                    } else {
                        if (widgets.count() < 2){
                            scanUserPathPoints.takeFirst();
                            if (scanUserPathPoints.isEmpty() == false){
                                onUpdateSliderPosition(val);
                            }
                        } else {
                            double deltaY = qAbs(position(1) - point.y());
                            if (deltaY > 0.010){
                                onGoToPosition(point.y(), 1);
                            } else {
                                if (widgets.count() < 3){
                                    scanUserPathPoints.takeFirst();
                                    if (scanUserPathPoints.isEmpty() == false){
                                        onUpdateSliderPosition(val);
                                    }
                                } else {
                                    double deltaZ = qAbs(position(2) - point.z());
                                    if (deltaZ > 0.010){
                                        onGoToPosition(point.z(), 2);
                                    } else {
                                        if (widgets.count() < 4){
                                            scanUserPathPoints.takeFirst();
                                            if (scanUserPathPoints.isEmpty() == false){
                                                onUpdateSliderPosition(val);
                                            }
                                        } else {
                                            double deltaW = qAbs(position(3) - point.w());
                                            if (deltaW > 0.010){
                                                onGoToPosition(point.w(), 3);
                                            } else {
                                                scanUserPathPoints.takeFirst();
                                                if (scanUserPathPoints.isEmpty() == false){
                                                    onUpdateSliderPosition(val);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScanner(float pos, int n, int N)
{
    // THIS SLOT IS CALLED BY THE USER AFTER THEY ARE DONE SCANNING
    // CHECK FOR ABORT CODE
    if (scanState == LAUVelmexWidget::ScanStateWaitingOnScanner) {
        if (n < 0) {
            // USER HAS CANCELLED SCANNING SO PASS THE SIGNAL DOWN TO ALL WIDGETS
            scanState = LAUVelmexWidget::ScanStateNoState;

            int enabledWidgetCount = -1;
            for (int k = 0; k < widgets.count(); k++){
                if (widgets.at(k)->isEnabled()){
                    enabledWidgetCount++;
                    currentScanStep.replace(enabledWidgetCount,-1);
                    if (enabledWidgetCount == 0){
                        emit emitTriggerScannerA(pos, -1, numberOfScansSteps.at(enabledWidgetCount));
                    } else if (enabledWidgetCount == 1){
                        emit emitTriggerScannerB(pos, -1, numberOfScansSteps.at(enabledWidgetCount));
                    } else if (enabledWidgetCount == 2){
                        emit emitTriggerScannerC(pos, -1, numberOfScansSteps.at(enabledWidgetCount));
                    } else if (enabledWidgetCount == 3){
                        emit emitTriggerScannerD(pos, -1, numberOfScansSteps.at(enabledWidgetCount));
                    } else if (enabledWidgetCount == 4){
                        emit emitTriggerScannerE(pos, -1, numberOfScansSteps.at(enabledWidgetCount));
                    } else if (enabledWidgetCount == 5){
                        emit emitTriggerScannerF(pos, -1, numberOfScansSteps.at(enabledWidgetCount));
                    }
                }
            }

            // DISCONNECT ALL THE WIDGETS AND CENTER THE AXES
            completeScanning(false);
        } else {
            // LET'S CALCULATE OUR SCANNER COORDINATES BASED ON USER COORDINATES
            //int enabledWidgetCount = -1;
            //for (int k = 0; k < widgets.count(); k++){
            //    if (widgets.at(k)->isEnabled()){
            //        enabledWidgetCount++;
            //        currentScanStep.replace(enabledWidgetCount, n % numberOfScansSteps.at(enabledWidgetCount));
            //        n = n / numberOfScansSteps.at(enabledWidgetCount);
            //    }
            //}

            // LET'S UPDATE THE SCAN STATE
            scanState = LAUVelmexWidget::ScanStateIterateCapture;

            // MOVE THE LOWEST RAIL TO THE NEXT POSITION
            emit emitTriggerScannerA(scannerPosition.at(0), currentScanStep.at(0), numberOfScansSteps.at(0));
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScannerA(float pos, int n, int N)
{
    // UPDATE SCANNER POSITION STATIC VECTOR
    if (enabledWidgetsList.at(0) == 0){
        LAUVelmexWidget::scannerPosition.setX(pos);
    } else if (enabledWidgetsList.at(0) == 1){
        LAUVelmexWidget::scannerPosition.setY(pos);
    } else if (enabledWidgetsList.at(0) == 2){
        LAUVelmexWidget::scannerPosition.setZ(pos);
    } else if (enabledWidgetsList.at(0) == 3){
        LAUVelmexWidget::scannerPosition.setW(pos);
    }

    // UPDATE CORRESPONDING COORDINATES THAT WE RECEIVE FROM THE RAIL WIDGET
    scannerPosition.replace(0, pos);
    currentScanStep.replace(0, n);
    numberOfScansSteps.replace(0, N);

    // SEE IF THIS IS THE MINUS ON THE LEAST RAIL AND ALL OTHER RAILS AT ZERO
    bool flag = (n < 0);
    for (int k = 1; k < currentScanStep.count(); k++){
        if (currentScanStep.at(k) > 0){
            flag = false;
        }
    }

    if (flag){
        // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
        int N = 1;
        for (int k = 0; k < currentScanStep.count(); k++){
            N *= numberOfScansSteps.at(k);
        }

        // WE ARE INITIATING A SCAN SO AS THE LEAST SIGNIFICANT AXES WE SHOULD TELL THE USER
        emit emitTriggerScanner(pos, -1, N);
    } else if (n == N){
        // LET'S TOGGLE THIS RAIL TO SPEED UP SCANS
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 0){
                    widgets.at(k)->toggleLeftToRightRaster();
                    break;
                }
            }
        }

        // IF CURRENT STEP IS EQUAL TO TOTAL NUMBER OF STEPS, THEN THIS AXES
        // IS COMPLETE SO CHECK TO SEE IF THIS IS THE MOST SIGNIFICANT RAIL
        if (currentScanStep.count() == 1){
            // SET SCAN STATE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateComplete;

            // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
            int n = 0;
            int N = 1;
            for (int k = 0; k < currentScanStep.count(); k++){
                n += (currentScanStep.at(k) * N);
                N *= numberOfScansSteps.at(k);
            }

            // TELL THE USER THAT WE ARE DONE
            emit emitTriggerScanner(pos, n, N);

            // COMPLETE SCANNING SINCE THE USER WON'T TELL US TO
            completeScanning(true);
        } else {
            // SINCE WE ARE NOT THE ONLY RAIL WE MUST TELL THE NEXT RAIL TO MOVE
            emit emitTriggerScannerB(scannerPosition.at(1), currentScanStep.at(1), numberOfScansSteps.at(1));
        }
    } else if (n > -1){
        // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
        n = 0;
        N = 1;
        for (int k = 0; k < currentScanStep.count(); k++){
            n += (currentScanStep.at(k) * N);
            N *= numberOfScansSteps.at(k);
        }

        qDebug() << currentScanStep;
        if (scanState == LAUVelmexWidget::ScanStateNoState) {
            setDisabled(false);
        } else if (scanState == LAUVelmexWidget::ScanStateInitiate) {
            ;
        } else if (scanState == LAUVelmexWidget::ScanStateIterateCapture) {
            // CHANGE THE STATE TO WAIT FOR THE INCOMING CAPTURE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateWaitingOnScanner;

            // TRIGGER ACTION LIKE TRIGGER SCANNER WHEN YOU REACH SCAN POSITION
            emit emitTriggerScanner(pos, n, N);
        } else if (scanState == LAUVelmexWidget::ScanStateIterateMove) {
            // CHANGE THE STATE TO WAIT FOR THE INCOMING CAPTURE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateNoState;

            // IF WE ARE NOT AT THE END OF A RAIL THEN TELL THE USER TO TRIGGER THEIR SCAN
            emit emitTriggerScanner(pos, n, N);
        } else if (scanState == LAUVelmexWidget::ScanStateCalibrate) {
            ;
        } else {
            ;
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScannerB(float pos, int n, int N)
{
    // UPDATE SCANNER POSITION STATIC VECTOR
    if (enabledWidgetsList.at(1) == 0){
        LAUVelmexWidget::scannerPosition.setX(pos);
    } else if (enabledWidgetsList.at(1) == 1){
        LAUVelmexWidget::scannerPosition.setY(pos);
    } else if (enabledWidgetsList.at(1) == 2){
        LAUVelmexWidget::scannerPosition.setZ(pos);
    } else if (enabledWidgetsList.at(1) == 3){
        LAUVelmexWidget::scannerPosition.setW(pos);
    }

    // UPDATE CORRESPONDING COORDINATES THAT WE RECEIVE FROM THE RAIL WIDGET
    scannerPosition.replace(1, pos);
    currentScanStep.replace(1, n);
    numberOfScansSteps.replace(1, N);

    // IF CURRENT STEP IS EQUAL TO TOTAL NUMBER OF STEPS, THEN THIS AXES IS COMPLETE
    if (n == N){
        // LET'S TOGGLE THIS RAIL TO SPEED UP SCANS
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 1){
                    widgets.at(k)->toggleLeftToRightRaster();
                    break;
                }
            }
        }

        // CHECK TO SEE IF THIS IS THE MOST SIGNIFICANT RAIL
        if (currentScanStep.count() == 2){
            // SET SCAN STATE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateComplete;

            // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
            int n = 0;
            int N = 1;
            for (int k = 0; k < currentScanStep.count(); k++){
                n += (currentScanStep.at(k) * N);
                N *= numberOfScansSteps.at(k);
            }

            // TELL THE USER THAT WE ARE DONE
            emit emitTriggerScanner(pos, n, N);

            // COMPLETE SCANNING SINCE THE USER WON'T TELL US TO
            completeScanning(true);
        } else {
            // SINCE WE ARE NOT THE ONLY RAIL WE MUST TELL THE NEXT RAIL TO MOVE
            emit emitTriggerScannerC(scannerPosition.at(2), currentScanStep.at(2), numberOfScansSteps.at(2));
        }
    } else if (n > -1){
        // AS THE SECOND SCANNER, WE SHOULD START A NEW SCAN ON THE FIRST
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 0){
                    widgets.at(k)->initiateScan();
                }
            }
        }
    } else {
        ;
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScannerC(float pos, int n, int N)
{
    // UPDATE SCANNER POSITION STATIC VECTOR
    if (enabledWidgetsList.at(2) == 0){
        LAUVelmexWidget::scannerPosition.setX(pos);
    } else if (enabledWidgetsList.at(2) == 1){
        LAUVelmexWidget::scannerPosition.setY(pos);
    } else if (enabledWidgetsList.at(2) == 2){
        LAUVelmexWidget::scannerPosition.setZ(pos);
    } else if (enabledWidgetsList.at(2) == 3){
        LAUVelmexWidget::scannerPosition.setW(pos);
    }

    // UPDATE CORRESPONDING COORDINATES THAT WE RECEIVE FROM THE RAIL WIDGET
    scannerPosition.replace(2, pos);
    currentScanStep.replace(2, n);
    numberOfScansSteps.replace(2, N);

    // IF CURRENT STEP IS EQUAL TO TOTAL NUMBER OF STEPS, THEN THIS AXES IS COMPLETE
    if (n == N){
        // LET'S TOGGLE THIS RAIL TO SPEED UP SCANS
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 2){
                    widgets.at(k)->toggleLeftToRightRaster();
                    break;
                }
            }
        }

        // CHECK TO SEE IF THIS IS THE MOST SIGNIFICANT RAIL
        if (currentScanStep.count() == 3){
            // SET SCAN STATE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateComplete;

            // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
            int n = 0;
            int N = 1;
            for (int k = 0; k < currentScanStep.count(); k++){
                n += (currentScanStep.at(k) * N);
                N *= numberOfScansSteps.at(k);
            }

            // TELL THE USER THAT WE ARE DONE
            emit emitTriggerScanner(pos, n, N);

            // COMPLETE SCANNING SINCE THE USER WON'T TELL US TO
            completeScanning(true);
        } else {
            // SINCE WE ARE NOT THE ONLY RAIL WE MUST TELL THE NEXT RAIL TO MOVE
            emit emitTriggerScannerC(scannerPosition.at(3), currentScanStep.at(3), numberOfScansSteps.at(3));
        }
    } else if (n > -1){
        // AS THE THIRD SCANNER, WE SHOULD START A NEW SCAN ON THE SECOND
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 1){
                    widgets.at(k)->initiateScan();
                }
            }
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScannerD(float pos, int n, int N)
{
    // UPDATE SCANNER POSITION STATIC VECTOR
    if (enabledWidgetsList.at(3) == 0){
        LAUVelmexWidget::scannerPosition.setX(pos);
    } else if (enabledWidgetsList.at(3) == 1){
        LAUVelmexWidget::scannerPosition.setY(pos);
    } else if (enabledWidgetsList.at(3) == 2){
        LAUVelmexWidget::scannerPosition.setZ(pos);
    } else if (enabledWidgetsList.at(3) == 3){
        LAUVelmexWidget::scannerPosition.setW(pos);
    }


    // UPDATE CORRESPONDING COORDINATES THAT WE RECEIVE FROM THE RAIL WIDGET
    scannerPosition.replace(3, pos);
    currentScanStep.replace(3, n);
    numberOfScansSteps.replace(3, N);

    // IF CURRENT STEP IS EQUAL TO TOTAL NUMBER OF STEPS, THEN THIS AXES IS COMPLETE
    if (n == N){
        // LET'S TOGGLE THIS RAIL TO SPEED UP SCANS
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 3){
                    widgets.at(k)->toggleLeftToRightRaster();
                    break;
                }
            }
        }

        // CHECK TO SEE IF THIS IS THE MOST SIGNIFICANT RAIL
        if (currentScanStep.count() == 4){
            // SET SCAN STATE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateComplete;

            // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
            int n = 0;
            int N = 1;
            for (int k = 0; k < currentScanStep.count(); k++){
                n += (currentScanStep.at(k) * N);
                N *= numberOfScansSteps.at(k);
            }

            // TELL THE USER THAT WE ARE DONE
            emit emitTriggerScanner(pos, n, N);

            // COMPLETE SCANNING SINCE THE USER WON'T TELL US TO
            completeScanning(true);
        } else {
            // SINCE WE ARE NOT THE ONLY RAIL WE MUST TELL THE NEXT RAIL TO MOVE
            emit emitTriggerScannerC(scannerPosition.at(4), currentScanStep.at(4), numberOfScansSteps.at(4));
        }
    } else if (n > -1){
        // AS THE FOURTH SCANNER, WE SHOULD START A NEW SCAN ON THE THIRD
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 2){
                    widgets.at(k)->initiateScan();
                }
            }
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScannerE(float pos, int n, int N)
{
    // UPDATE CORRESPONDING COORDINATES THAT WE RECEIVE FROM THE RAIL WIDGET
    scannerPosition.replace(4, pos);
    currentScanStep.replace(4, n);
    numberOfScansSteps.replace(4, N);

    // IF CURRENT STEP IS EQUAL TO TOTAL NUMBER OF STEPS, THEN THIS AXES IS COMPLETE
    if (n == N){
        // LET'S TOGGLE THIS RAIL TO SPEED UP SCANS
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 4){
                    widgets.at(k)->toggleLeftToRightRaster();
                    break;
                }
            }
        }

        // CHECK TO SEE IF THIS IS THE MOST SIGNIFICANT RAIL
        if (currentScanStep.count() == 5){
            // SET SCAN STATE TO COMPLETE
            scanState = LAUVelmexWidget::ScanStateComplete;

            // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
            int n = 0;
            int N = 1;
            for (int k = 0; k < currentScanStep.count(); k++){
                n += (currentScanStep.at(k) * N);
                N *= numberOfScansSteps.at(k);
            }

            // TELL THE USER THAT WE ARE DONE
            emit emitTriggerScanner(pos, n, N);

            // COMPLETE SCANNING SINCE THE USER WON'T TELL US TO
            completeScanning(true);
        } else {
            // SINCE WE ARE NOT THE ONLY RAIL WE MUST TELL THE NEXT RAIL TO MOVE
            emit emitTriggerScannerC(scannerPosition.at(5), currentScanStep.at(5), numberOfScansSteps.at(5));
        }
    } else if (n > -1){
        // AS THE FIFTH SCANNER, WE SHOULD START A NEW SCAN ON THE FOURTH
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 3){
                    widgets.at(k)->initiateScan();
                }
            }
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUMultiVelmexWidget::onTriggerScannerF(float pos, int n, int N)
{
    // UPDATE CORRESPONDING COORDINATES THAT WE RECEIVE FROM THE RAIL WIDGET
    scannerPosition.replace(5, pos);
    currentScanStep.replace(5, n);
    numberOfScansSteps.replace(5, N);

    // IF CURRENT STEP IS EQUAL TO TOTAL NUMBER OF STEPS, THEN THIS AXES IS COMPLETE
    if (n == N){
        // LET'S TOGGLE THIS RAIL TO SPEED UP SCANS
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 5){
                    widgets.at(k)->toggleLeftToRightRaster();
                    break;
                }
            }
        }

        // SET SCAN STATE TO COMPLETE
        scanState = LAUVelmexWidget::ScanStateComplete;

        // DERIVE CUMULATIVE SCANNER COORDINATES THAT WE CAN PASS TO THE USER
        int n = 0;
        int N = 1;
        for (int k = 0; k < currentScanStep.count(); k++){
            n += (currentScanStep.at(k) * N);
            N *= numberOfScansSteps.at(k);
        }

        // TELL THE USER THAT WE ARE DONE
        emit emitTriggerScanner(pos, n, N);

        // COMPLETE SCANNING SINCE THE USER WON'T TELL US TO
        completeScanning(true);
    } else if (n > -1){
        // AS THE SIXTH SCANNER, WE SHOULD START A NEW SCAN ON THE FIFTH
        int enabledWidgetCount = -1;
        for (int k = 0; k < widgets.count(); k++){
            if (widgets.at(k)->isEnabled()){
                enabledWidgetCount++;
                if (enabledWidgetCount == 4){
                    widgets.at(k)->initiateScan();
                }
            }
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexWidget::LAUVelmexWidget(int dim, LAUVelmexController *obj, QWidget *parent) : QWidget(parent), dimension(dim), scanState(ScanStateNoState), leftToRightRasterFlag(true), autoCenteringFlag(true), isConnectedFlag(false), isRotaryType(false), movingScanModeFlag(false), isMovingCounter(0), velmexScannerSpeed(1000), velmexScannerNumberOfScanSteps(32), motorClicksPerRotation(400), railPitch(0.1), controllerFlag(false), controllerThread(nullptr), controller(obj)
{
    QSettings settings;

    // CHECK FOR INVALID AXIS INDEX AND IF SO ASK USER TO PICK AN AXIS
    if (dimension < 0){
        bool ok = false;
        QSettings settings;
        dimension = settings.value(QString("LAUVelmexWidget::velmexAxis"), 1).toInt() + 1;
        dimension = QInputDialog::getInt(nullptr, QString("Velmex Axis"), QString("Select Velmex axis."), dimension, 1, 3, 1, &ok) - 1;
        if (ok){
            settings.setValue(QString("LAUVelmexWidget::velmexAxis"), dimension);
        } else {
            dimension = -1;
        }
    }

    this->setLayout(new QHBoxLayout());
    this->layout()->setContentsMargins(0, 0, 0, 0);
    this->layout()->setSpacing(6);

    groupBox = new QGroupBox(QString("Slide Controls"));
    groupBox->setLayout(new QHBoxLayout());
    groupBox->layout()->setContentsMargins(6, 6, 6, 6);

    if (dimension == 0) {
        railString = QString("LAUVelmexWidget::X");
        groupBox->setTitle("X Controls:");
    } else if (dimension == 1) {
        railString = QString("LAUVelmexWidget::Y");
        groupBox->setTitle("Y Controls:");
    } else if (dimension == 2) {
        // On this machine SDK axis 2 (PU3/DR3) drives the Y cross-beam, so label it "Y".
        railString = QString("LAUVelmexWidget::Z");   // settings key kept as Z for continuity
        groupBox->setTitle("Y Controls:");
    } else if (dimension == 3) {
        railString = QString("LAUVelmexWidget::A");
        groupBox->setTitle("A Controls:");
    } else if (dimension == 4) {
        railString = QString("LAUVelmexWidget::B");
        groupBox->setTitle("B Controls:");
    } else if (dimension == 5) {
        railString = QString("LAUVelmexWidget::C");
        groupBox->setTitle("C Controls:");
    }
    groupBox->setCheckable(true);
    groupBox->setChecked(settings.value(QString("%1::groupBox::Checked").arg(railString), true).toBool());
    this->layout()->addWidget(groupBox);

    settingsButton = new QPushButton(QString("?"));
    settingsButton->setFixedWidth(30);
    settingsButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    settingsButton->setToolTip(QString("Axis settings: units, home direction, speed and scan steps."));
    groupBox->layout()->addWidget(settingsButton);
    connect(settingsButton, SIGNAL(clicked()), this, SLOT(onSettingsButton_clicked()));

    QWidget *widget = new QWidget();
    widget->setLayout(new QGridLayout());
    widget->layout()->setSpacing(3);
    widget->layout()->setContentsMargins(0, 0, 0, 0);
    groupBox->layout()->addWidget(widget);

    velmexScannerNumberOfScanSteps = settings.value(QString("%1::velmexScannerNumberOfScanSteps").arg(railString), velmexScannerNumberOfScanSteps).toInt();
    motorClicksPerRotation = settings.value(QString("%1::motorClicksPerRotation").arg(railString), motorClicksPerRotation).toInt();
    velmexScannerSpeed = settings.value(QString("%1::velmexScannerSpeed").arg(railString), velmexScannerSpeed).toInt();
    isRotaryType = settings.value(QString("%1::isRotaryType").arg(railString), isRotaryType).toBool();
    railPitch = settings.value(QString("%1::railPitch").arg(railString), railPitch).toDouble();
    unitIndex = settings.value(QString("LAUVelmexWidget::unitIndex"), unitIndex).toInt();

    leftLimitSlider = new QSlider(Qt::Horizontal);
    leftLimitSlider->setMinimum(settings.value(QString("%1::leftLimitSlider::Minimum").arg(railString), 0).toInt());
    leftLimitSlider->setMaximum(settings.value(QString("%1::leftLimitSlider::Maximum").arg(railString), 100000).toInt());
    leftLimitSlider->setValue(settings.value(QString("%1::leftLimitSlider::Value").arg(railString), 0).toInt());
    connect(leftLimitSlider, SIGNAL(valueChanged(int)), this, SLOT(onLeftLimitSlider_valueChanged(int)));
    qDebug() << leftLimitSlider->minimum() << leftLimitSlider->value() << leftLimitSlider->maximum();

    positionSlider = new QSlider(Qt::Horizontal);
    positionSlider->setMinimum(settings.value(QString("%1::positionSlider::Minimum").arg(railString), 0).toInt());
    positionSlider->setMaximum(settings.value(QString("%1::positionSlider::Maximum").arg(railString), 100000).toInt());
    positionSlider->setValue(settings.value(QString("%1::positionSlider::Value").arg(railString), 100000 / 2).toInt());
    connect(positionSlider, SIGNAL(valueChanged(int)), this, SLOT(onPositionSlider_valueChanged(int)));
    qDebug() << positionSlider->minimum() << positionSlider->value() << positionSlider->maximum();

    rightLimitSlider = new QSlider(Qt::Horizontal);
    rightLimitSlider->setMinimum(settings.value(QString("%1::rightLimitSlider::Minimum").arg(railString), 0).toInt());
    rightLimitSlider->setMaximum(settings.value(QString("%1::rightLimitSlider::Maximum").arg(railString), 100000).toInt());
    rightLimitSlider->setValue(settings.value(QString("%1::rightLimitSlider::Value").arg(railString), 100000).toInt());
    connect(rightLimitSlider, SIGNAL(valueChanged(int)), this, SLOT(onRightLimitSlider_valueChanged(int)));
    qDebug() << rightLimitSlider->minimum() << rightLimitSlider->value() << rightLimitSlider->maximum();

    leftLimitSpinBox = new LAUDoubleSpinBox();
    leftLimitSpinBox->setAlignment(Qt::AlignRight);
    leftLimitSpinBox->setDecimals(4);
    leftLimitSpinBox->setFixedWidth(90);
    leftLimitSpinBox->setSingleStep(0.1);
    connect(leftLimitSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onLeftLimitSpinBox_valueChanged(double)));

    positionSpinBox = new LAUDoubleSpinBox();
    positionSpinBox->setAlignment(Qt::AlignRight);
    positionSpinBox->setDecimals(4);
    positionSpinBox->setFixedWidth(90);
    positionSpinBox->setSingleStep(0.1);
    connect(positionSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onPositionSpinBox_valueChanged(double)));

    rightLimitSpinBox = new LAUDoubleSpinBox();
    rightLimitSpinBox->setAlignment(Qt::AlignRight);
    rightLimitSpinBox->setDecimals(4);
    rightLimitSpinBox->setFixedWidth(90);
    rightLimitSpinBox->setSingleStep(0.1);
    connect(rightLimitSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onRightLimitSpinBox_valueChanged(double)));

    calibrateButton = new QPushButton(QString("Calibrate"));
    calibrateButton->setToolTip(QString("Home this axis: drive to the home switch to set zero,\n"
                                        "then seek the far switch to set the travel."));
    connect(calibrateButton, SIGNAL(clicked()), this, SLOT(onCalibrateButton_clicked()));
    centerButton = new QPushButton(QString("Center"));
    centerButton->setToolTip(QString("Move this axis to the middle of its travel."));
    connect(centerButton, SIGNAL(clicked()), this, SLOT(onCenterButton_clicked()));
    scanButton = new QPushButton(QString("Scan"));
    scanButton->setToolTip(QString("Sweep this axis from the left limit to the right limit\n"
                                   "in the configured number of scan steps."));
    connect(scanButton, SIGNAL(clicked()), this, SLOT(onScanButton_clicked()));

    ((QGridLayout *)(widget->layout()))->addWidget(new QLabel(QString("Left Limit:")), 0, 0);
    ((QGridLayout *)(widget->layout()))->addWidget(new QLabel(QString("Slider Position:")), 1, 0);
    ((QGridLayout *)(widget->layout()))->addWidget(new QLabel(QString("Right Limit:")), 2, 0);

    ((QGridLayout *)(widget->layout()))->addWidget(leftLimitSlider, 0, 1);
    ((QGridLayout *)(widget->layout()))->addWidget(positionSlider, 1, 1);
    ((QGridLayout *)(widget->layout()))->addWidget(rightLimitSlider, 2, 1);

    ((QGridLayout *)(widget->layout()))->addWidget(leftLimitSpinBox, 0, 2);
    ((QGridLayout *)(widget->layout()))->addWidget(positionSpinBox, 1, 2);
    ((QGridLayout *)(widget->layout()))->addWidget(rightLimitSpinBox, 2, 2);

    ((QGridLayout *)(widget->layout()))->addWidget(calibrateButton, 0, 3);
    ((QGridLayout *)(widget->layout()))->addWidget(centerButton, 1, 3);
    ((QGridLayout *)(widget->layout()))->addWidget(scanButton, 2, 3);

    if (dimension > -1){
        if (controller == nullptr) {
            controllerFlag = true;
            controller = new LAUVelmexController(dimension);
            connect(this, SIGNAL(emitUpdateUnits()), controller, SLOT(onUpdateUnits()), Qt::QueuedConnection);
            connect(this, SIGNAL(emitSetVelocity(int)), controller, SLOT(onSetVelocity(int)), Qt::QueuedConnection);
            connect(this, SIGNAL(emitSetVelocity(int, int)), controller, SLOT(onSetVelocity(int, int)), Qt::QueuedConnection);
            connect(this, SIGNAL(emitCalibrateSlider(int)), controller, SLOT(onCalibrateSlide(int)), Qt::QueuedConnection);
            connect(this, SIGNAL(emitSetSliderPosition(int, int)), controller, SLOT(onMoveToPosition(int, int)), Qt::QueuedConnection);
            connect(controller, SIGNAL(emitError(QString)), this, SLOT(onError(QString)), Qt::QueuedConnection);
            connect(controller, SIGNAL(emitCalibrationComplete(int, int, int)), this, SLOT(onUpdateLimits(int, int, int)), Qt::QueuedConnection);
            connect(controller, SIGNAL(emitSliderPosition(int, int)), this, SLOT(onUpdatePositon(int, int)), Qt::QueuedConnection);

            controllerThread = new QThread();
            connect(controllerThread, SIGNAL(started()), controller, SLOT(onStart()));
            connect(controllerThread, SIGNAL(finished()), controller, SLOT(onFinish()));
            connect(controllerThread, SIGNAL(finished()), controller, SLOT(deleteLater()));
            connect(controller, SIGNAL(destroyed()), controllerThread, SLOT(deleteLater()));
            controller->moveToThread(controllerThread);
            controllerThread->start();
        } else {
            controllerFlag = false;
#ifndef SIMULATESLIDER
            connect(this, SIGNAL(emitUpdateUnits()), controller, SLOT(onUpdateUnits()), Qt::QueuedConnection);
            connect(this, SIGNAL(emitSetVelocity(int)), controller, SLOT(onSetVelocity(int)), Qt::QueuedConnection);
            connect(this, SIGNAL(emitSetVelocity(int, int)), controller, SLOT(onSetVelocity(int, int)), Qt::QueuedConnection);
            connect(this, SIGNAL(emitCalibrateSlider(int)), controller, SLOT(onCalibrateSlide(int)), Qt::QueuedConnection);
            connect(this, SIGNAL(emitSetSliderPosition(int, int)), controller, SLOT(onMoveToPosition(int, int)), Qt::QueuedConnection);
            //connect(controller, SIGNAL(emitError(QString)), this, SLOT(onError(QString)), Qt::QueuedConnection);
            connect(controller, SIGNAL(emitCalibrationComplete(int, int, int)), this, SLOT(onUpdateLimits(int, int, int)), Qt::QueuedConnection);
            connect(controller, SIGNAL(emitSliderPosition(int, int)), this, SLOT(onUpdatePositon(int, int)), Qt::QueuedConnection);
#endif
        }
        updateScaleFactor();

        // SEE IF WE ARE CONNECTED TO A SLIDE RAIL
        if (controller) {
            controller->addWidget(this);
#ifndef SIMULATESLIDER
            setEnabled(isConnectedFlag);
#endif
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
LAUVelmexWidget::~LAUVelmexWidget()
{
    if (controllerFlag) {
        if (controllerThread) {
            controllerThread->quit();
        }
    }

    QSettings settings;
    settings.setValue(QString("%1::groupBox::Checked").arg(railString), groupBox->isChecked());

    settings.setValue(QString("%1::leftLimitSlider::Minimum").arg(railString), leftLimitSlider->minimum());
    settings.setValue(QString("%1::leftLimitSlider::Maximum").arg(railString), leftLimitSlider->maximum());
    settings.setValue(QString("%1::leftLimitSlider::Value").arg(railString), leftLimitSlider->value());

    settings.setValue(QString("%1::positionSlider::Minimum").arg(railString), positionSlider->minimum());
    settings.setValue(QString("%1::positionSlider::Maximum").arg(railString), positionSlider->maximum());
    settings.setValue(QString("%1::positionSlider::Value").arg(railString), positionSlider->value());

    settings.setValue(QString("%1::rightLimitSlider::Minimum").arg(railString), rightLimitSlider->minimum());
    settings.setValue(QString("%1::rightLimitSlider::Maximum").arg(railString), rightLimitSlider->maximum());
    settings.setValue(QString("%1::rightLimitSlider::Value").arg(railString), rightLimitSlider->value());
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::enableLimitSliders(bool state)
{
    if (state == true){
        // MOVE SLIDERS BACK TO THEIR SAVE VALUES
        leftLimitSlider->setValue(leftSliderMemory);
        rightLimitSlider->setValue(rightSliderMemory);

        // CENTER THE RAIL IF ITS OUTSIDE THE LIMITS
        if (positionSlider->value() < leftSliderMemory){
            positionSlider->setValue(leftSliderMemory);
        } else if (positionSlider->value() > rightSliderMemory){
            positionSlider->setValue(rightSliderMemory);
        }

        // LET THE APPLICATION PROCESS ANY SIGNALS FOR THESE CHANGES
        qApp->processEvents();
    } else {
        // SAVE THE CURRENT VALUE OF THE SLIDERS SO THAT WE CAN RESTORE LATER
        leftSliderMemory = leftLimitSlider->value();
        rightSliderMemory = rightLimitSlider->value();

        // MOVE THE SLIDERS TO THEIR MOST EXTREME POSITIONS
        leftLimitSlider->setValue(leftLimitSlider->minimum());
        rightLimitSlider->setValue(rightLimitSlider->maximum());

        // LET THE APPLICATION PROCESS ANY SIGNALS FOR THESE CHANGES
        qApp->processEvents();
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onConnected(bool state)
{
    isConnectedFlag = state;
#ifndef SIMULATESLIDER
    setEnabled(isConnectedFlag);
#endif
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::updateScaleFactor()
{
    // FMC4030 is native millimetres.  One "count" = 1 micron (0.001 mm), fixed; the unit
    // selection only changes how that count is displayed (millimetres vs inches).
    if (unitIndex == 0) {            // INCHES
        railStepsToUnitsScaleFactor = 0.001 / 25.4;
    } else {                         // MILLIMETERS
        railStepsToUnitsScaleFactor = 0.001;
    }

    positionSpinBox->blockSignals(true);
    leftLimitSpinBox->blockSignals(true);
    rightLimitSpinBox->blockSignals(true);

    double a = leftLimitSlider->minimum() * railStepsToUnitsScaleFactor;
    double b = leftLimitSlider->maximum() * railStepsToUnitsScaleFactor;
    double c = leftLimitSlider->value() * railStepsToUnitsScaleFactor;

    leftLimitSpinBox->setMinimum(a);
    leftLimitSpinBox->setMaximum(b);
    leftLimitSpinBox->setValue(c);

    a = rightLimitSlider->minimum() * railStepsToUnitsScaleFactor;
    b = rightLimitSlider->maximum() * railStepsToUnitsScaleFactor;
    c = rightLimitSlider->value() * railStepsToUnitsScaleFactor;

    rightLimitSpinBox->setMinimum(a);
    rightLimitSpinBox->setMaximum(b);
    rightLimitSpinBox->setValue(c);

    a = positionSlider->minimum() * railStepsToUnitsScaleFactor;
    b = positionSlider->maximum() * railStepsToUnitsScaleFactor;
    c = positionSlider->value() * railStepsToUnitsScaleFactor;

    positionSpinBox->setMinimum(a);
    positionSpinBox->setMaximum(b);
    positionSpinBox->setValue(c);

    positionSpinBox->blockSignals(false);
    leftLimitSpinBox->blockSignals(false);
    rightLimitSpinBox->blockSignals(false);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onGoToPosition(double pos, int dim)
{
    if (dim == dimension){
        positionSpinBox->setValue(pos);
        qApp->processEvents();
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onLeftLimitSlider_valueChanged(int val)
{
    int position = positionSlider->value();
    if (val > position) {
        leftLimitSlider->setValue(position);
    } else {
        double lambda = (double)val / (double)leftLimitSlider->maximum();
        lambda *= (leftLimitSpinBox->maximum() - leftLimitSpinBox->minimum());
        lambda += leftLimitSpinBox->minimum();
        leftLimitSpinBox->setValue(lambda);
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onRightLimitSlider_valueChanged(int val)
{
    int position = positionSlider->value();
    if (val < position) {
        rightLimitSlider->setValue(position);
    } else {
        double lambda = (double)val / (double)rightLimitSlider->maximum();
        lambda *= (rightLimitSpinBox->maximum() - rightLimitSpinBox->minimum());
        lambda += rightLimitSpinBox->minimum();
        rightLimitSpinBox->setValue(lambda);
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onPositionSlider_valueChanged(int val)
{
    int leftLimit = leftLimitSlider->value();
    if (val < leftLimit) {
        positionSlider->setValue(leftLimit);
    } else {
        int rightLimit = rightLimitSlider->value();
        if (val > rightLimit) {
            positionSlider->setValue(rightLimit);
        } else {
            double lambda = (double)val / (double)positionSlider->maximum();
            lambda *= (positionSpinBox->maximum() - positionSpinBox->minimum());
            lambda += positionSpinBox->minimum();
            positionSpinBox->setValue(lambda);
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onLeftLimitSpinBox_valueChanged(double val)
{
    double lambda = (val - leftLimitSpinBox->minimum()) / (leftLimitSpinBox->maximum() - leftLimitSpinBox->minimum());
    leftLimitSlider->setValue(qRound(lambda * (leftLimitSlider->maximum() - leftLimitSlider->minimum())) + leftLimitSlider->minimum());
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onRightLimitSpinBox_valueChanged(double val)
{
    double lambda = (val - rightLimitSpinBox->minimum()) / (rightLimitSpinBox->maximum() - rightLimitSpinBox->minimum());
    rightLimitSlider->setValue(qRound(lambda * (rightLimitSlider->maximum() - rightLimitSlider->minimum())) + rightLimitSlider->minimum());
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onPositionSpinBox_valueChanged(double val)
{
    // CLAMP INCOMING VALUE BY THE LEFT AND RIGHT LIMIT SPIN BOXES
    double valP = qMin(rightLimitSpinBox->value(), qMax(leftLimitSpinBox->value(), val));

    // SEE IF THE INCOMING POSITION IS IN THE LIMITS OR NEED TO BE UPDATED
    if (val != valP){
        positionSpinBox->setValue(valP);
    } else {
        // CALCULATE THE NEW VALUE
        double lambda = (val - positionSpinBox->minimum()) / (positionSpinBox->maximum() - positionSpinBox->minimum());
        int newSliderPosition = qRound(lambda * (positionSlider->maximum() - positionSlider->minimum())) + positionSlider->minimum();

        // MAKE SURE NEW SLIDER POSITION IS WITHIN THE LIMITS THE LEFT AND RIGHT SLIDERS
        newSliderPosition= qMin(rightLimitSlider->value(), qMax(newSliderPosition, leftLimitSlider->value()));

        // SET THE SLIDER POSITION SO THAT IT REFLECTS THE SPIN BOX VALUE
        positionSlider->setValue(newSliderPosition);

        // TELL THE RAIL TO MOVE
        emit emitSetSliderPosition(qRound(val / railStepsToUnitsScaleFactor), dimension);

        // SET THE MOVING FLAG SO THAT WE KNOW WE ARE WAITING ON THE AN UPDATE POSITION SIGNAL
        isMovingCounter++;
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onCalibrateButton_clicked()
{
    this->setDisabled(true);
    scanState = ScanStateCalibrate;
    emit emitCalibrateSlider(dimension);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onCenterButton_clicked()
{
    setDisabled(true);

    // MOVE POSITION SLIDER TO ITS CENTER POSITION
    positionSlider->setValue((leftLimitSlider->value() + rightLimitSlider->value()) / 2);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onSettingsButton_clicked()
{
    LAUVelmexSettingsDialog d(dimension, this);
    d.setSpeed(velmexScannerSpeed);
    d.setUnits(unitIndex);
    d.setScanSteps(velmexScannerNumberOfScanSteps);
    if (controller) {
        d.setHomeDirection(controller->homeDirection(dimension));
    }
    if (d.exec() == QDialog::Accepted) {
        // EXTRACT THE PARAMETERS OF INTEREST
        velmexScannerSpeed = d.speed();
        velmexScannerNumberOfScanSteps = d.scanSteps();
        unitIndex = d.units();

        // PUSH HOME DIRECTION + SPEED DOWN TO THE FMC4030 (travel comes from calibration)
        if (controller) {
            controller->setHomeDirection(dimension, d.homeDirection());
        }
        emit emitSetVelocity(dimension, velmexScannerSpeed);

        // SAVE THE SETTINGS TO DISK
        QSettings settings;
        settings.setValue(QString("%1::velmexScannerNumberOfScanSteps").arg(railString), velmexScannerNumberOfScanSteps);
        settings.setValue(QString("%1::velmexScannerSpeed").arg(railString), velmexScannerSpeed);
        settings.setValue(QString("LAUVelmexWidget::unitIndex"), unitIndex);

        // unitIndex is shared by EVERY axis widget, so refresh the spin boxes on all of them
        // (not just this one).  findChildren keeps it on the GUI thread, unlike onUpdateUnits().
        if (parentWidget()) {
            QList<LAUVelmexWidget *> siblings = parentWidget()->findChildren<LAUVelmexWidget *>();
            for (int n = 0; n < siblings.count(); n++) {
                siblings.at(n)->updateScaleFactor();
            }
        } else {
            updateScaleFactor();
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onScanButton_clicked()
{
    if (movingScanModeFlag) {
        // DISABLE THIS WIDGET SO THAT USER CAN CANCEL THROUGH THE VIDEO RECORDING INTERFACE
        this->setDisabled(true);

        // SET THE NUMBER OF SCAN STEPS TO TWO, A STARTING POINT AND AN END POINT
        velmexScannerNumberOfScanSteps = 2;

        // TELL THE SCANNER TO BE READY FOR SCANNING
        emit emitTriggerScanner((float)positionSpinBox->value(), -1, velmexScannerNumberOfScanSteps);

        // SET SCANNING PARAMETERS
        scanState = ScanStateIterateMove;
        scanIntervalCounter = 0;

        if (positionSlider->value() == leftLimitSlider->value()) {
            // SINCE WE ARE AT THE LEFT MOST POSITION, CALL SLOT DIRECTLY
            onPositionSpinBox_valueChanged(leftLimitSlider->value());
        } else {
            // MOVE POSITION SLIDER TO ITS FARTHEST LEFT POSITION
            positionSlider->setValue(leftLimitSlider->value());
        }
    } else {
        // ASK THE USER HOW MANY SCAN STEPS THEY WANT THIS TIME AROUND
        bool ok = false;
        int val = QInputDialog::getInt(this, QString("Velmex Widget"), QString("How many scan steps?"), velmexScannerNumberOfScanSteps, 3, 1024, 1, &ok);
        if (ok) {
            velmexScannerNumberOfScanSteps = val;
            QSettings().setValue(QString("%1::velmexScannerNumberOfScanSteps").arg(railString), velmexScannerNumberOfScanSteps);

            // MAKE SURE THE USER TURNED OFF THE SCREEN SAVER SO THAT WE CAN CONTROL
            // THE BACKLIGHT DISPLAY OTHERWISE CANCEL THE SCAN
            int ret = QMessageBox::warning(this, QString("Calibration Widget"), QString("Please disable the screen saver before continuing."), QMessageBox::Ok | QMessageBox::Cancel);
            if (ret == QMessageBox::Ok) {
                initiateScan();
            }
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::initiateScan()
{
    // DISABLE THE INTERFACE DURING SCANNING
    this->setDisabled(true);

    // SET THE SPEED TO THE USER DEFINED RATE
    emit emitSetVelocity(dimension, velmexScannerSpeed);

    // GET THE NUMBER OF SCAN STEPS FROM SETTINGS
    velmexScannerNumberOfScanSteps = QSettings().value(QString("%1::velmexScannerNumberOfScanSteps").arg(railString), velmexScannerNumberOfScanSteps).toInt();

    // TELL THE SCANNER TO BE READY FOR SCANNING
    emit emitTriggerScanner((float)positionSpinBox->value(), -1, velmexScannerNumberOfScanSteps);

    // SET SCANNING PARAMETERS
    scanState = ScanStateIterateMove;
    scanIntervalCounter = 0;

    // DETERMINE IF WE ARE GOING LEFT TO RIGHT OR RIGHT TO LEFT
    if (leftToRightRasterFlag){
        if (positionSlider->value() == leftLimitSlider->value()) {
            // SINCE WE ARE AT THE LEFT MOST POSITION, CALL SLOT DIRECTLY
            onPositionSpinBox_valueChanged(leftLimitSpinBox->value());
        } else {
            // MOVE POSITION SLIDER TO ITS FARTHEST LEFT POSITION
            positionSlider->setValue(leftLimitSlider->value());
        }
    } else {
        if (positionSlider->value() == rightLimitSlider->value()) {
            // SINCE WE ARE AT THE LEFT MOST POSITION, CALL SLOT DIRECTLY
            onPositionSpinBox_valueChanged(rightLimitSpinBox->value());
        } else {
            // MOVE POSITION SLIDER TO ITS FARTHEST LEFT POSITION
            positionSlider->setValue(rightLimitSlider->value());
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onTriggerScanner(float pos, int n, int N)
{
    qDebug() << "LAUVelmexWidget::onTriggerScanner()" << pos << n << N << railString;

    // CHECK FOR ABORT CODE
    if (scanState == ScanStateWaitingOnScanner) {
        if (n < 0) {
            // RELOAD THE NUMBER OF SCAN STEPS IF IN MOVING SCAN MODE
            if (movingScanModeFlag) {
                velmexScannerNumberOfScanSteps = QSettings().value(QString("%1::velmexScannerNumberOfScanSteps").arg(railString), velmexScannerNumberOfScanSteps).toInt();
            }
            emit emitSetVelocity(dimension);

            // USER HAS CANCELLED SCANNING
            scanState = ScanStateNoState;
            setDisabled(false);
        } else {
            // MOVE THE VELMEX RAIL TO THE NEXT POSITION
            scanState = ScanStateIterateCapture;
            onUpdatePositon(n, dimension);
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onUpdatePositon(int val, int dim)
{
    //qDebug() << "LAUVelmexWidget::onUpdatePositon()" << val << dim;

    if (dim == dimension) {
        // SET FLAG SO WE KNOW THAT WE HAVE REACHED OUR POSITION
        isMovingCounter = qMax(0, --isMovingCounter);

        // EMIT NEW SLIDER POSITION FOR CURRENT DIMENSION
        emit emitSliderPosition((float)positionSpinBox->value());

        if (scanState == ScanStateNoState) {
            setDisabled(false);
        } else if (scanState == ScanStateInitiate) {
            ;
        } else if (scanState == ScanStateIterateMove) {
            // GET SCAN RANGE
            float stepSize = (float)(rightLimitSlider->value() - leftLimitSlider->value()) / (float)(velmexScannerNumberOfScanSteps - 1);
            float lambda = (float)(val - leftLimitSlider->value()) / (float)stepSize;

            // MAKE SURE WE ARE WITHIN 1% OF A STEP POSITION
            if (qAbs(lambda - qRound(lambda)) < 0.01f) {
                // CALCULATE WHAT STEP WE ARE AT BASED ON POSITION OF CARRIAGE ALONG RAIL
                int incomingCounter = (leftToRightRasterFlag) ? qRound((float)(val - leftLimitSlider->value()) / (float)stepSize) : qRound((float)(rightLimitSlider->value() - val) / (float)stepSize);

                // MAKE SURE WE ARE AT THE RIGHT STEP
                if (incomingCounter != scanIntervalCounter) {
                    // OTHERWISE KEEP MOVING
                    return;
                }
            } else if (LAUVelmexController::velmexRailHasReachedLimitSwitch){
                // LET'S SEE IF WE REACHED A LIMIT SWITCH
                qDebug() << "WE ARE AT THE LIMIT SWITCH";
            } else {
                // OTHERWISE KEEP MOVING
                return;
            }

            // CHANGE THE STATE TO WAIT FOR THE INCOMING CAPTURE TO COMPLETE
            scanState = ScanStateWaitingOnScanner;

            // TRIGGER ACTION LIKE TRIGGER SCANNER WHEN YOU REACH SCAN POSITION
            emit emitSliderPosition((float)positionSpinBox->value());
            emit emitTriggerScanner((float)positionSpinBox->value(), scanIntervalCounter, velmexScannerNumberOfScanSteps);
        } else if (scanState == ScanStateIterateCapture) {
            // CHANGE THE STATE TO WAIT FOR THE INCOMING CAPTURE TO COMPLETE
            scanState = ScanStateIterateMove;

            // INCREMENT THE SCAN INTERVAL COUNT SO WE KNOW TO MOVE TO THE NEXT POSITION
            scanIntervalCounter++;

            if (scanIntervalCounter < velmexScannerNumberOfScanSteps) {
                // GRAB THE LEFT AND RIGHT CARRIAGE POSITIONS
                int leftPosition = leftLimitSlider->value();
                int rigtPosition = rightLimitSlider->value();

                // CALCULATE THE NEXT POSITION DEPENDING ON RASTER DIRECTION
                int nextPosition;
                if (leftToRightRasterFlag){
                    nextPosition = qRound(scanIntervalCounter * ((float)(rigtPosition - leftPosition)) / ((float)(velmexScannerNumberOfScanSteps - 1)) + leftPosition);
                } else {
                    nextPosition = qRound(scanIntervalCounter * ((float)(leftPosition - rigtPosition)) / ((float)(velmexScannerNumberOfScanSteps - 1)) + rigtPosition);
                }

                // MOVE THE SLIDER TO THE NEXT POSITION
                positionSlider->setValue(nextPosition);
            } else {
                // RELOAD THE NUMBER OF SCAN STEPS IF IN MOVING SCAN MODE
                emit emitSetVelocity(dimension);
                if (movingScanModeFlag) {
                    velmexScannerNumberOfScanSteps = QSettings().value(QString("%1::velmexScannerNumberOfScanSteps").arg(railString), velmexScannerNumberOfScanSteps).toInt();
                }

                // SCANNING IS COMPLETE, SO RECORD THE STATE AND CENTER THE RAIL
                scanState = ScanStateNoState;
                if (autoCenteringFlag){
                    onCenterButton_clicked();
                }

                // TELL THE USER THAT WE ARE DONE SCANNING AS WELL
                emit emitTriggerScanner((float)positionSpinBox->value(), velmexScannerNumberOfScanSteps, velmexScannerNumberOfScanSteps);
            }
        } else if (scanState == ScanStateCalibrate) {
            ;
        } else {
            ;
        }
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
void LAUVelmexWidget::onUpdateLimits(int min, int max, int dim)
{
    if (dim == dimension) {
        scanState = ScanStateNoState;

        if (isRotaryType){
            // SET MAXIMUM VALUE TO DEFAULT FOR ROTARY DIAL
            max = 36000;
            railStepsToUnitsScaleFactor = 0.01;

            // UPDATE SLIDER WIDGETS BASED ON RAIL PARAMETERS
            leftLimitSlider->setMinimum(min);
            leftLimitSpinBox->setMinimum(min * railStepsToUnitsScaleFactor);
            leftLimitSlider->setMaximum(max);
            leftLimitSpinBox->setMaximum(max * railStepsToUnitsScaleFactor);
            leftLimitSlider->setValue(min);
            leftLimitSpinBox->setValue(min * railStepsToUnitsScaleFactor);

            rightLimitSlider->setMinimum(min);
            rightLimitSpinBox->setMinimum(min * railStepsToUnitsScaleFactor);
            rightLimitSlider->setMaximum(max);
            rightLimitSpinBox->setMaximum(max * railStepsToUnitsScaleFactor);
            rightLimitSlider->setValue(max);
            rightLimitSpinBox->setValue(max * railStepsToUnitsScaleFactor);

            positionSlider->setMinimum(min);
            positionSpinBox->setMinimum(min * railStepsToUnitsScaleFactor);
            positionSlider->setMaximum(max);
            positionSpinBox->setMaximum(max * railStepsToUnitsScaleFactor);

            // SET THE SLIDER TO ITS CURRENT POSITION
            positionSlider->setValue(min);
        } else {
            // UPDATE SLIDER WIDGETS BASED ON RAIL PARAMETERS
            leftLimitSlider->setMinimum(min);
            qDebug() << leftLimitSpinBox->minimum() << leftLimitSpinBox->value() << leftLimitSpinBox->maximum();
            leftLimitSpinBox->setMinimum(min * railStepsToUnitsScaleFactor);
            qDebug() << leftLimitSpinBox->minimum() << leftLimitSpinBox->value() << leftLimitSpinBox->maximum();
            leftLimitSlider->setMaximum(max);
            qDebug() << leftLimitSpinBox->minimum() << leftLimitSpinBox->value() << leftLimitSpinBox->maximum();
            leftLimitSpinBox->setMaximum(max * railStepsToUnitsScaleFactor);
            qDebug() << leftLimitSpinBox->minimum() << leftLimitSpinBox->value() << leftLimitSpinBox->maximum();
            leftLimitSlider->setValue(min);
            qDebug() << leftLimitSpinBox->minimum() << leftLimitSpinBox->value() << leftLimitSpinBox->maximum();
            leftLimitSpinBox->setValue(min * railStepsToUnitsScaleFactor);
            qDebug() << leftLimitSpinBox->minimum() << leftLimitSpinBox->value() << leftLimitSpinBox->maximum();

            rightLimitSlider->setMinimum(min);
            rightLimitSpinBox->setMinimum(min * railStepsToUnitsScaleFactor);
            rightLimitSlider->setMaximum(max);
            rightLimitSpinBox->setMaximum(max * railStepsToUnitsScaleFactor);
            rightLimitSlider->setValue(max);
            rightLimitSpinBox->setValue(max * railStepsToUnitsScaleFactor);
            qDebug() << rightLimitSlider->minimum() << rightLimitSlider->value() << rightLimitSlider->maximum();

            positionSlider->setMinimum(min);
            positionSpinBox->setMinimum(min * railStepsToUnitsScaleFactor);
            positionSlider->setMaximum(max);
            positionSpinBox->setMaximum(max * railStepsToUnitsScaleFactor);
            qDebug() << positionSlider->minimum() << positionSlider->value() << positionSlider->maximum();

            // SET THE SLIDER TO ITS CURRENT POSITION
            positionSlider->setValue(max);

            // NOW LET'S CENTER THE RAIL AND LET THE RETURNING SIGNAL TELL US WHERE THE RAIL IS CURRENTLY POSITIONED
            onCenterButton_clicked();
        }
    }
}

/****************************************************************************************************************/
/* FMC4030 backend -- the "controller" the Velmex widgets drive, swappable between the real   */
/* vendor SDK and a built-in software simulator (define FUYU_SIMULATE).  One widget "count" =  */
/* 1 micron (0.001 mm); the FMC4030 itself is commanded in native millimetres.                */
/****************************************************************************************************************/
#include <QThread>

static const int    FMC_MODE_ABSOLUTE   = 2;
static const int    FMC_STOP_DECEL      = 1;
static const double FMC_COUNTS_PER_MM   = 1000.0;   // 1 count = 1 micron
static const double FMC_DEFAULT_ACCEL   = 100.0;    // mm/s^2
static const double FMC_DEFAULT_HOMESPD = 10.0;     // mm/s
static const double FMC_DEFAULT_HOMEBACK = 5.0;     // mm
static const double FMC_MAXTRAVEL_MM     = 2000.0;  // safety bound for the "seek positive limit" move
static const int    FMC_UNSET           = -2000000000;
static const int    FMC_NO_LIB          = -100;       // fmcOpen() returns this when the DLL is unavailable

#ifdef FUYU_SIMULATE
namespace {
    // limitMax emulates the positive limit switch so calibration can "discover" the travel
    struct FmcSimAxis { double pos = 0.0; double target = 0.0; double speed = 20.0; bool moving = false; double limitMax = 250.0; };
    FmcSimAxis gFmcSim[VELMEXMAXDIMENSIONS];
    const double gFmcSimPoll = 0.5;   // matches the 500 ms controller timer
}
static int fmcOpen(int, const char *, int)
{
    for (int a = 0; a < VELMEXMAXDIMENSIONS; a++) { gFmcSim[a].pos = 0.0; gFmcSim[a].target = 0.0; gFmcSim[a].moving = false; }
    return (0);
}
static int fmcClose(int) { return (0); }
static int fmcJog(int, int axis, float pos, float speed, float, float, int)
{
    if (axis < 0 || axis >= VELMEXMAXDIMENSIONS) return (-1);
    gFmcSim[axis].target = (double)pos;
    gFmcSim[axis].speed = (speed > 0.0f ? (double)speed : 20.0);
    gFmcSim[axis].moving = true;
    return (0);
}
static int fmcHome(int, int axis, float, float, float, int dir)
{
    if (axis < 0 || axis >= VELMEXMAXDIMENSIONS) return (-1);
    gFmcSim[axis].target = (dir == 1) ? gFmcSim[axis].limitMax : 0.0;   // home toward + or - switch
    gFmcSim[axis].moving = true;
    return (0);
}
static int fmcStop(int, int axis, int)
{
    if (axis >= 0 && axis < VELMEXMAXDIMENSIONS) { gFmcSim[axis].target = gFmcSim[axis].pos; gFmcSim[axis].moving = false; }
    return (0);
}
static int fmcCheckStop(int, int axis)
{
    if (axis < 0 || axis >= VELMEXMAXDIMENSIONS) return (1);
    FmcSimAxis &s = gFmcSim[axis];
    if (s.moving == false) return (1);
    double step = s.speed * gFmcSimPoll;
    double next = (qAbs(s.target - s.pos) <= step) ? s.target : s.pos + ((s.target > s.pos) ? step : -step);

    // emulate the physical travel limits (the "limit switches"): halt at 0 or limitMax
    if (next >= s.limitMax) { s.pos = s.limitMax; s.moving = false; return (1); }
    if (next <= 0.0)        { s.pos = 0.0;        s.moving = false; return (1); }

    s.pos = next;
    if (s.pos == s.target) { s.moving = false; return (1); }
    return (0);
}
static int fmcGetPos(int, int axis, float *pos)
{
    if (axis < 0 || axis >= VELMEXMAXDIMENSIONS || pos == nullptr) return (-1);
    *pos = (float)gFmcSim[axis].pos;
    return (0);
}
#else
// ---- real vendor library, loaded at RUN TIME via QLibrary ----
// There is NO link-time dependency on the SDK, so the app launches even if FMC4030-Dll.dll
// is absent -- it simply behaves as "disconnected".  Drop FMC4030-Dll.dll next to the .exe
// (Windows) or libFMC4030-Lib.so on the library path (Linux) for real hardware.
#include <QLibrary>

namespace {
    // Plain function-pointer typedefs.  On 64-bit Windows all functions share one calling
    // convention, so this is correct.  For a 32-bit build, prefix each with __stdcall if the
    // DLL is __stdcall.
    typedef int (*FnOpen)(int, char *, int);
    typedef int (*FnClose)(int);
    typedef int (*FnJog)(int, int, float, float, float, float, int);
    typedef int (*FnCheck)(int, int);
    typedef int (*FnHome)(int, int, float, float, float, int);
    typedef int (*FnStop)(int, int, int);
    typedef int (*FnGetPos)(int, int, float *);

    struct FmcLib {
        QLibrary lib;
        FnOpen   open      = nullptr;
        FnClose  close     = nullptr;
        FnJog    jog       = nullptr;
        FnCheck  checkStop = nullptr;
        FnHome   home      = nullptr;
        FnStop   stop      = nullptr;
        FnGetPos getPos    = nullptr;
        bool loaded = false;

        FmcLib() {
#ifdef Q_OS_WIN
            lib.setFileName(QStringLiteral("FMC4030-Dll"));    // -> FMC4030-Dll.dll
#else
            lib.setFileName(QStringLiteral("FMC4030-Lib"));    // -> libFMC4030-Lib.so
#endif
            if (lib.load()) {
                open      = (FnOpen)   lib.resolve("FMC4030_Open_Device");
                close     = (FnClose)  lib.resolve("FMC4030_Close_Device");
                jog       = (FnJog)    lib.resolve("FMC4030_Jog_Single_Axis");
                checkStop = (FnCheck)  lib.resolve("FMC4030_Check_Axis_Is_Stop");
                home      = (FnHome)   lib.resolve("FMC4030_Home_Single_Axis");
                stop      = (FnStop)   lib.resolve("FMC4030_Stop_Single_Axis");
                getPos    = (FnGetPos) lib.resolve("FMC4030_Get_Axis_Current_Pos");
                loaded = open && close && jog && checkStop && home && stop && getPos;
                if (!loaded) {
                    qWarning() << "FMC4030 library loaded but a symbol was missing:" << lib.errorString();
                }
            } else {
                qWarning() << "FMC4030 library not found (running disconnected):" << lib.errorString();
            }
        }
    };

    // function-local static: built on first use, with thread-safe initialisation (C++11)
    FmcLib &fmcLib() { static FmcLib instance; return (instance); }
}

static int fmcOpen(int id, const char *ip, int port)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.open(id, (char *)ip, port) : FMC_NO_LIB);
}
static int fmcClose(int id)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.close(id) : FMC_NO_LIB);
}
static int fmcJog(int id, int axis, float p, float v, float a, float d, int m)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.jog(id, axis, p, v, a, d, m) : FMC_NO_LIB);
}
static int fmcHome(int id, int axis, float hs, float had, float fall, int dir)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.home(id, axis, hs, had, fall, dir) : FMC_NO_LIB);
}
static int fmcStop(int id, int axis, int mode)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.stop(id, axis, mode) : FMC_NO_LIB);
}
static int fmcCheckStop(int id, int axis)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.checkStop(id, axis) : 1);   // report "stopped" so the poll loop never hangs
}
static int fmcGetPos(int id, int axis, float *pos)
{
    FmcLib &L = fmcLib();
    return (L.loaded ? L.getPos(id, axis, pos) : FMC_NO_LIB);
}
#endif

static inline double fmcCountToMM(int count) { return ((double)count / FMC_COUNTS_PER_MM); }
static inline int    fmcMMToCount(double mm) { return ((int)qRound(mm * FMC_COUNTS_PER_MM)); }

/****************************************************************************************************************/
/* LAUVelmexSettingsDialog -- trimmed to FMC4030-relevant fields                             */
/****************************************************************************************************************/
LAUVelmexSettingsDialog::LAUVelmexSettingsDialog(int dim, QWidget *parent) : QDialog(parent)
{
    this->setLayout(new QVBoxLayout());
    this->layout()->setContentsMargins(6, 6, 6, 6);

    if (dim == 0) {
        this->setWindowTitle(QString("X Axis Settings"));
    } else if (dim == 1) {
        this->setWindowTitle(QString("Y Axis Settings"));
    } else if (dim == 2) {
        this->setWindowTitle(QString("Z Axis Settings"));
    } else {
        this->setWindowTitle(QString("Axis Settings"));
    }

    QGroupBox *groupBox = new QGroupBox(QString("FMC4030 Parameters"));
    groupBox->setLayout(new QFormLayout());
    groupBox->layout()->setContentsMargins(6, 6, 6, 6);
    groupBox->layout()->setSpacing(3);
    this->layout()->addWidget(groupBox);

    unitsComboBox = new QComboBox();
    unitsComboBox->addItem(QString("inches"));
    unitsComboBox->addItem(QString("millimeters"));
    unitsComboBox->setFixedWidth(150);
    unitsComboBox->setToolTip(QString("Display units for the position read-outs and the speed field.\n"
                                      "Applies to every axis at once."));
    reinterpret_cast<QFormLayout *>(groupBox->layout())->addRow(QString("Units:"), unitsComboBox);

    homeComboBox = new QComboBox();
    homeComboBox->addItem(QString("toward the motor end"));
    homeComboBox->addItem(QString("away from the motor end"));
    homeComboBox->setFixedWidth(150);
    homeComboBox->setToolTip(QString("Which end Calibrate homes to:\n"
                                     "  toward the motor end  = positive / LP limit switch\n"
                                     "  away from the motor end = negative / LN home switch (default)"));
    reinterpret_cast<QFormLayout *>(groupBox->layout())->addRow(QString("Home Toward:"), homeComboBox);

    speedSpinBox = new QDoubleSpinBox();
    speedSpinBox->setFixedWidth(150);
    speedSpinBox->setToolTip(QString("Rail speed for jogs, Center and Scan moves (1-200 mm/s).\n"
                                     "Shown in the selected unit (mm/s or in/s)."));
    reinterpret_cast<QFormLayout *>(groupBox->layout())->addRow(QString("Speed:"), speedSpinBox);

    // THE SPEED DISPLAY UNIT FOLLOWS THE UNITS COMBO; CONVERT THE VALUE WHEN UNITS CHANGE
    prevUnitIndex = unitsComboBox->currentIndex();
    applySpeedUnit(prevUnitIndex);
    connect(unitsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onUnitsChanged(int)));

    groupBox = new QGroupBox(QString("Scan Parameters"));
    groupBox->setLayout(new QFormLayout());
    groupBox->layout()->setContentsMargins(6, 6, 6, 6);
    groupBox->layout()->setSpacing(3);
    this->layout()->addWidget(groupBox);

    scanStepsSpinBox = new QSpinBox();
    scanStepsSpinBox->setRange(2, 1024);
    scanStepsSpinBox->setFixedWidth(150);
    scanStepsSpinBox->setToolTip(QString("Number of evenly-spaced stops in a Scan sweep,\n"
                                         "from the left limit to the right limit."));
    reinterpret_cast<QFormLayout *>(groupBox->layout())->addRow(QString("Scan Steps:"), scanStepsSpinBox);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    this->layout()->addWidget(buttonBox);
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));
}

void LAUVelmexSettingsDialog::applySpeedUnit(int idx)
{
    // 1..200 mm/s, displayed in the selected unit
    if (idx == 0) {   // inches
        speedSpinBox->setDecimals(3);
        speedSpinBox->setRange(1.0 / 25.4, 200.0 / 25.4);
        speedSpinBox->setSuffix(QString(" in/s"));
    } else {          // millimetres
        speedSpinBox->setDecimals(0);
        speedSpinBox->setRange(1.0, 200.0);
        speedSpinBox->setSuffix(QString(" mm/s"));
    }
}

void LAUVelmexSettingsDialog::onUnitsChanged(int idx)
{
    // Convert the displayed speed from the previous unit to the new one, then re-label.
    double mmPerSec = speedSpinBox->value() * speedMMPerUnit(prevUnitIndex);
    applySpeedUnit(idx);
    speedSpinBox->setValue(mmPerSec / speedMMPerUnit(idx));
    prevUnitIndex = idx;
}

/****************************************************************************************************************/
/* LAUVelmexController -- FMC4030-backed reimplementation (Ethernet, native mm).             */
/* Public interface is byte-for-byte the same as the original so the Velmex widgets are       */
/* untouched; only the guts now drive the FMC4030 instead of a serial VXM.                    */
/****************************************************************************************************************/
LAUVelmexController::LAUVelmexController(int dim, QObject *parent) : QObject(parent), state(StateStopped), deviceID(0), portNumber(8088), connectedFlag(false)
{
    channels << dim;
    initialize();
}

LAUVelmexController::LAUVelmexController(QList<int> dims, QObject *parent) : QObject(parent), state(StateStopped), channels(dims), deviceID(0), portNumber(8088), connectedFlag(false)
{
    if (channels.isEmpty()) {
        channels << 0;
    }
    initialize();
}

LAUVelmexController::~LAUVelmexController()
{
    QSettings settings;
    settings.setValue(QString("LAUVelmexController::ipAddress"), ipAddress);
    settings.setValue(QString("LAUVelmexController::portNumber"), (int)portNumber);
    settings.beginWriteArray(QString("LAUVelmexController"));
    for (int n = 0; n < channels.count(); n++) {
        settings.setArrayIndex(channels.at(n));
        settings.setValue(QString("LAUVelmexController::rightMostCount"), rightMostCount[channels.at(n)]);
        settings.setValue(QString("LAUVelmexController::homeDir"), homeDir[channels.at(n)]);
    }
    settings.endArray();
}

void LAUVelmexController::initialize()
{
    QSettings settings;
    ipAddress = settings.value(QString("LAUVelmexController::ipAddress"), QString("192.168.0.30")).toString();
    portNumber = (quint16)settings.value(QString("LAUVelmexController::portNumber"), (int)portNumber).toInt();

    for (int a = 0; a < VELMEXMAXDIMENSIONS; a++) {
        velocityMMPerSec[a] = 20;
        leftMostCount[a] = 0;
        rightMostCount[a] = fmcMMToCount(250.0);   // default 250 mm of travel
        currentCount[a] = 0;
        nextPosition[a] = 0;
        lastCommanded[a] = FMC_UNSET;
        homeDir[a] = 2;
    }

    settings.beginReadArray(QString("LAUVelmexController"));
    for (int n = 0; n < channels.count(); n++) {
        settings.setArrayIndex(channels.at(n));
        rightMostCount[channels.at(n)] = settings.value(QString("LAUVelmexController::rightMostCount"), rightMostCount[channels.at(n)]).toInt();
        homeDir[channels.at(n)] = settings.value(QString("LAUVelmexController::homeDir"), homeDir[channels.at(n)]).toInt();
    }
    settings.endArray();
}

bool LAUVelmexController::testConnection(const QString &ip, quint16 port, QString *message)
{
#ifdef FUYU_SIMULATE
    Q_UNUSED(ip);
    Q_UNUSED(port);
    if (message) {
        *message = QString("Simulation build — no real connection is attempted (the simulator always reports connected).");
    }
    return (true);
#else
    QByteArray ipBytes = ip.toLatin1();
    int rc = fmcOpen(0, ipBytes.constData(), (int)port);
    if (rc == 0) {
        fmcClose(0);
        if (message) {
            *message = QString("Success — reached the FMC4030 at %1:%2.").arg(ip).arg(port);
        }
        return (true);
    }
    if (message) {
        if (rc == FMC_NO_LIB) {
            *message = QString("FMC4030 control library (FMC4030-Dll) not loaded — cannot test. "
                               "Put FMC4030-Dll.dll next to the application.");
        } else {
            *message = QString("Failed — could not reach %1:%2 (code %3).").arg(ip).arg(port).arg(rc);
        }
    }
    return (false);
#endif
}

void LAUVelmexController::onStart()
{
    QByteArray ip = ipAddress.toLatin1();
    int rc = fmcOpen(deviceID, ip.constData(), (int)portNumber);
    if (rc != 0) {
        if (rc == FMC_NO_LIB) {
            // The vendor DLL could not be loaded at run time (missing / wrong bitness / bad symbol).
            emit emitError(QString("FMC4030 control library (FMC4030-Dll) not loaded — running disconnected. "
                                   "Put FMC4030-Dll.dll next to the application."));
        } else {
            emit emitError(QString("DLL loaded, but cannot reach the FMC4030 at %1:%2 (code %3).").arg(ipAddress).arg(portNumber).arg(rc));
        }
        connectedFlag = false;
        emit emitConnected(false);
        return;
    }

    connectedFlag = true;
    for (int n = 0; n < channels.count(); n++) {
        updateMotorPosition(channels.at(n));
        nextPosition[channels.at(n)] = currentCount[channels.at(n)];
        lastCommanded[channels.at(n)] = FMC_UNSET;
    }

    // POLL THE CONTROLLER FROM OUR THREAD, LIKE THE VELMEX TIMER
    startTimer(500);
    emit emitConnected(true);
}

void LAUVelmexController::onFinish()
{
    // Do NOT emit here: this runs during teardown and emitting back to a half-destroyed
    // widget would assert.  Just stop, close, and self-delete.
    if (connectedFlag) {
        for (int n = 0; n < channels.count(); n++) {
            fmcStop(deviceID, channels.at(n), FMC_STOP_DECEL);
        }
        fmcClose(deviceID);
        connectedFlag = false;
    }
    this->deleteLater();
}

bool LAUVelmexController::getModel(int dim)
{
    Q_UNUSED(dim);
    return (connectedFlag);
}

bool LAUVelmexController::setVelocity(int dim, int velocity)
{
    if (channels.contains(dim) == false) {
        return (false);
    }
    if (velocity < 1) {
        velocity = 1;
    } else if (velocity > 200) {
        velocity = 200;   // FMC4030 here is configured for <= 200 mm/s
    }
    velocityMMPerSec[dim] = velocity;
    return (true);
}

void LAUVelmexController::onCalibrateSlide(int dim)
{
    if (channels.contains(dim) == false || connectedFlag == false) {
        return;
    }
    // Discover BOTH ends from the limit switches, exactly like the Velmex controller:
    // home onto the near switch (sets zero) then seek the far switch (sets the travel).
    calibrateLeft(dim);
    calibrateRight(dim);
    emit emitCalibrationComplete(leftMostCount[dim], rightMostCount[dim], dim);
}

bool LAUVelmexController::calibrateLeft(int dim)
{
    if (channels.contains(dim) == false || connectedFlag == false) {
        return (false);
    }

    state = StateCalibrateLeft;
    if (fmcHome(deviceID, dim, (float)FMC_DEFAULT_HOMESPD, (float)FMC_DEFAULT_ACCEL, (float)FMC_DEFAULT_HOMEBACK, homeDir[dim]) != 0) {
        state = StateStopped;
        return (false);
    }

    // THE CALIBRATE PATH IS SYNCHRONOUS (like Velmex): block this controller thread until home settles
    for (int i = 0; i < 6000; i++) {
        if (fmcCheckStop(deviceID, dim) == 1) {
            break;
        }
        QThread::msleep(10);
    }

    updateMotorPosition(dim);
    leftMostCount[dim] = currentCount[dim];   // the controller zeroed the axis at the switch
    nextPosition[dim] = currentCount[dim];
    lastCommanded[dim] = FMC_UNSET;
    state = StateStopped;
    return (true);
}

bool LAUVelmexController::calibrateRight(int dim)
{
    if (channels.contains(dim) == false || connectedFlag == false) {
        return (false);
    }

    state = StateCalibrateRight;

    // DRIVE TOWARD THE POSITIVE LIMIT SWITCH.  The FMC4030 halts the axis when the switch
    // trips; FMC_MAXTRAVEL_MM is only a safety ceiling in case the switch is missing.
    if (fmcJog(deviceID, dim, (float)FMC_MAXTRAVEL_MM, (float)velocityMMPerSec[dim], (float)FMC_DEFAULT_ACCEL, (float)FMC_DEFAULT_ACCEL, FMC_MODE_ABSOLUTE) != 0) {
        state = StateStopped;
        return (false);
    }

    // BLOCK UNTIL THE AXIS STOPS (at the switch, or at the safety ceiling)
    for (int i = 0; i < 6000; i++) {
        if (fmcCheckStop(deviceID, dim) == 1) {
            break;
        }
        QThread::msleep(10);
    }

    updateMotorPosition(dim);
    rightMostCount[dim] = currentCount[dim];   // the far switch position == full travel
    nextPosition[dim] = currentCount[dim];
    lastCommanded[dim] = FMC_UNSET;
    state = StateStopped;
    return (true);
}

bool LAUVelmexController::updateMotorPosition(int dim)
{
    if (channels.contains(dim) == false || connectedFlag == false) {
        return (false);
    }
    float mm = 0.0f;
    if (fmcGetPos(deviceID, dim, &mm) != 0) {
        return (false);
    }
    currentCount[dim] = fmcMMToCount((double)mm);
    emit emitSliderPosition(currentCount[dim], dim);
    return (true);
}

LAUVelmexController::Position LAUVelmexController::moveToPosition(int pos, int dim)
{
    if (channels.contains(dim) == false) {
        return (PositionNoSuchChannel);
    }
    if (connectedFlag == false) {
        return (PositionPortNotOpen);
    }

    // REFRESH + BROADCAST THE LIVE POSITION
    updateMotorPosition(dim);

    // ALREADY THERE?
    if (qAbs(currentCount[dim] - pos) <= 1) {
        lastCommanded[dim] = pos;
        return (PositionReached);
    }

    // COMMAND THE MOVE ONCE PER TARGET CHANGE (the FMC4030 is non-blocking)
    if (lastCommanded[dim] != pos) {
        double mm = fmcCountToMM(pos);
        if (fmcJog(deviceID, dim, (float)mm, (float)velocityMMPerSec[dim], (float)FMC_DEFAULT_ACCEL, (float)FMC_DEFAULT_ACCEL, FMC_MODE_ABSOLUTE) == 0) {
            lastCommanded[dim] = pos;
        } else {
            return (PositionNotReached);
        }
    }

    // SETTLED?
    if (fmcCheckStop(deviceID, dim) == 1) {
        updateMotorPosition(dim);
        return (PositionReached);
    }
    return (PositionNotReached);
}

void LAUVelmexController::addWidget(QWidget *widget)
{
    widgets.append(widget);
}

void LAUVelmexController::removeWidget(QWidget *widget)
{
    int index = widgets.indexOf(widget);
    if (index > -1) {
        widgets.removeAt(index);
    }
}

void LAUVelmexController::onUpdateUnits()
{
    for (int n = 0; n < widgets.count(); n++) {
        reinterpret_cast<LAUVelmexWidget *>(widgets.at(n))->updateScaleFactor();
    }
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
// Factory-default controller address (used to seed the fields and the "Restore Defaults" button).
static const char *FUYU_DEFAULT_IP    = "192.168.0.30";
static const int    FUYU_DEFAULT_PORT = 8088;

LAUFuyuConnectDialog::LAUFuyuConnectDialog(QWidget *parent) : QDialog(parent)
{
    this->setWindowTitle(QString("FMC4030 Controller Connection"));

    QSettings settings;
    QString ip = settings.value(QString("LAUVelmexController::ipAddress"), QString(FUYU_DEFAULT_IP)).toString();
    int port   = settings.value(QString("LAUVelmexController::portNumber"), FUYU_DEFAULT_PORT).toInt();

    QGroupBox *box = new QGroupBox(QString("Controller Address"));
    QFormLayout *form = new QFormLayout();
    box->setLayout(form);

    // Plain IPv4 line edit (validated -- no zero-padding input mask, so the SDK gets a clean string).
    ipAddressLineEdit = new QLineEdit();
    QRegularExpression ipRegex(QStringLiteral("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$"));
    ipAddressLineEdit->setValidator(new QRegularExpressionValidator(ipRegex, this));
    ipAddressLineEdit->setText(ip);
    ipAddressLineEdit->setToolTip(QString("IPv4 address of the FMC4030 (factory default 192.168.0.30)."));
    form->addRow(QString("IP Address:"), ipAddressLineEdit);

    portSpinBox = new QSpinBox();
    portSpinBox->setRange(1, 65535);
    portSpinBox->setValue(port);
    portSpinBox->setToolTip(QString("TCP port of the FMC4030 (factory default 8088)."));
    form->addRow(QString("Port:"), portSpinBox);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QPushButton *defaultButton = buttonBox->addButton(QString("Default"), QDialogButtonBox::ResetRole);
    defaultButton->setToolTip(QString("Reset the IP address and port to the factory defaults."));
    connect(defaultButton, SIGNAL(clicked()), this, SLOT(onDefault()));
    connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
    connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), this, SLOT(reject()));

    this->setLayout(new QVBoxLayout());
    this->layout()->setContentsMargins(6, 6, 6, 6);
    this->layout()->addWidget(box);
    this->layout()->addWidget(buttonBox);

    // Size to the layout, then make it twice as wide and lock the geometry (non-resizable).
    this->adjustSize();
    this->setFixedSize(this->width() * 2, this->height());
}

QString LAUFuyuConnectDialog::ipAddress() const
{
    return (ipAddressLineEdit->text().trimmed());
}

quint16 LAUFuyuConnectDialog::portNumber() const
{
    return ((quint16)portSpinBox->value());
}

void LAUFuyuConnectDialog::onDefault()
{
    ipAddressLineEdit->setText(QString(FUYU_DEFAULT_IP));
    portSpinBox->setValue(FUYU_DEFAULT_PORT);
}

void LAUFuyuConnectDialog::accept()
{
    // Try to connect with the entered values.  On success, persist them (the controller reads
    // these same keys when it is constructed) and close the dialog.  On failure, show the error
    // and stay open so the user can correct the IP address or port and try again.
    QString message;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    bool ok = LAUVelmexController::testConnection(ipAddress(), portNumber(), &message);
    QApplication::restoreOverrideCursor();

    if (ok == false) {
        // Failed: show the error and stay open.  Leave QSettings untouched -- only a successful
        // connection persists the values.  (Use the dialog's Default button to reset the fields.)
        QMessageBox::critical(this, QString("FMC4030 Connection"), message);
        return;
    }

    // Success: persist the address/port (the controller reads these same keys on construction).
    QSettings settings;
    settings.setValue(QString("LAUVelmexController::ipAddress"), ipAddress());
    settings.setValue(QString("LAUVelmexController::portNumber"), (int)portNumber());
    QDialog::accept();
}
