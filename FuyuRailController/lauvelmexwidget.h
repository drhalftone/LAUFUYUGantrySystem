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

#ifndef LAUVELMEXWIDGET_H
#define LAUVELMEXWIDGET_H

#include <QList>
#include <QLabel>
#include <QEvent>
#include <QSlider>
#include <QDialog>
#include <QGroupBox>
#include <QVector4D>
#include <QComboBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QMessageBox>
#include <QMouseEvent>
#include <QInputDialog>
#include <QDoubleSpinBox>
#include <QProgressDialog>
#include <QDialogButtonBox>

#ifdef ENABLEPROSILICAFPGA
#define VELMEXSCANNERNUMBEROFSCANSTEPS 64
#else
#define VELMEXSCANNERNUMBEROFSCANSTEPS 512
#endif

class LAUVelmexDialog;
class LAUVelmexWidget;
class LAUVelmexController;

#define VELMEXMAXDIMENSIONS 3
#define VELMEXDEFAULTVELOCITY 3000
//#define SIMULATESLIDER

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUVelmexUserPathOffsetDialog : public QDialog
{
    Q_OBJECT

public:
    LAUVelmexUserPathOffsetDialog(int chns, QWidget *parent = nullptr);

    void setOffset(QVector4D point)
    {
        if (xSpinBox){
            xSpinBox->setValue(point.x());
        }
        if (ySpinBox){
            ySpinBox->setValue(point.y());
        }
        if (zSpinBox){
            zSpinBox->setValue(point.z());
        }
        if (wSpinBox){
            wSpinBox->setValue(point.w());
        }
    }

    QVector4D offset() const
    {
        QVector4D point(0,0,0,0);
        if (xSpinBox){
            point.setX(xSpinBox->value());
        }
        if (ySpinBox){
            point.setY(ySpinBox->value());
        }
        if (zSpinBox){
            point.setZ(zSpinBox->value());
        }
        if (wSpinBox){
            point.setW(wSpinBox->value());
        }
        return(point);
    }

protected:
    void accept();

private:
    int numChannels;

    QDoubleSpinBox *xSpinBox;
    QDoubleSpinBox *ySpinBox;
    QDoubleSpinBox *zSpinBox;
    QDoubleSpinBox *wSpinBox;
};

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUDoubleSpinBox : public QDoubleSpinBox
{
    Q_OBJECT

public:
    LAUDoubleSpinBox(QWidget *parent = nullptr) : QDoubleSpinBox(parent)
    {
        this->lineEdit()->installEventFilter(this);
    }

protected:
    bool eventFilter(QObject *obj, QEvent *event)
    {
        if (event->type() == QEvent::MouseButtonPress) {
            if (((QMouseEvent*)event)->button() == Qt::RightButton) {
                bool ok = false;
                double val = QInputDialog::getDouble(this, QString("Velmex Widget"), QString("Value?"), this->value(), this->minimum(), this->maximum(), this->decimals(), &ok);
                if (ok) {
                    this->setValue(val);
                }
                return(true);
            }
        } else if (event->type() == QEvent::ContextMenu) {
            return(true);
        }
        return (false);
    }

};

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUVelmexController : public QObject
{
    Q_OBJECT

public:
    explicit LAUVelmexController(int dim = 0, QObject *parent = nullptr);
    explicit LAUVelmexController(QList<int> dims, QObject *parent = nullptr);
    ~LAUVelmexController();

    void addWidget(QWidget *widget);
    void removeWidget(QWidget *widget);

    // Try to open (then close) the FMC4030 at ip:port; returns true on success and, if
    // message is non-null, a human-readable result.  Safe to call from the GUI thread
    // (used by the connection dialog's Test button).
    static bool testConnection(const QString &ip, quint16 port, QString *message = nullptr);

    bool isConnected() const
    {
        return (connectedFlag);
    }

    int dimensions() const
    {
        return (channels.count());
    }

    bool isValid() const
    {
        return (ipAddress.isEmpty() == false);
    }

    bool setVelocity(int dim = 0, int velocity = VELMEXDEFAULTVELOCITY);
    bool getModel(int dim = 0);

    // FMC4030 homing direction per axis: 1 = positive-end switch, 2 = negative-end switch
    void setHomeDirection(int dim, int dir)
    {
        if (dim >= 0 && dim < VELMEXMAXDIMENSIONS) {
            homeDir[dim] = (dir == 1 ? 1 : 2);
        }
    }
    int homeDirection(int dim) const
    {
        return ((dim >= 0 && dim < VELMEXMAXDIMENSIONS) ? homeDir[dim] : 2);
    }

    static bool velmexControllerIsWaitingWhileBusy;
    static bool velmexRailHasReachedLimitSwitch;

public slots:
    void onStart();
    void onFinish();
    void onUpdateUnits();
    void onCalibrateSlide(int dim = 0);
    void onSetVelocity(int dim = 0, int velocity = VELMEXDEFAULTVELOCITY)
    {
        setVelocity(dim, velocity);
    }
    void onMoveToPosition(int pos, int dim = 0)
    {
        if (channels.contains(dim)) {
            // SET THE BUSY FLAG HERE AND WAIT UNTIL WE REACH THIS REQUESTED POINT
            // IN OUR TIMER EVENT HANDLER BELOW
            velmexControllerIsWaitingWhileBusy = true;

            // STORE THE MOST RECENTLY REQUESTED POINT FOR THE TIMER EVENT HANDLER
            nextPosition[dim] = pos;
        }
    }

protected:
    void timerEvent(QTimerEvent *)
    {
        if (state == StateStopped) {
            // POLL EACH AXIS TOWARD ITS REQUESTED POSITION.  moveToPosition() talks to the
            // FMC4030 (or the built-in simulator) and reports whether the axis has settled.
            bool flag = true;
            for (int n = 0; n < channels.count(); n++) {
                if (moveToPosition(nextPosition[channels.at(n)], channels.at(n)) != PositionReached) {
                    flag = false;
                }
            }
            if (flag) {
                velmexControllerIsWaitingWhileBusy = false;
            }
        }
    }

private:
    enum State { StateCalibrateLeft, StateCalibrateRight, StateMoving, StateStopped };
    enum Position { PositionReached, PositionNotReached, PositionLimitSwitch, PositionPortNotOpen, PositionNoSuchChannel, PositionRailBusy };

    State state;
    QList<int> channels;

    // FMC4030 connection (Ethernet) -- replaces the Velmex serial port
    int deviceID;
    QString ipAddress;
    quint16 portNumber;
    bool connectedFlag;

    int velocityMMPerSec[VELMEXMAXDIMENSIONS];
    int leftMostCount[VELMEXMAXDIMENSIONS];
    int rightMostCount[VELMEXMAXDIMENSIONS];
    int currentCount[VELMEXMAXDIMENSIONS];
    int nextPosition[VELMEXMAXDIMENSIONS];
    int lastCommanded[VELMEXMAXDIMENSIONS];
    int homeDir[VELMEXMAXDIMENSIONS];

    QList<QWidget *> widgets;

    void initialize();
    bool calibrateLeft(int dim = 0);
    bool calibrateRight(int dim = 0);
    bool updateMotorPosition(int dim = 0);

    Position moveToPosition(int pos, int dim = 0);

signals:
    void emitConnected(bool state);
    void emitSliderPosition(int pos, int dim);
    void emitError(QString errorString);
    void emitCalibrationComplete(int left, int right, int dim);
    void emitScanComplete();
};

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUVelmexWidget : public QWidget
{
    Q_OBJECT

public:
    enum ScanState { ScanStateNoState, ScanStateIterateMove, ScanStateIterateCapture, ScanStateWaitingOnScanner, ScanStateInitiate, ScanStateComplete, ScanStateCalibrate };

    explicit LAUVelmexWidget(int dim = 0, LAUVelmexController *obj = nullptr, QWidget *parent = nullptr);
    ~LAUVelmexWidget();

    bool isConnected() const
    {
        return (isConnectedFlag);
    }

    bool isEnabled() const
    {
        return(groupBox->isChecked());
    }

    bool isDisabled() const
    {
        return(!groupBox->isChecked());
    }

    bool isValid() const
    {
        if (controller) {
            return (controller->isValid());
        }
        return (false);
    }

    bool isMoving() const
    {
        // WE ARE NOT MOVING IF THE COUNTER IS ZERO
        return(LAUVelmexController::velmexControllerIsWaitingWhileBusy);
    }

    int dim() const
    {
        return(dimension);
    }

    double left(bool *ok = nullptr) const
    {
        if (leftLimitSpinBox) {
            if (ok){
                *ok = true;
            }
            return (leftLimitSpinBox->value());
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    double right(bool *ok = nullptr) const
    {
        if (rightLimitSpinBox) {
            if (ok){
                *ok = true;
            }
            return (rightLimitSpinBox->value());
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    double maximum(bool *ok = nullptr) const
    {
        if (positionSpinBox) {
            if (ok){
                *ok = true;
            }
            return (positionSpinBox->maximum());
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    double minimum(bool *ok = nullptr) const
    {
        if (positionSpinBox) {
            if (ok){
                *ok = true;
            }
            return (positionSpinBox->minimum());
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    double position(bool *ok = nullptr) const
    {
        if (positionSpinBox) {
            if (ok){
                *ok = true;
            }
            return (positionSpinBox->value());
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    void enableLimitSliders(bool state = true);
    void disableLimitSliders(bool state = true)
    {
        enableLimitSliders(!state);
    }

    int scanSteps() const
    {
        return (velmexScannerNumberOfScanSteps);
    }

    int scanSpeed() const
    {
        return(velmexScannerSpeed);
    }

    bool movingScanMode() const
    {
        return (movingScanModeFlag);
    }

    void toggleLeftToRightRaster()
    {
        leftToRightRasterFlag = !leftToRightRasterFlag;
    }

    void enableLeftToRightRaster(bool state)
    {
        leftToRightRasterFlag = state;
    }

    void disableLeftToRightRaster(bool state)
    {
        leftToRightRasterFlag = !state;
    }

    void enableAutoCentering(bool state)
    {
        autoCenteringFlag = state;
    }

    void disableAutoCentering(bool state)
    {
        autoCenteringFlag = !state;
    }

    void enableScanButton(bool state)
    {
        if (scanButton){
            scanButton->setEnabled(state);
        }
    }

    void disableScanButton(bool state)
    {
        if (scanButton){
            scanButton->setDisabled(state);
        }
    }

    void enableMovingScanMode(bool state)
    {
        movingScanModeFlag = state;
    }

    void initiateScan();
    void updateScaleFactor();
    void setScanSteps(int val)
    {
        velmexScannerNumberOfScanSteps = val;
    }

    static QVector4D scannerPosition;

public slots:
    void onError(QString string)
    {
#ifndef SIMULATESLIDER
        setEnabled(false);
#endif
        QMessageBox::critical(this, QString("Velmex Slider Controller"), string, QMessageBox::Close);
    }
    void onConnected(bool state);
    void onTriggerScanner(float pos, int n, int N);      // TELLS WIDGET TO MOVE RAIL TO NEXT POSITION

    void onGoToPosition(double pos, int dim);
    void onCalibrateButton_clicked();
    void onCenterButton_clicked();
    void onScanButton_clicked();

protected:
    void showEvent(QShowEvent *)
    {
        this->setFixedHeight(this->height());
    }

private slots:
    void onLeftLimitSlider_valueChanged(int val);
    void onRightLimitSlider_valueChanged(int val);
    void onPositionSlider_valueChanged(int val);

    void onLeftLimitSpinBox_valueChanged(double val);
    void onRightLimitSpinBox_valueChanged(double val);
    void onPositionSpinBox_valueChanged(double val);

    void onSettingsButton_clicked();

    void onUpdateLimits(int min, int max, int dim = 0);   // RECIEVES CALIBRATED LEFT AND RIGHT LIMITS OF SLIDER
    void onUpdatePositon(int val, int dim = 0);           // RECIEVES SIGNAL FROM VELMEX CONTROLLER GIVING IT A NEW POSITION

private:
    int dimension;
    QString railString;
    ScanState scanState;

    static int unitIndex;

    bool leftToRightRasterFlag;
    bool autoCenteringFlag;
    bool isConnectedFlag;
    bool isRotaryType;
    bool movingScanModeFlag;
    int isMovingCounter;
    int velmexScannerSpeed;
    int velmexScannerNumberOfScanSteps;
    int motorClicksPerRotation;
    double railPitch;

    double railStepsToUnitsScaleFactor;
    int scanIntervalCounter;

    int leftSliderMemory, rightSliderMemory;

    QGroupBox *groupBox;
    QSlider *leftLimitSlider;
    QSlider *rightLimitSlider;
    QSlider *positionSlider;

    LAUDoubleSpinBox *leftLimitSpinBox;
    LAUDoubleSpinBox *rightLimitSpinBox;
    LAUDoubleSpinBox *positionSpinBox;

    QPushButton *calibrateButton;
    QPushButton *settingsButton;
    QPushButton *centerButton;
    QPushButton *scanButton;

    bool controllerFlag;
    QThread *controllerThread;
    LAUVelmexController *controller;

signals:
    void emitUpdateUnits();
    void emitSetVelocity(int dim);
    void emitSetVelocity(int dim, int velocity);
    void emitTriggerScanner(float pos, int n, int N);           // TELLS USER WHAT THE CURRENT SCAN ITERATION THIS IS
    void emitSetSliderPosition(int val, int dim);               // TELLS VELMEX RAIL TO GO TO NEW POSITION
    void emitCalibrateSlider(int dim);                          // TELLS VELMEX RAIL TO CALIBRATE ITSELF
    void emitSliderPosition(float val);                         // TELLS THE USER WHAT THE CURRENT RAIL POSITION
    void emitScan(int startIndex, int endIndex, int numSteps);  // TELLS VELMEX RAIL TO INITIATE A SCAN
};

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUMultiVelmexWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LAUMultiVelmexWidget(int dims, QWidget *parent = nullptr);
    explicit LAUMultiVelmexWidget(QList<int> channels, QWidget *parent = nullptr);
    ~LAUMultiVelmexWidget();

    bool isValid() const
    {
        if (widgets.count() > 0){
            if (controller) {
                return (controller->isValid());
            }
        }
        return (false);
    }

    int channels() const
    {
        return (widgets.count());
    }

    bool isMoving() const
    {
        // RETURN TRUE IF ANY RAIL IS MOVING
        for (int n = 0; n < widgets.count(); n++){
            if (widgets.at(n)->isMoving() == true){
                return(true);
            }
        }
        return(false);
    }

    void enableLimitSliders(bool state = true);
    void disableLimitSliders(bool state = true)
    {
        enableLimitSliders(!state);
    }

    double left(int dim, bool *ok = nullptr) const
    {
        for (int n = 0; n < widgets.count(); n++){
            if (widgets.at(n)->dim() == dim){
                return (widgets.at(n)->left(ok));
            }
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    double right(int dim, bool *ok = nullptr) const
    {
        for (int n = 0; n < widgets.count(); n++){
            if (widgets.at(n)->dim() == dim){
                return (widgets.at(n)->right(ok));
            }
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    double position(int dim, bool *ok = nullptr) const
    {
        for (int n = 0; n < widgets.count(); n++){
            if (widgets.at(n)->dim() == dim){
                return (widgets.at(n)->position(ok));
            }
        }
        if (ok){
            *ok = false;
        }
        return (0.0);
    }

    void onGoToPosition(double pos, int dim, bool *ok = nullptr) const
    {
        for (int n = 0; n < widgets.count(); n++){
            if (widgets.at(n)->dim() == dim){
                if (ok){
                    *ok = true;
                }
                widgets.at(n)->onGoToPosition(pos, dim);
            }
        }
        if (ok){
            *ok = false;
        }
        return;
    }

    QList<int> scanSteps() const
	{
		QList<int> stps;
		for (int n = 0; n < widgets.count(); n++){
			stps << widgets.at(n)->scanSteps();
		}
		return(stps);
	}

	void enableSerpentineRaster(bool flag = true)
    {
        employSerpentineRasterFlag = flag;
    }

    void disableSerpentineRaster(bool flag = true)
    {
        employSerpentineRasterFlag = !flag;
    }

    static int getDimensions();

    void scanUserPath(QList<QVector4D> points);

public slots:
    void onScanButton_clicked();

    void onError(QString string)
    {
        QMessageBox::critical(this, QString("Velmex Slider Controller"), string, QMessageBox::Close);
    }

    void onUpdateSliderPosition(float val);

    void onTriggerScanner(float pos, int n, int N);
    void onTriggerScannerA(float pos, int n, int N);
    void onTriggerScannerB(float pos, int n, int N);
    void onTriggerScannerC(float pos, int n, int N);
    void onTriggerScannerD(float pos, int n, int N);
    void onTriggerScannerE(float pos, int n, int N);
    void onTriggerScannerF(float pos, int n, int N);

    void onControllerDeleted()
    {
        controllerExistsFlag = false;
    }

protected:
    void showEvent(QShowEvent *)
    {
        // MAKE THE HEIGHT A FIXED VALUE SO THE USER CAN'T CHANGE IT
        this->setFixedHeight(this->height());

        // SEE IF WE ARE IN USER SCANNING MODE, AND IF SO, THEN INITIATE THE SCAN
        if (scanUserPathFlag){
            // DISABLE THIS WIDGET SO THE USER CAN'T MESS UP THE SLIDERS DURING SCANNING
            this->setEnabled(false);

            // SEND THE FIRST AXIS TO THE FIRST COORDINATE
            onUpdateSliderPosition(-1.0);
        }
    }

private:
    bool scanUserPathFlag;
    bool employSerpentineRasterFlag;
    LAUVelmexWidget::ScanState scanState;
    QList<int> enabledWidgetsList;
    QList<int> numberOfScansSteps;
    QList<int> currentScanStep;
    QList<float> scannerPosition;
    QList<QVector4D> scanUserPathPoints;
    QList<LAUVelmexWidget*> widgets;
    bool controllerExistsFlag = false;
    QThread *controllerThread;
    LAUVelmexController *controller;
    QProgressDialog *progressDialog;

    void initiateScanning();
    void completeScanning(bool centerRailsFlag);

signals:
    void emitSetVelocity(int dim);
    void emitSetVelocity(int dim, int velocity);
    void emitTriggerScanner(float pos, int n, int N);
    void emitTriggerScannerA(float pos, int n, int N);
    void emitTriggerScannerB(float pos, int n, int N);
    void emitTriggerScannerC(float pos, int n, int N);
    void emitTriggerScannerD(float pos, int n, int N);
    void emitTriggerScannerE(float pos, int n, int N);
    void emitTriggerScannerF(float pos, int n, int N);
};

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUVelmexDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUVelmexDialog(int dims, QDialog *parent = nullptr);
    explicit LAUVelmexDialog(QList<int> channels, QDialog *parent = nullptr);
    explicit LAUVelmexDialog(LAUVelmexWidget *widget, QDialog *parent = nullptr);
    explicit LAUVelmexDialog(LAUMultiVelmexWidget *widget, QDialog *parent = nullptr);

    bool isValid() const
    {
        if (velmexWidget) {
            return (velmexWidget->isValid());
        } else if (velmexMultiWidget){
            return (velmexMultiWidget->isValid());
        }
        return (false);
    }

    int channels() const
    {
        if (velmexMultiWidget){
            return(velmexMultiWidget->channels());
        } else if (velmexWidget){
            return(1);
        }
        return(0);
    }

    QList<int> scanSteps() const
    {
        QList<int> stps;
        if (velmexWidget) {
            stps << velmexWidget->scanSteps();
        } else if (velmexMultiWidget){
            for (int n = 0; n < channels(); n++){
                stps << velmexMultiWidget->scanSteps();
            }
        }
        return(stps);
    }

    QWidget* widget() const
    {
        if (velmexWidget) {
            return (velmexWidget);
        } else if (velmexMultiWidget){
            return (velmexMultiWidget);
        }
        return (nullptr);
    }

public slots:
    void onTriggerScanner(float pos, int n, int N)
    {
        if (n < 0){
            // WE SHOULD PREPARE FOR SCANNING BY INITIALIZING OURSELVES
        } else if (n >= N){
            // WE ARE DONE SCANNING SO NOW DEALLOCATE ANY SCANNING DATA STRUCTURES
        } else {
            // MAKE SURE THE USER TURNED OFF THE SCREEN SAVER SO THAT WE CAN CONTROL
            // THE BACKLIGHT DISPLAY OTHERWISE CANCEL THE SCAN
            int ret = QMessageBox::warning(this, QString("Calibration Widget"), QString("Scanning step %1 of %2.").arg(n+1).arg(N), QMessageBox::Ok | QMessageBox::Cancel);
            if (ret == QMessageBox::Ok) {
                if (velmexWidget){
                    velmexWidget->onTriggerScanner(pos, n, N);
                } else if (velmexMultiWidget){
                    velmexMultiWidget->onTriggerScanner(pos, n, N);
                }
            } else {
                if (velmexWidget){
                    velmexWidget->onTriggerScanner(pos, -1, N);
                } else if (velmexMultiWidget){
                    velmexMultiWidget->onTriggerScanner(pos, -1, N);
                }
            }
        }
    }

protected:
    inline void accept()
    {
        QDialog::accept();
    }

    inline void reject()
    {
        QDialog::reject();
    }

    void showEvent(QShowEvent *)
    {
        this->setFixedHeight(this->height());
    }

private:
    LAUVelmexWidget *velmexWidget;
    LAUMultiVelmexWidget *velmexMultiWidget;

private slots:
    void onRelayTriggerScanner(float pos, int n, int N)
    {
        emit emitTriggerScanner(pos, n, N);
    }

signals:
    void emitTriggerScanner(float pos, int n, int N);
};

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
class LAUVelmexSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUVelmexSettingsDialog(int dim = 0, QWidget *parent = nullptr);

    // speed is always stored/returned as mm/s; the spin box DISPLAYS it in the selected unit
    static double speedMMPerUnit(int idx) { return (idx == 0 ? 25.4 : 1.0); }   // in/s vs mm/s

    void setUnits(int index) { unitsComboBox->setCurrentIndex(index); }            // 0 = inches, 1 = mm
    void setSpeed(int mmPerSec) { speedSpinBox->setValue((double)mmPerSec / speedMMPerUnit(unitsComboBox->currentIndex())); }
    void setScanSteps(int val) { scanStepsSpinBox->setValue(val); }
    // combo index 0 = "toward the motor end" (SDK dir 1, + switch); 1 = "away" (SDK dir 2, - switch)
    void setHomeDirection(int dir) { homeComboBox->setCurrentIndex(dir == 1 ? 0 : 1); }

    int units() const { return (unitsComboBox->currentIndex()); }
    int speed() const { return (qRound(speedSpinBox->value() * speedMMPerUnit(unitsComboBox->currentIndex()))); }   // mm/s
    int scanSteps() const { return (scanStepsSpinBox->value()); }
    int homeDirection() const { return (homeComboBox->currentIndex() == 0 ? 1 : 2); }

protected:
    void showEvent(QShowEvent *)
    {
        this->setFixedSize(this->size());
    }

private slots:
    void onUnitsChanged(int idx);

private:
    void applySpeedUnit(int idx);

    int prevUnitIndex;
    QComboBox *unitsComboBox;
    QComboBox *homeComboBox;
    QDoubleSpinBox *speedSpinBox;
    QSpinBox *scanStepsSpinBox;
};

/****************************************************************************************************************/
/****************************************************************************************************************/
/* Startup dialog letting the user set the FMC4030 controller IP address and port, with a Test Connection     */
/* button.  Accepting it persists the values to the same QSettings keys the controller reads on construction. */
/****************************************************************************************************************/
class LAUFuyuConnectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LAUFuyuConnectDialog(QWidget *parent = nullptr);

    QString ipAddress() const;
    quint16 portNumber() const;

protected:
    void accept();   // try to connect; on success save IP/port and close, else show an error and stay

private slots:
    void onDefault();   // reset the IP/port fields to the factory defaults

private:
    QLineEdit *ipAddressLineEdit;
    QSpinBox *portSpinBox;
};

#endif // LAUVELMEXWIDGET_H
