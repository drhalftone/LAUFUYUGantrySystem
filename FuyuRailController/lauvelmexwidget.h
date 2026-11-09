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
#include <QListWidget>
#include <QPointer>
#include <QTimer>
#include <QMap>

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

    // Widget counts always grow AWAY from the home switch.  Homing to the positive-end switch
    // (homeDir 1) zeroes the axis there and leaves the travel at negative controller mm, so flip
    // the sign at the SDK boundary to keep the sliders running 0 .. +travel either way.
    double axisSign(int dim) const
    {
        return ((dim >= 0 && dim < VELMEXMAXDIMENSIONS && homeDir[dim] == 1) ? -1.0 : 1.0);
    }

    static bool velmexControllerIsWaitingWhileBusy;
    static bool velmexRailHasReachedLimitSwitch;

public slots:
    void onStart();
    void onStopRails();    // STOPS EVERY AXIS WHERE IT IS, THEN emitStopComplete()
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
    void timerEvent(QTimerEvent *event)
    {
        if (event->timerId() == limitTimerId) {
            pollLimits();
            return;
        }
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
            for (int n = 0; n < channels.count(); n++) {
                savePosition(channels.at(n));
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

    // HARDWARE DIAGNOSTICS.  FMC4030_Jog_Single_Axis only reports *communication* errors
    // (-1..-8), so a return of 0 means "the packet arrived" -- NOT "the axis will move".  The
    // controller can accept a move and emit no pulses at all if its own axis parameters (lead,
    // subdivision, soft limits) are wrong, so log every motion command we issue.  Disable with
    // the QSettings key LAUVelmexController::diagnostics = false.
    //
    // FMC4030_Get_Machine_Status reports the limit-input state.  It once returned -5 and crashed
    // the process; the cause was an undersized output buffer (the DLL writes the whole ~660-byte
    // machine_status block) and calling it before the controller had settled after open.  It is
    // now called with a 1 KB buffer (fmcGetLimits) and works -- verified by tools/fmc_limits.py,
    // whose realPos field matches FMC4030_Get_Axis_Current_Pos.
    bool diagnosticsFlag;

    // Speed used for the two switch-seeking legs of Calibrate, in mm/s.  Tunable at run time via
    // the QSettings key LAUVelmexController::homeSpeed because it interacts with a limit we cannot
    // see: the FMC4030 abandons a zero-return after its own "reset timeout" (User Manual V2001,
    // p.17), so seek_speed x timeout is the furthest a seek can ever reach.  Too slow and the
    // switch is never found on a long rail; too fast and the carriage overshoots the switch, since
    // stopping distance grows as speed^2 / (2 x accel).
    double homeSpeedMMPerSec;

    int velocityMMPerSec[VELMEXMAXDIMENSIONS];
    int leftMostCount[VELMEXMAXDIMENSIONS];
    int rightMostCount[VELMEXMAXDIMENSIONS];
    int currentCount[VELMEXMAXDIMENSIONS];
    int nextPosition[VELMEXMAXDIMENSIONS];
    int lastCommanded[VELMEXMAXDIMENSIONS];
    int homeDir[VELMEXMAXDIMENSIONS];

    // Has this axis's position ever been READ from the controller?  Until it has, currentCount is
    // only an initialised 0 -- and because onStart() adopts currentCount as the move target, that
    // guess would be commanded as a real move, yanking the carriage to coordinate 0 on launch.
    // Never command motion from an unread axis.
    bool positionKnown[VELMEXMAXDIMENSIONS];

    // CALIBRATION SURVIVES AN APP RESTART BUT NOT A CONTROLLER POWER-CYCLE.  Calibrate zeroes the
    // FMC4030's own position counter at the left-end switch; the PC only stores the travel measured
    // from that zero.  Steppers have no encoder, so a controller that was switched off comes back
    // counting from wherever the carriage sits, and the saved travel no longer lines up.  To catch
    // that, each axis's position is saved whenever it changes (QSettings key
    // LAUVelmexController::axisN::lastCount) and compared with the controller's reading on connect;
    // a mismatch clears LAUVelmexController::axisN::zeroValid and warns the user to calibrate.
    int lastSavedCount[VELMEXMAXDIMENSIONS];
    void savePosition(int dim, bool force = false);
    void checkZeroAfterConnect();

    QList<QWidget *> widgets;

    // Limit-switch inputs are read on their own fast timer (and from inside the blocking calibrate
    // wait) and broadcast with emitLimitStatus() so every rail panel can show its two switches live.
    int limitTimerId = 0;
    void pollLimits();

    void initialize();
    bool calibrateLeft(int dim = 0);
    bool calibrateRight(int dim = 0);
    bool updateMotorPosition(int dim = 0);

    // Block the controller thread until the axis reports stopped.  Returns false on timeout, which
    // means the reported position is wherever the carriage got to rather than a limit switch.
    // elapsedMS receives how long the move actually took -- a seek that always ends after the same
    // DURATION regardless of speed is being cut off by the controller's reset timeout, not by a switch.
    bool waitForAxisToStop(int dim, const char *what, qint64 *elapsedMS = nullptr);

    // Drive toward targetMM, re-issuing the move until the carriage stops making progress (i.e. it
    // is against a limit switch or a soft travel limit).  Returns how many moves that took.
    int driveUntilBlocked(int dim, double targetMM, const char *what);

    Position moveToPosition(int pos, int dim = 0);

signals:
    void emitConnected(bool state);
    void emitSliderPosition(int pos, int dim);
    void emitError(QString errorString);
    void emitWarning(QString warningString);                 // advisory only: does NOT disable the panels
    void emitCalibrationComplete(int left, int right, int dim);
    void emitScanComplete();
    void emitStopComplete();
    void emitLimitStatus(int limitN, int limitP, bool ok);   // bit 0 = axis 0 (LN1/LP1) ... bit 2 = axis 2
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

    // Length in the units the spin boxes are showing (one count = 1 micron)
    double unitsFromMM(double mm) const
    {
        return (mm * 1000.0 * railStepsToUnitsScaleFactor);
    }

    // Sends the rail to pos (in spin box units) even if the spin box already shows pos, and returns
    // the target in controller counts.
    int commandPosition(double pos);

    // Puts the slider and spin box on the rail's real position without commanding a move
    void showActualPosition(int count);

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
    void onWarning(QString string)
    {
        QMessageBox::warning(this, QString("Calibration Needed"), string, QMessageBox::Ok);
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
    // connectNow = false BUILDS THE CONTROLS (DISABLED) WITHOUT TOUCHING THE NETWORK; CALL
    // connectToController() LATER TO OPEN THE FMC4030 AT THE IP/PORT SAVED IN QSETTINGS
    explicit LAUMultiVelmexWidget(QList<int> channels, QWidget *parent = nullptr, bool connectNow = true);
    ~LAUMultiVelmexWidget();

    void connectToController();

    // Moves the rails to where the camera is calibrated: X (single rail, SDK axis 0) at the middle of
    // its travel, Y (rail pair, SDK axis 2) 5 mm short of its idler-end switch.  Always answers with
    // emitHomeFinished(), including when it refuses to move.
    void goToHome();
    void abortHome();      // STOPS THE RAILS; emitHomeFinished() FOLLOWS ONCE THEY HAVE HALTED

    // The same move with an explicit target, in mm from each axis's zero; answers with emitHomeFinished()
    void moveRails(double xMM, double yMM);

    // Where this axis may be sent, in mm: its limit sliders, kept 5 mm clear of both end switches
    bool scanRange(int dim, double *lowMM, double *highMM) const;
    double reportedMM(int dim) const         // WHERE THE LAST MOVE ACTUALLY ENDED
    {
        return (homeCounts.value(dim) / 1000.0);
    }
    int homeDirection(int dim) const
    {
        return (controller ? controller->homeDirection(dim) : 2);
    }
    bool isHoming() const
    {
        return (homingFlag);
    }
    bool isControllerStarted() const
    {
        return (controllerStartedFlag);
    }

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
    void onWarning(QString string)
    {
        QMessageBox::warning(this, QString("Calibration Needed"), string, QMessageBox::Ok);
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
    bool controllerStartedFlag = false;
    QThread *controllerThread;
    LAUVelmexController *controller;
    QProgressDialog *progressDialog;

    bool homingFlag = false;
    QMap<int, int> homeTargets;             // axes still on their way, axis -> target in counts
    QMap<int, int> homeCounts;              // last position each axis reported, axis -> counts
    QTimer *homeTimer = nullptr;
    bool homeStopping = false;
    QString homeStopMessage;
    void stopHome(const QString &message);
    void startMove(bool useTarget, double xMM, double yMM);
    void finishHome(bool ok, const QString &message);

    void initiateScanning();
    void completeScanning(bool centerRailsFlag);

private slots:
    void onHomePosition(int count, int dim);   // LIVE POSITION FROM THE CONTROLLER WHILE GOING HOME
    void onRailsStopped();

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
    void emitHomeFinished(bool ok, QString message);
    void emitStopRails();
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
    // Left End combo: index 0 = "motor end" (SDK dir 1, + switch); 1 = "idler end" (SDK dir 2, - switch)
    void setHomeDirection(int dir) { homeComboBox->setCurrentIndex(dir == 1 ? 0 : 1); }

    int units() const { return (unitsComboBox->currentIndex()); }
    int speed() const { return (qRound(speedSpinBox->value() * speedMMPerUnit(unitsComboBox->currentIndex()))); }   // mm/s
    int scanSteps() const { return (scanStepsSpinBox->value()); }
    int homeDirection() const { return (homeComboBox->currentIndex() == 0 ? 1 : 2); }

public slots:
    void onLimitStatus(int limitN, int limitP, bool ok);   // live LN/LP inputs from the controller

protected:
    void showEvent(QShowEvent *)
    {
        this->setFixedSize(this->size());
    }
    void timerEvent(QTimerEvent *);   // grey the limit tiles if readings stop arriving

private slots:
    void onUnitsChanged(int idx);

private:
    void applySpeedUnit(int idx);
    void paintLimits();

    // Limit-switch strip: the LEFT-end switch on the left, the RIGHT-end switch on the right.  Which
    // physical end is left follows the Left End combo live, so changing it swaps the tiles at once.
    int axis = 0;
    QLabel *leftEndLabel = nullptr;
    QLabel *rightEndLabel = nullptr;
    int limitStaleTicks = 0;
    bool limitOk = false;
    int limitN = 0, limitP = 0;

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
