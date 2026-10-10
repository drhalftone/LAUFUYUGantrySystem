#ifndef LAUI1PRODEVICE_H
#define LAUI1PRODEVICE_H

#include <QObject>
#include <QStringList>

#include "i1Pro.h"   // X-Rite i1Pro SDK 4.2.x C API
#include "lauargyllchart.h"

/****************************************************************************/
/* LAUi1ProDevice                                                           */
/*                                                                          */
/* Headless wrapper around the X-Rite i1Pro SDK, refactored out of          */
/* LAUEyeOneDialog so the gantry can drive the instrument without any UI.   */
/* Results land directly in LAUColorPatch objects, ready for writeTI3().    */
/*                                                                          */
/* THREADING: I1_TriggerMeasurement blocks for the whole of a scan, and the */
/* SDK delivers its events on its own thread.  For gantry scans, move this  */
/* object to a worker QThread and call scanLine() there; scanReadyToMove()  */
/* then reaches the motion controller through a queued connection.          */
/*                                                                          */
/* The SDK supports a single event handler, so create one instance only.   */
/****************************************************************************/
class LAUi1ProDevice : public QObject
{
    Q_OBJECT

public:
    enum Mode { ModeSpot, ModeScan };

    explicit LAUi1ProDevice(QObject *parent = nullptr);
    ~LAUi1ProDevice();

    bool open();
    void close();

    bool isOpen() const
    {
        return (device != nullptr);
    }

    QString error() const
    {
        return (errorString);
    }

    static QString sdkVersion();
    QString serialNumber() const;
    QString hardwareRevision() const;
    QStringList availablePatchRecognitions() const;
    bool hasZebraRulerSensor() const;

    // CALIBRATION IS KEPT PER MODE BY THE SDK; SELECT THE MODE FIRST, THEN CALIBRATE
    bool setMode(Mode mode);
    Mode mode() const
    {
        return (currentMode);
    }
    bool isCalibrated() const;
    int secondsUntilCalibrationExpires() const;
    bool calibrate();                               // DEVICE MUST BE ON ITS WHITE TILE

    // ONE SPOT READING INTO patch (ModeSpot)
    bool measureSpot(LAUColorPatch &patch);

    // ONE CONTINUOUS PASS OVER line (ModeScan).  BLOCKS UNTIL THE SCAN ENDS; EMITS
    // scanReadyToMove() WHEN THE HEAD MAY START MOVING.  RESULTS ARE MATCHED TO line
    // IN WHICHEVER DIRECTION (FORWARD OR REVERSED) BEST FITS THE EXPECTED COLORS.
    bool scanLine(QList<LAUColorPatch> &line);

    static QString decodeError(int errorCode);

signals:
    void buttonPressed();
    void deviceArrived();
    void deviceDeparted();
    void scanReadyToMove();
    void lampRestoreStarted();

private:
    I1_DeviceHandle device;
    Mode currentMode;
    QString errorString;

    QString option(const char *key) const;
    bool check(I1_ResultType result, const QString &context);
    void applyMeasurementConditions();
    bool readSample(int index, LAUColorPatch &patch);

    static void deviceEventHandler(I1_DeviceHandle devHndl, I1_DeviceEvent event, void *context);
};

#endif // LAUI1PRODEVICE_H
