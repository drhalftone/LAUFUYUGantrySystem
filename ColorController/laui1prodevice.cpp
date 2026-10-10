#include "laui1prodevice.h"

#include <QDebug>
#include <QVector>
#include <algorithm>

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUi1ProDevice::deviceEventHandler(I1_DeviceHandle devHndl, I1_DeviceEvent event, void *context)
{
    // RUNS ON THE SDK'S EVENT THREAD; SIGNALS REACH RECEIVERS IN OTHER THREADS AS QUEUED CALLS.
    // NO SDK CALLS ARE ALLOWED IN HERE WHILE I1_TriggerMeasurement IS RUNNING.
    Q_UNUSED(devHndl);
    LAUi1ProDevice *object = reinterpret_cast<LAUi1ProDevice *>(context);
    if (!object) {
        return;
    }

    switch (event) {
        case eI1ProButtonPressed:
            emit object->buttonPressed();
            break;
        case eI1ProScanReadyToMove:
            emit object->scanReadyToMove();
            break;
        case eI1ProLampRestore:
            emit object->lampRestoreStarted();
            break;
        case eI1ProArrival:
            emit object->deviceArrived();
            break;
        case eI1ProDeparture:
            emit object->deviceDeparted();
            break;
        default:
            break;
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUi1ProDevice::LAUi1ProDevice(QObject *parent) : QObject(parent), device(nullptr), currentMode(ModeSpot)
{
    I1_RegisterDeviceEventHandler(deviceEventHandler, this);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
LAUi1ProDevice::~LAUi1ProDevice()
{
    close();
    I1_RegisterDeviceEventHandler(nullptr, nullptr);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::open()
{
    if (device) {
        return (true);
    }

    I1_DeviceHandle *devices = nullptr;
    I1_UInteger count = 0;
    I1_ResultType result = I1_GetDevices(&devices, &count);
    if (result != eNoError || count == 0 || devices == nullptr || devices[0] == nullptr) {
        errorString = QString("No i1Pro devices detected.");
        return (false);
    }

    result = I1_OpenDevice(devices[0]);
    if (result != eNoError && result != eDeviceAlreadyOpen) {
        errorString = decodeError(result);
        return (false);
    }
    device = devices[0];

    // X-RITE RECOMMENDS PRECISION CALIBRATION (CHECKS AND RESTORES LAMP DRIFT)
    I1_SetOption(device, I1_PRECISION_CALIBRATION_KEY, I1_YES);
    applyMeasurementConditions();
    errorString.clear();
    return (setMode(currentMode));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUi1ProDevice::close()
{
    if (device) {
        I1_CloseDevice(device);
        device = nullptr;
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUi1ProDevice::sdkVersion()
{
    const char *string = I1_GetGlobalOptionD(I1_SDK_VERSION);
    return (string ? QString(string) : QString());
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUi1ProDevice::option(const char *key) const
{
    if (!device) {
        return (QString());
    }
    const char *string = I1_GetOptionD(device, key);
    return (string ? QString(string) : QString());
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUi1ProDevice::serialNumber() const
{
    return (option(I1_SERIAL_NUMBER));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUi1ProDevice::hardwareRevision() const
{
    return (option(I1_HW_REVISION_KEY));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QStringList LAUi1ProDevice::availablePatchRecognitions() const
{
    return (option(I1_AVAILABLE_PATCH_RECOGNITIONS_KEY).split(QString(I1_VALUE_DELIMITER), Qt::SkipEmptyParts));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::hasZebraRulerSensor() const
{
    return (option(I1_HAS_ZEBRA_RULER_SENSOR_KEY) == QString(I1_YES));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::check(I1_ResultType result, const QString &context)
{
    if (result == eNoError) {
        return (true);
    }
    errorString = QString("%1: %2").arg(context).arg(decodeError(result));
    qWarning() << errorString;
    return (false);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
void LAUi1ProDevice::applyMeasurementConditions()
{
    // ARGYLL'S PCS IS D50 / 2 DEGREE XYZ; ASK THE SDK FOR EXACTLY THAT SO THE .TI3 NEEDS NO CONVERSION.
    // A FAILURE HERE IS NOT FATAL -- THE SPECTRA ARE WRITTEN TOO, AND COLPROF CAN WORK FROM THOSE.
    struct { const char *key; const char *value; } conditions[] = {
        { ILLUMINATION_KEY, ILLUMINATION_D50     },
        { OBSERVER_KEY,     OBSERVER_TWO_DEGREE  },
        { WHITE_BASE_KEY,   WHITE_BASE_ABSOLUTE  },
        { COLOR_SPACE_KEY,  COLOR_SPACE_CIEXYZ   },
    };
    for (const auto &condition : conditions) {
        I1_ResultType result = I1_SetOption(device, condition.key, condition.value);
        if (result != eNoError) {
            qWarning() << "i1Pro: unable to set" << condition.key << "to" << condition.value << ":" << decodeError(result);
        }
    }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::setMode(Mode mode)
{
    currentMode = mode;
    if (!device) {
        return (true);
    }
    const char *value = (mode == ModeScan) ? I1_REFLECTANCE_SCAN : I1_REFLECTANCE_SPOT;
    return (check(I1_SetOption(device, I1_MEASUREMENT_MODE, value), QString("Set measurement mode")));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::isCalibrated() const
{
    return (secondsUntilCalibrationExpires() > 0);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
int LAUi1ProDevice::secondsUntilCalibrationExpires() const
{
    QString string = option(I1_TIME_UNTIL_CALIBRATION_EXPIRE);
    return (string.isEmpty() ? -1 : string.toInt());
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::calibrate()
{
    if (!device) {
        errorString = QString("No i1Pro device is open.");
        return (false);
    }
    return (check(I1_Calibrate(device), QString("Calibrate")));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::readSample(int index, LAUColorPatch &patch)
{
    float triStimulus[TRISTIMULUS_SIZE];
    if (!check(I1_GetTriStimulus(device, triStimulus, index), QString("Read XYZ"))) {
        return (false);
    }
    float spectrum[SPECTRUM_SIZE];
    if (!check(I1_GetSpectrum(device, spectrum, index), QString("Read spectrum"))) {
        return (false);
    }
    float densities[DENSITY_SIZE] = { 0.0f, 0.0f, 0.0f, 0.0f };
    I1_Integer autoDensityIndex = 0;
    I1_GetDensities(device, densities, &autoDensityIndex, index);   // OPTIONAL; NOT ALL MODES REPORT DENSITY

    static_assert(SPECTRUM_SIZE == LAU_SPECTRUM_SIZE, "i1Pro spectrum size does not match LAU_SPECTRUM_SIZE");
    for (int k = 0; k < 3; k++) {
        patch.xyz[k] = triStimulus[k];
    }
    for (int k = 0; k < LAU_SPECTRUM_SIZE; k++) {
        patch.spectrum[k] = spectrum[k];
    }
    for (int k = 0; k < LAU_DENSITY_SIZE; k++) {
        patch.density[k] = densities[k];
    }
    patch.measured = true;
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::measureSpot(LAUColorPatch &patch)
{
    if (!device) {
        errorString = QString("No i1Pro device is open.");
        return (false);
    }
    if (currentMode != ModeSpot) {
        errorString = QString("measureSpot() requires spot mode.");
        return (false);
    }
    if (!check(I1_TriggerMeasurement(device), QString("Spot measurement"))) {
        return (false);
    }
    return (readSample(0, patch));
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
bool LAUi1ProDevice::scanLine(QList<LAUColorPatch> &line)
{
    if (!device) {
        errorString = QString("No i1Pro device is open.");
        return (false);
    }
    if (currentMode != ModeScan) {
        errorString = QString("scanLine() requires scan mode.");
        return (false);
    }
    if (line.count() < 6) {
        errorString = QString("A scan line needs at least 6 patches (X-Rite chart design rules).");
        return (false);
    }

    // WITH PREDICTED COLORS FOR EVERY PATCH, USE CORRELATION RECOGNITION AGAINST THEIR LAB VALUES;
    // OTHERWISE FALL BACK TO BASIC (CONTRAST-ONLY) PATCH RECOGNITION.  NEITHER NEEDS A ZEBRA RULER.
    bool hasExpected = true;
    for (const LAUColorPatch &patch : line) {
        hasExpected = hasExpected && patch.hasExpected;
    }
    bool correlate = hasExpected && availablePatchRecognitions().contains(QString(I1_PATCH_RECOGNITION_CORRELATION));
    if (correlate) {
        if (!check(I1_SetOption(device, I1_PATCH_RECOGNITION_KEY, I1_PATCH_RECOGNITION_CORRELATION), QString("Select correlation recognition")) ||
            !check(I1_SetOption(device, I1_REFERENCE_CHART_COLOR_SPACE_KEY, I1_REFERENCE_CHART_LAB), QString("Select Lab reference"))) {
            return (false);
        }
        QVector<float> reference;
        for (const LAUColorPatch &patch : line) {
            double lab[3];
            LAUColor::xyzToLab(patch.expectedXYZ, lab);
            reference << static_cast<float>(lab[0]) << static_cast<float>(lab[1]) << static_cast<float>(lab[2]);
        }
        if (!check(I1_SetReferenceChartLine(device, reference.constData(), line.count()), QString("Set reference line"))) {
            return (false);
        }
    } else if (!check(I1_SetOption(device, I1_PATCH_RECOGNITION_KEY, I1_PATCH_RECOGNITION_BASIC), QString("Select basic recognition"))) {
        return (false);
    }

    // BLOCKS FOR THE WHOLE PASS; scanReadyToMove() FIRES FROM THE SDK THREAD PART WAY THROUGH
    I1_ResultType result = I1_TriggerMeasurement(device);
    if (result != eNoError) {
        errorString = QString("Scan: %1").arg(decodeError(result));
        if (correlate) {
            errorString.append(QString(" (%1 of %2 patches recognized; slow the scan or widen the patches)")
                               .arg(option(I1_PATCH_RECOGNITION_RECOGNIZED_PATCHES)).arg(line.count()));
        }
        return (false);
    }

    int count = I1_GetNumberOfAvailableSamples(device);
    if (count != line.count()) {
        errorString = QString("Scan found %1 patches, expected %2.").arg(count).arg(line.count());
        return (false);
    }

    QList<LAUColorPatch> samples;
    for (int n = 0; n < count; n++) {
        LAUColorPatch sample;
        if (!readSample(n, sample)) {
            return (false);
        }
        samples << sample;
    }

    // THE SAMPLES COME BACK IN TRAVEL ORDER (OR CORRELATED ORDER, WHICH THE SDK SAYS MAY BE
    // REVERSED).  PICK THE ORIENTATION WHOSE COLORS BEST MATCH THE PREDICTIONS.
    bool reversed = false;
    if (hasExpected) {
        double forward = 0.0, backward = 0.0;
        for (int n = 0; n < count; n++) {
            double expected[3], measuredF[3], measuredB[3];
            LAUColor::xyzToLab(line.at(n).expectedXYZ, expected);
            LAUColor::xyzToLab(samples.at(n).xyz, measuredF);
            LAUColor::xyzToLab(samples.at(count - 1 - n).xyz, measuredB);
            forward  += LAUColor::deltaE76(expected, measuredF);
            backward += LAUColor::deltaE76(expected, measuredB);
        }
        reversed = (backward < forward);
    }

    for (int n = 0; n < count; n++) {
        const LAUColorPatch &sample = samples.at(reversed ? count - 1 - n : n);
        LAUColorPatch &patch = line[n];
        for (int k = 0; k < 3; k++) {
            patch.xyz[k] = sample.xyz[k];
        }
        std::copy(sample.spectrum, sample.spectrum + LAU_SPECTRUM_SIZE, patch.spectrum);
        std::copy(sample.density, sample.density + LAU_DENSITY_SIZE, patch.density);
        patch.measured = true;
    }
    errorString.clear();
    return (true);
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
QString LAUi1ProDevice::decodeError(int errorCode)
{
    switch (errorCode) {
        case eNoError:                  return QString("No error");
        case eException:                return QString("Internal exception");
        case eBadBuffer:                return QString("Buffer too small for data");
        case eInvalidHandle:            return QString("Invalid device handle (device unplugged)");
        case eInvalidArgument:          return QString("Invalid argument");
        case eDeviceNotOpen:            return QString("Device is not open");
        case eDeviceNotConnected:       return QString("Device is not connected");
        case eDeviceNotCalibrated:      return QString("Device is not calibrated or calibration expired");
        case eNoDataAvailable:          return QString("No measurement data available");
        case eNoMeasureModeSet:         return QString("No measurement mode has been set");
        case eDeviceAlreadyOpen:        return QString("Device is already open");
        case eDeviceAlreadyInUse:       return QString("Device is already in use by another application");
        case eDeviceCommunicationError: return QString("USB communication error; reconnect the device");
        case eUSBPowerProblem:          return QString("USB power problem detected");
        case eNotOnWhiteTile:           return QString("Device is not on its white tile (or slider is closed)");
        case eStripRecognitionFailed:   return QString("Patch recognition failed; scan again");
        case eChartCorrelationFailed:   return QString("Scan could not be matched to the reference line; scan again");
        case eEarlyScanStart:           return QString("Movement started before the device was ready; missed the first patches");
        case eIncompleteScan:           return QString("The scan did not cover all patches");
        default:                        return QString("i1Pro error code: %1").arg(errorCode);
    }
}
