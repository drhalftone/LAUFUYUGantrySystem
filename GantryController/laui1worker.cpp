/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laui1worker.h"

void LAUi1Worker::onStart()
{
    if (!device.isOpen() && !device.open()) {
        emit emitReady(false, QString("Couldn't open the i1 Pro: %1").arg(device.error()), 0);
        return;
    }
    if (!device.loadCalibration()) {
        emit emitReady(false, QString("Couldn't read the i1 Pro's stored calibration: %1").arg(device.error()), 0);
        return;
    }
    if (!device.calibrateOnTile()) {
        emit emitReady(false, QString("The i1 Pro couldn't calibrate on its white tile: %1").arg(device.error()), 0);
        return;
    }
    emit emitReady(true, QString(), device.serialNumber());
}

void LAUi1Worker::onMeasure(int index)
{
    const QVector<double> reflectance = device.measureReflectance();
    emit emitReading(index, reflectance, reflectance.isEmpty() ? device.error() : QString());
}
