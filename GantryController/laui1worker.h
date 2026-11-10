/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUi1Worker -- the i1 Pro (LAUi1ProUsb) in its own thread: every call to the instrument blocks, a reading for about
// six seconds, so none of it may run where the window is drawn.  onStart() opens it, reads its stored calibration and
// calibrates on the white tile; onMeasure() then takes one reading.
//
#ifndef LAUI1WORKER_H
#define LAUI1WORKER_H

#include <QObject>
#include <QString>
#include <QVector>

#include "laui1prousb.h"

class LAUi1Worker : public QObject
{
    Q_OBJECT

public:
    explicit LAUi1Worker(QObject *parent = nullptr) : QObject(parent) { }

public slots:
    void onStart();
    void onMeasure(int index);

signals:
    void emitReady(bool ok, QString message, int serial);
    void emitReading(int index, QVector<double> reflectance, QString error);   // 36 bands, 380..730 nm, percent

private:
    LAUi1ProUsb device;
};

#endif // LAUI1WORKER_H
