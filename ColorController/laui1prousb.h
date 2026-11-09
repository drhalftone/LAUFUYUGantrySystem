/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUi1ProUsb -- talks to a gray (Rev A-D) i1 Pro directly over USB, with neither the X-Rite SDK nor ArgyllCMS.
// The instrument must be on the libusb-win32 driver: on this PC it hangs on a GET_STATUS request under WinUSB and
// libusbK, and never starts.  libusb0.dll is loaded at run time and comes with that driver.
//
// Written from observing the instrument's USB traffic.  Only requests seen working are used, and it never writes to
// the instrument's memory.
//
#ifndef LAUI1PROUSB_H
#define LAUI1PROUSB_H

#include <QByteArray>
#include <QString>
#include <QVector>

class LAUi1ProUsb
{
public:
    ~LAUi1ProUsb();

    bool open();
    void close();
    bool isOpen() const
    {
        return (handle != nullptr);
    }
    QString error() const
    {
        return (errorString);
    }

    // a vendor control read; empty on failure
    QByteArray vendorRead(int request, int length);

    bool vendorWrite(int request, const QByteArray &data, int value = 0);
    QByteArray bulkRead(int length, int timeoutMS);

    // The start-up sequence: clears the pipes and resets the instrument.
    bool reset();

    // Reads length bytes of the instrument's stored calibration memory.  Empty on failure.
    QByteArray readMemory(quint32 address, quint16 length);

    // Spot measuring, in this order.  loadCalibration() reads the instrument's own tables, calibrateOnTile() needs the
    // i1 on its white tile, and measureReflectance() then returns 36 percentages, 380 to 730 nm in 10 nm steps.
    // A reading takes about 6 seconds: three dark-and-lit cycles, spaced so the lamp behaves as it did in testing.
    bool loadCalibration();
    bool calibrateOnTile();
    QVector<double> measureReflectance();

    int serialNumber() const
    {
        return (serial);
    }
    bool isCalibrated() const
    {
        return (!whiteBands.isEmpty());
    }

    static const int BANDS = 36, PIXELS = 128;

private:
    void *library = nullptr;
    void *handle = nullptr;
    QString errorString;

    int serial = 0;
    QVector<int> bandStart;                  // first sensor pixel of each band
    QVector<double> bandWeights;             // BANDS x 16
    QVector<double> tileReflectance;         // the white tile, as a fraction, from the instrument's memory
    QVector<double> whiteBands;              // what the sensor saw on the tile at the last calibration

    bool prepareToMeasure();
    QByteArray trigger();                    // one measurement: three frames of raw sensor data
    QVector<double> signalBands();           // lamp-on minus dark, resampled to the 36 bands; empty on failure
};

#endif // LAUI1PROUSB_H
