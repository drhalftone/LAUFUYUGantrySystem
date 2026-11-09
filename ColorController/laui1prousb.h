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

private:
    void *library = nullptr;
    void *handle = nullptr;
    QString errorString;
};

#endif // LAUI1PROUSB_H
