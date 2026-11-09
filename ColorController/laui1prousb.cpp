/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laui1prousb.h"

#include <windows.h>

namespace
{
    const unsigned short I1PRO_VENDOR = 0x0971, I1PRO_PRODUCT = 0x2000;
    const int CONFIGURATION = 1, INTERFACE_NUMBER = 0;
    const int VENDOR_IN = 0xC0;                  // device to host, vendor request, device recipient
    const int TIMEOUT_MS = 2000;

    // the parts of libusb-win32's public structures this needs; the other fields are never touched
    const int LIBUSB_PATH_MAX = 512;

#pragma pack(push, 1)
    struct DeviceDescriptor
    {
        unsigned char bLength, bDescriptorType;
        unsigned short bcdUSB;
        unsigned char bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0;
        unsigned short idVendor, idProduct;
    };
#pragma pack(pop)

    struct UsbBus;
    struct UsbDevice
    {
        UsbDevice *next, *prev;
        char filename[LIBUSB_PATH_MAX];
        UsbBus *bus;
        DeviceDescriptor descriptor;
    };
    struct UsbBus
    {
        UsbBus *next, *prev;
        char dirname[LIBUSB_PATH_MAX];
        UsbDevice *devices;
    };

    using InitFn = void (*)();
    using FindFn = int (*)();
    using GetBussesFn = UsbBus *(*)();
    using OpenFn = void *(*)(UsbDevice *);
    using CloseFn = int (*)(void *);
    using SetConfigFn = int (*)(void *, int);
    using ClaimFn = int (*)(void *, int);
    using ReleaseFn = int (*)(void *, int);
    using ControlFn = int (*)(void *, int, int, int, int, char *, int, int);
    using ErrorFn = char *(*)();

    template <typename T> T symbol(void *library, const char *name)
    {
        return reinterpret_cast<T>(GetProcAddress(static_cast<HMODULE>(library), name));
    }
}

LAUi1ProUsb::~LAUi1ProUsb()
{
    close();
    if (library) {
        FreeLibrary(static_cast<HMODULE>(library));
    }
}

bool LAUi1ProUsb::open()
{
    if (handle) {
        return true;
    }
    if (!library) {
        library = LoadLibraryW(L"libusb0.dll");
        if (!library) {
            errorString = QString("libusb0.dll isn't installed. The i1 Pro needs the libusb-win32 driver.");
            return false;
        }
    }

    const auto usbInit = symbol<InitFn>(library, "usb_init");
    const auto findBusses = symbol<FindFn>(library, "usb_find_busses");
    const auto findDevices = symbol<FindFn>(library, "usb_find_devices");
    const auto getBusses = symbol<GetBussesFn>(library, "usb_get_busses");
    const auto usbOpen = symbol<OpenFn>(library, "usb_open");
    const auto usbClose = symbol<CloseFn>(library, "usb_close");
    const auto setConfiguration = symbol<SetConfigFn>(library, "usb_set_configuration");
    const auto claimInterface = symbol<ClaimFn>(library, "usb_claim_interface");
    const auto lastError = symbol<ErrorFn>(library, "usb_strerror");
    if (!usbInit || !findBusses || !findDevices || !getBusses || !usbOpen || !usbClose || !setConfiguration || !claimInterface || !lastError) {
        errorString = QString("libusb0.dll is missing functions this program needs.");
        return false;
    }

    usbInit();
    findBusses();
    findDevices();
    UsbDevice *found = nullptr;
    for (UsbBus *bus = getBusses(); bus && !found; bus = bus->next) {
        for (UsbDevice *device = bus->devices; device; device = device->next) {
            if (device->descriptor.idVendor == I1PRO_VENDOR && device->descriptor.idProduct == I1PRO_PRODUCT) {
                found = device;
                break;
            }
        }
    }
    if (!found) {
        errorString = QString("No i1 Pro on the libusb-win32 driver was found.");
        return false;
    }

    void *opened = usbOpen(found);
    if (!opened) {
        errorString = QString("Couldn't open the i1 Pro: %1").arg(QString::fromLocal8Bit(lastError()));
        return false;
    }
    if (setConfiguration(opened, CONFIGURATION) < 0 || claimInterface(opened, INTERFACE_NUMBER) < 0) {
        errorString = QString("Couldn't claim the i1 Pro: %1. Is another program using it?").arg(QString::fromLocal8Bit(lastError()));
        usbClose(opened);
        return false;
    }
    handle = opened;
    errorString.clear();
    return true;
}

void LAUi1ProUsb::close()
{
    if (handle) {
        const auto release = symbol<ReleaseFn>(library, "usb_release_interface");
        const auto usbClose = symbol<CloseFn>(library, "usb_close");
        if (release) {
            release(handle, INTERFACE_NUMBER);
        }
        if (usbClose) {
            usbClose(handle);
        }
        handle = nullptr;
    }
}

QByteArray LAUi1ProUsb::vendorRead(int request, int length)
{
    if (!handle) {
        errorString = QString("The i1 Pro isn't open.");
        return QByteArray();
    }
    const auto control = symbol<ControlFn>(library, "usb_control_msg");
    const auto lastError = symbol<ErrorFn>(library, "usb_strerror");
    QByteArray data(length, '\0');
    const int got = control ? control(handle, VENDOR_IN, request, 0, 0, data.data(), length, TIMEOUT_MS) : -1;
    if (got < 0) {
        errorString = QString("USB request 0x%1 failed: %2").arg(request, 2, 16, QChar('0')).arg(lastError ? QString::fromLocal8Bit(lastError()) : QString());
        return QByteArray();
    }
    data.truncate(got);
    return data;
}
