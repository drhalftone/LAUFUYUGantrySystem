/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laui1prousb.h"

#include <cstring>
#include <cmath>
#include <QList>
#include <windows.h>

namespace
{
    const unsigned short I1PRO_VENDOR = 0x0971, I1PRO_PRODUCT = 0x2000;
    const int CONFIGURATION = 1, INTERFACE_NUMBER = 0;
    const int VENDOR_IN = 0xC0;                  // device to host, vendor request, device recipient
    const int VENDOR_OUT = 0x40;                 // host to device, vendor request, device recipient
    const int TIMEOUT_MS = 2000;
    const int BULK_TIMEOUT_MS = 5000;
    const int BULK_IN = 0x82;
    const int REQUEST_RESET = 0xCA, REQUEST_READ_MEMORY = 0xC4;
    const char RESET_MASK = 0x1F;
    const int RESET_SETTLE_MS = 120;

    // where the instrument keeps its calibration, within its 8 KB memory; all big-endian
    const quint16 MEMORY_SIZE = 0x2000;
    const int MEMORY_SERIAL = 0x1E44, MEMORY_BAND_START = 4648, MEMORY_BAND_COUNT = 4792, MEMORY_BAND_WEIGHTS = 4936, MEMORY_TILE = 7248;
    const int WEIGHTS_PER_BAND = 16;

    // the measurement cycle, as observed: settings for the dark frames, then for the lit frames
    const int REQUEST_TRIGGER = 0xC0, REQUEST_SETTINGS = 0xC1, REQUEST_PREPARE = 0xCF, REQUEST_STATE = 0xD1;
    const char SETTINGS_DARK[8] = { 0x01, 0x25, 0x00, 0x00, 0x00, 0x03, 0x06, 0x00 };
    const char SETTINGS_LIT[8] = { 0x01, 0x25, 0x00, 0x13, 0x00, 0x03, 0x04, 0x00 };
    const int FIRST_CYCLE_WAIT_MS = 900, LAMP_REST_MS = 1500, CYCLES = 3;
    const int FRAMES_PER_SHOT = 3, FRAME_BYTES = 256, PACKET_BYTES = 64;
    const int FIRST_PACKET_TIMEOUT_MS = 4000, NEXT_PACKET_TIMEOUT_MS = 300;

    // counts above this are the dark level, a small negative offset wrapped around 16 bits
    const int WRAP_THRESHOLD = 60000;

    quint32 bigEndian32(const QByteArray &d, int at)
    {
        const uchar *p = reinterpret_cast<const uchar *>(d.constData()) + at;
        return ((quint32)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
    }

    double bigEndianFloat(const QByteArray &d, int at)
    {
        const quint32 raw = bigEndian32(d, at);
        float value;
        memcpy(&value, &raw, 4);
        return value;
    }

    // the mean of each pixel over the frames of one or more shots
    QVector<double> pixelMean(const QList<QByteArray> &shots)
    {
        QVector<double> mean(LAUi1ProUsb::PIXELS, 0.0);
        int frames = 0;
        for (const QByteArray &shot : shots) {
            const uchar *p = reinterpret_cast<const uchar *>(shot.constData());
            for (int f = 0; f < shot.size() / FRAME_BYTES; f++) {
                for (int i = 0; i < LAUi1ProUsb::PIXELS; i++) {
                    int value = (p[f * FRAME_BYTES + 2 * i] << 8) | p[f * FRAME_BYTES + 2 * i + 1];
                    if (value > WRAP_THRESHOLD) {
                        value -= 65536;
                    }
                    mean[i] += value;
                }
                frames++;
            }
        }
        for (double &v : mean) {
            v /= qMax(frames, 1);
        }
        return mean;
    }

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
    using ClearHaltFn = int (*)(void *, unsigned int);
    using BulkFn = int (*)(void *, int, char *, int, int);

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

bool LAUi1ProUsb::loadCalibration()
{
    if (!reset()) {
        return false;
    }
    const QByteArray memory = readMemory(0, MEMORY_SIZE);
    if (memory.size() != MEMORY_SIZE) {
        return false;
    }

    QVector<int> starts(BANDS);
    QVector<double> weights(BANDS * WEIGHTS_PER_BAND), tile(BANDS);
    for (int b = 0; b < BANDS; b++) {
        starts[b] = (int)bigEndian32(memory, MEMORY_BAND_START + 4 * b);
        double sum = 0.0;
        for (int w = 0; w < WEIGHTS_PER_BAND; w++) {
            weights[b * WEIGHTS_PER_BAND + w] = bigEndianFloat(memory, MEMORY_BAND_WEIGHTS + 4 * (b * WEIGHTS_PER_BAND + w));
            sum += weights[b * WEIGHTS_PER_BAND + w];
        }
        tile[b] = bigEndianFloat(memory, MEMORY_TILE + 4 * b);

        // a different instrument could lay its memory out differently: check before trusting any of it
        if ((int)bigEndian32(memory, MEMORY_BAND_COUNT + 4 * b) != WEIGHTS_PER_BAND || starts[b] < 0 || starts[b] + WEIGHTS_PER_BAND > PIXELS ||
            std::abs(sum - 1.0) > 0.01 || !(tile[b] > 0.05 && tile[b] < 1.2)) {
            errorString = QString("This i1 Pro's stored calibration isn't laid out as expected (band %1).").arg(b);
            return false;
        }
    }
    bandStart = starts;
    bandWeights = weights;
    tileReflectance = tile;
    serial = (int)bigEndian32(memory, MEMORY_SERIAL);
    whiteBands.clear();
    errorString.clear();
    return true;
}

bool LAUi1ProUsb::prepareToMeasure()
{
    if (!vendorWrite(REQUEST_RESET, QByteArray(1, RESET_MASK))) {
        return false;
    }
    Sleep(RESET_SETTLE_MS);
    return vendorWrite(REQUEST_PREPARE, QByteArray(1, 0x01)) && !vendorRead(REQUEST_STATE, 6).isEmpty();
}

QByteArray LAUi1ProUsb::trigger()
{
    if (!vendorWrite(REQUEST_TRIGGER, QByteArray())) {
        return QByteArray();
    }

    // In 64-byte pieces: the transfer ends on a packet boundary, so one large read would wait out its timeout.
    const int expected = FRAMES_PER_SHOT * FRAME_BYTES;
    QByteArray shot;
    while (shot.size() < expected) {
        const QByteArray piece = bulkRead(PACKET_BYTES, shot.isEmpty() ? FIRST_PACKET_TIMEOUT_MS : NEXT_PACKET_TIMEOUT_MS);
        if (piece.isEmpty()) {
            errorString = QString("The i1 Pro stopped after %1 of %2 bytes of a measurement. %3").arg(shot.size()).arg(expected).arg(errorString);
            return QByteArray();
        }
        shot += piece;
    }
    return shot;
}

QVector<double> LAUi1ProUsb::signalBands()
{
    if (bandStart.isEmpty()) {
        errorString = QString("The i1 Pro's stored calibration hasn't been read.");
        return QVector<double>();
    }

    // left idle the instrument stops answering the trigger until it has been through this again
    if (!prepareToMeasure()) {
        return QVector<double>();
    }

    // Dark from every cycle, lit from the last: the lamp is still warming over the first cycles, and the white tile
    // and the sample have to be compared at the same point in that warm-up.
    QList<QByteArray> dark;
    QByteArray lit;
    for (int cycle = 0; cycle < CYCLES; cycle++) {
        if (!vendorWrite(REQUEST_SETTINGS, QByteArray(SETTINGS_DARK, 8))) {
            return QVector<double>();
        }
        Sleep(cycle == 0 ? FIRST_CYCLE_WAIT_MS : LAMP_REST_MS);
        const QByteArray d = trigger();
        if (d.isEmpty() || !vendorWrite(REQUEST_SETTINGS, QByteArray(SETTINGS_LIT, 8))) {
            return QVector<double>();
        }
        lit = trigger();
        if (lit.isEmpty()) {
            return QVector<double>();
        }
        dark << d;
    }

    const QVector<double> black = pixelMean(dark), white = pixelMean({ lit });
    QVector<double> bands(BANDS, 0.0);
    for (int b = 0; b < BANDS; b++) {
        for (int w = 0; w < WEIGHTS_PER_BAND; w++) {
            const int pixel = bandStart[b] + w;
            bands[b] += bandWeights[b * WEIGHTS_PER_BAND + w] * (white[pixel] - black[pixel]);
        }
    }
    return bands;
}

bool LAUi1ProUsb::calibrateOnTile()
{
    const QVector<double> bands = signalBands();
    if (bands.isEmpty()) {
        return false;
    }
    for (int b = 0; b < BANDS; b++) {
        if (bands[b] <= 0.0) {
            errorString = QString("The white tile read too dark at band %1. Is the i1 Pro on its tile?").arg(b);
            return false;
        }
    }
    whiteBands = bands;
    errorString.clear();
    return true;
}

QVector<double> LAUi1ProUsb::measureReflectance()
{
    if (whiteBands.isEmpty()) {
        errorString = QString("The i1 Pro needs calibrating on its white tile first.");
        return QVector<double>();
    }
    const QVector<double> bands = signalBands();
    if (bands.isEmpty()) {
        return QVector<double>();
    }
    QVector<double> reflectance(BANDS);
    for (int b = 0; b < BANDS; b++) {
        reflectance[b] = 100.0 * bands[b] / whiteBands[b] * tileReflectance[b];
    }
    return reflectance;
}

bool LAUi1ProUsb::vendorWrite(int request, const QByteArray &data, int value)
{
    const auto control = symbol<ControlFn>(library, "usb_control_msg");
    const auto lastError = symbol<ErrorFn>(library, "usb_strerror");
    if (!handle || !control) {
        errorString = QString("The i1 Pro isn't open.");
        return false;
    }
    QByteArray copy = data;
    if (control(handle, VENDOR_OUT, request, value, 0, copy.data(), copy.size(), TIMEOUT_MS) < 0) {
        errorString = QString("USB request 0x%1 failed: %2").arg(request, 2, 16, QChar('0')).arg(lastError ? QString::fromLocal8Bit(lastError()) : QString());
        return false;
    }
    return true;
}

QByteArray LAUi1ProUsb::bulkRead(int length, int timeoutMS)
{
    const auto bulk = symbol<BulkFn>(library, "usb_bulk_read");
    const auto lastError = symbol<ErrorFn>(library, "usb_strerror");
    if (!handle || !bulk) {
        errorString = QString("The i1 Pro isn't open.");
        return QByteArray();
    }
    QByteArray data(length, '\0');
    const int got = bulk(handle, BULK_IN, data.data(), length, timeoutMS);
    if (got < 0) {
        errorString = QString("Bulk read failed: %1").arg(lastError ? QString::fromLocal8Bit(lastError()) : QString());
        return QByteArray();
    }
    data.truncate(got);
    return data;
}

bool LAUi1ProUsb::reset()
{
    if (!handle) {
        errorString = QString("The i1 Pro isn't open.");
        return false;
    }
    const auto clearHalt = symbol<ClearHaltFn>(library, "usb_clear_halt");
    const auto control = symbol<ControlFn>(library, "usb_control_msg");
    if (!clearHalt || !control) {
        errorString = QString("libusb0.dll is missing functions this program needs.");
        return false;
    }
    for (int endpoint : { 0x01, 0x03, 0x82, 0x84 }) {
        clearHalt(handle, endpoint);
    }
    char mask = RESET_MASK;
    if (control(handle, VENDOR_OUT, REQUEST_RESET, 0, 0, &mask, 1, TIMEOUT_MS) < 0) {
        errorString = QString("The i1 Pro didn't accept the reset.");
        return false;
    }
    Sleep(RESET_SETTLE_MS);
    return true;
}

QByteArray LAUi1ProUsb::readMemory(quint32 address, quint16 length)
{
    if (!handle) {
        errorString = QString("The i1 Pro isn't open.");
        return QByteArray();
    }
    const auto control = symbol<ControlFn>(library, "usb_control_msg");
    const auto bulkRead = symbol<BulkFn>(library, "usb_bulk_read");
    const auto lastError = symbol<ErrorFn>(library, "usb_strerror");
    if (!control || !bulkRead) {
        errorString = QString("libusb0.dll is missing functions this program needs.");
        return QByteArray();
    }

    // eight bytes: the address and the length, most significant byte first, then two unused
    char command[8] = { (char)(address >> 24), (char)(address >> 16), (char)(address >> 8), (char)address,
                        (char)(length >> 8), (char)length, 0, 0 };
    if (control(handle, VENDOR_OUT, REQUEST_READ_MEMORY, 0, 0, command, sizeof(command), TIMEOUT_MS) < 0) {
        errorString = QString("The i1 Pro refused the memory read: %1").arg(lastError ? QString::fromLocal8Bit(lastError()) : QString());
        return QByteArray();
    }

    QByteArray data(length, '\0');
    int total = 0;
    while (total < length) {
        const int got = bulkRead(handle, BULK_IN, data.data() + total, length - total, BULK_TIMEOUT_MS);
        if (got <= 0) {
            errorString = QString("The memory read stopped after %1 of %2 bytes: %3").arg(total).arg(length).arg(lastError ? QString::fromLocal8Bit(lastError()) : QString());
            return QByteArray();
        }
        total += got;
    }
    return data;
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
