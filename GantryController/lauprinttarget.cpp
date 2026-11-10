/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauprinttarget.h"

#include <QMap>
#include <QFile>
#include <QStringList>
#include <QRegularExpression>

namespace
{
    const int TIFF_ENTRY_BYTES = 12;
    const int TIFF_TAG_XMP = 700;

    // the text of <lau:name>...</lau:name>, as LAUPrintTargetDialog::embedChartData escaped it
    QString element(const QString &xmp, const QString &name, bool *found)
    {
        const QRegularExpression pattern(QString("<lau:%1>(.*?)</lau:%1>").arg(name), QRegularExpression::DotMatchesEverythingOption);
        const QRegularExpressionMatch match = pattern.match(xmp);
        *found = match.hasMatch();
        QString text = match.captured(1);
        text.replace(QString("&lt;"), QString("<")).replace(QString("&gt;"), QString(">")).replace(QString("&quot;"), QString("\""));
        text.replace(QString("&#39;"), QString("'")).replace(QString("&amp;"), QString("&"));
        return text;
    }

    // a CGATS data line split into fields, double quotes keeping spaces together
    QStringList fields(const QString &line)
    {
        QStringList out;
        QString current;
        bool quoted = false, any = false;
        for (const QChar c : line) {
            if (c == QChar('"')) {
                quoted = !quoted;
                any = true;
            } else if (c.isSpace() && !quoted) {
                if (any) {
                    out << current;
                }
                current.clear();
                any = false;
            } else {
                current += c;
                any = true;
            }
        }
        if (any) {
            out << current;
        }
        return out;
    }
}

QByteArray LAUPrintTarget::readXmp(const QString &tiffPath, QString *error)
{
    const auto fail = [error](const QString &text) {
        if (error) {
            *error = text;
        }
        return QByteArray();
    };
    QFile file(tiffPath);
    if (!file.open(QIODevice::ReadOnly)) {
        return fail(file.errorString());
    }
    const QByteArray header = file.read(8);
    if (header.size() != 8 || (!header.startsWith("II") && !header.startsWith("MM"))) {
        return fail(QString("it isn't a TIFF"));
    }
    const bool little = header.startsWith("II");
    const auto number = [little](const QByteArray &b, int at, int size) {
        quint32 v = 0;
        for (int k = 0; k < size; k++) {
            const quint32 byte = (uchar)b[at + (little ? k : size - 1 - k)];
            v |= byte << (8 * k);
        }
        return v;
    };
    if (number(header, 2, 2) != 42) {
        return fail(QString("it isn't a classic TIFF"));
    }
    if (!file.seek(number(header, 4, 4))) {
        return fail(QString("its tag directory is missing"));
    }
    const QByteArray countBytes = file.read(2);
    if (countBytes.size() != 2) {
        return fail(QString("its tag directory is cut short"));
    }
    const int count = (int)number(countBytes, 0, 2);
    const QByteArray entries = file.read((qint64)count * TIFF_ENTRY_BYTES);
    if (entries.size() != count * TIFF_ENTRY_BYTES) {
        return fail(QString("its tag directory is cut short"));
    }
    for (int n = 0; n < count; n++) {
        const int at = n * TIFF_ENTRY_BYTES;
        if ((int)number(entries, at, 2) != TIFF_TAG_XMP) {
            continue;
        }
        const quint32 length = number(entries, at + 4, 4);
        if (length <= 4) {
            return entries.mid(at + 8, (int)length);
        }
        if (!file.seek(number(entries, at + 8, 4))) {
            return fail(QString("its XMP is missing"));
        }
        const QByteArray packet = file.read(length);
        return (packet.size() == (int)length) ? packet : fail(QString("its XMP is cut short"));
    }
    return fail(QString("it carries no chart data: make it again with Generate print target"));
}

bool LAUPrintTarget::load(const QString &tiffPath)
{
    QString why;
    const QString xmp = QString::fromUtf8(readXmp(tiffPath, &why));
    if (xmp.isEmpty()) {
        return fail(why);
    }
    bool haveTi1 = false, haveTi2 = false, haveCht = false, haveTurned = false, havePage = false;
    const QString ti1 = element(xmp, QString("ti1"), &haveTi1);
    const QString ti2 = element(xmp, QString("ti2"), &haveTi2);
    const QString cht = element(xmp, QString("cht"), &haveCht);
    const QString turnedText = element(xmp, QString("turned"), &haveTurned);
    const QStringList page = element(xmp, QString("page_mm"), &havePage).split(QChar(' '), Qt::SkipEmptyParts);
    if (!haveTi2) {
        return fail(QString("it carries no chart data: make it again with Generate print target"));
    }
    if (!haveCht || !havePage || page.count() != 2) {
        return fail(QString("it was made before targets recorded where their patches are: make it again with Generate print target"));
    }
    return setChart(ti1, ti2, cht, haveTurned && turnedText.trimmed() == QString("1"), page[0].toDouble(), page[1].toDouble());
}

bool LAUPrintTarget::setChart(const QString &ti1, const QString &ti2, const QString &cht, bool isTurned, double pageW, double pageH)
{
    ti1Text = ti1;
    ti2Text = ti2;
    chtText = cht;
    turned = isTurned;
    pageWidthMM = pageW;
    pageHeightMM = pageH;
    patches.clear();
    if (pageW <= 0.0 || pageH <= 0.0) {
        return fail(QString("its page size is missing"));
    }

    // .cht boxes: X <label> <label> _ _ w h x y 0 0, in mm on the page as printtarg wrote it
    QMap<QString, cv::Rect2d> boxes;
    bool inBoxes = false;
    for (const QString &raw : cht.split(QChar('\n'))) {
        const QStringList t = raw.split(QRegularExpression(QString("\\s+")), Qt::SkipEmptyParts);
        if (t.isEmpty()) {
            continue;
        }
        if (t[0] == QString("BOXES")) {
            inBoxes = true;
            continue;
        }
        if (inBoxes && t[0] != QString("X") && t[0] != QString("F") && t[0] != QString("Y") && t[0] != QString("D")) {
            inBoxes = false;
        }
        if (!inBoxes || t[0] != QString("X")) {
            continue;
        }
        if (t.count() < 9 || t[1] != t[2] || t[3] != QString("_")) {
            return fail(QString("its .cht has a patch layout this program doesn't read"));
        }
        cv::Rect2d box(t[7].toDouble(), t[8].toDouble(), t[5].toDouble(), t[6].toDouble());
        if (turned) {
            // the TIFF was turned a quarter turn counter-clockwise: old (x, y) is now (y, old width - x)
            box = cv::Rect2d(box.y, pageH - box.x - box.width, box.height, box.width);
        }
        boxes.insert(t[1], box);
    }
    if (boxes.isEmpty()) {
        return fail(QString("its .cht lists no patches"));
    }

    // .ti2 data
    QStringList format;
    bool inFormat = false, inData = false;
    QMap<QString, int> column;
    for (const QString &raw : ti2.split(QChar('\n'))) {
        const QString line = raw.trimmed();
        if (line == QString("BEGIN_DATA_FORMAT")) {
            inFormat = true;
            continue;
        }
        if (line == QString("END_DATA_FORMAT")) {
            inFormat = false;
            for (int k = 0; k < format.count(); k++) {
                column[format[k]] = k;
            }
            continue;
        }
        if (inFormat) {
            format << line.split(QRegularExpression(QString("\\s+")), Qt::SkipEmptyParts);
            continue;
        }
        if (line == QString("BEGIN_DATA")) {
            inData = true;
            for (const char *name : { "SAMPLE_ID", "SAMPLE_LOC", "CMYK_C", "CMYK_M", "CMYK_Y", "CMYK_K" }) {
                if (!column.contains(QString(name))) {
                    return fail(QString("its .ti2 has no %1").arg(name));
                }
            }
            continue;
        }
        if (line == QString("END_DATA")) {
            break;
        }
        if (!inData || line.isEmpty()) {
            continue;
        }
        const QStringList f = fields(line);
        if (f.count() < format.count()) {
            return fail(QString("its .ti2 has a short line"));
        }
        LAUPrintPatch patch;
        patch.sampleId = f[column["SAMPLE_ID"]];
        patch.location = f[column["SAMPLE_LOC"]];
        if (!boxes.contains(patch.location)) {
            return fail(QString("patch %1 isn't on the page").arg(patch.location));
        }
        patch.boxMM = boxes.value(patch.location);
        const char *cmyk[4] = { "CMYK_C", "CMYK_M", "CMYK_Y", "CMYK_K" };
        const char *xyz[3] = { "XYZ_X", "XYZ_Y", "XYZ_Z" };
        for (int k = 0; k < 4; k++) {
            patch.cmyk[k] = f[column[QString(cmyk[k])]].toDouble();
        }
        for (int k = 0; k < 3; k++) {
            patch.xyz[k] = column.contains(QString(xyz[k])) ? f[column[QString(xyz[k])]].toDouble() : 50.0;
        }
        patches.append(patch);
    }
    if (patches.isEmpty()) {
        return fail(QString("its .ti2 lists no patches"));
    }
    errorString.clear();
    return true;
}
