/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUPrintTarget -- a print target made by LAUPrintTargetDialog, read back from the TIFF alone: the .ti1, .ti2 and
// .cht it carries in its XMP metadata.  Every patch comes out with its place on the page (from the .cht, in mm from
// the page's top-left corner, x right and y down, turned with the TIFF if it was turned), its CMYK values and its
// expected XYZ (from the .ti2, matched by the patch's location label such as "A12").
//
#ifndef LAUPRINTTARGET_H
#define LAUPRINTTARGET_H

#include <QString>
#include <QVector>
#include <QByteArray>
#include <opencv2/core.hpp>

struct LAUPrintPatch
{
    QString sampleId;                            // the .ti2's SAMPLE_ID
    QString location;                            // its SAMPLE_LOC, the .cht's box label: strip letter and patch number
    cv::Rect2d boxMM;                            // on the page
    double cmyk[4] = { 0, 0, 0, 0 };             // 0..100
    double xyz[3] = { 0, 0, 0 };                 // expected, Y of the paper = 100

    cv::Point2d centreMM() const
    {
        return cv::Point2d(boxMM.x + boxMM.width / 2.0, boxMM.y + boxMM.height / 2.0);
    }
};

class LAUPrintTarget
{
public:
    bool load(const QString &tiffPath);
    bool setChart(const QString &ti1, const QString &ti2, const QString &cht, bool turned, double pageWidthMM, double pageHeightMM);

    QString error() const
    {
        return errorString;
    }
    bool isValid() const
    {
        return !patches.isEmpty();
    }

    QVector<LAUPrintPatch> patches;
    double pageWidthMM = 0.0, pageHeightMM = 0.0;
    bool turned = false;
    QString ti1Text, ti2Text, chtText;

    // the TIFF's XMP packet (tag 700), empty if it has none
    static QByteArray readXmp(const QString &tiffPath, QString *error = nullptr);

private:
    QString errorString;
    bool fail(const QString &text)
    {
        errorString = text;
        patches.clear();
        return false;
    }
};

#endif // LAUPRINTTARGET_H
