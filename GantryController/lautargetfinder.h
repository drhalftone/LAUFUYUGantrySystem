/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUTargetFinder -- C++ port of vision/target.py: swatch centres on the platen from the platen mosaic.
//
// No registration marks: the sheet's four edges are found where the white paper meets the dark platen, then,
// inside the sheet, the swatch block's four edges are found where the white margin meets the first swatch.
// Each edge is a straight-line fit through hundreds of sub-pixel edge points, robust to the odd scanline that
// runs down a gap between swatches or over a swatch nearly as light as the paper.  The block's corners and the
// chart layout (rows, columns, gap between swatches) give every swatch centre.
//
// Everything is in mosaic pixels; corners run (u0,v0) (u1,v0) (u1,v1) (u0,v1), u along the sheet's short side,
// roughly the picture's +x, and v roughly its +y.  The sheet is a rectangle, so it looks the same turned 180 deg:
// it must lie within +/-45 deg of that orientation.
//
#ifndef LAUTARGETFINDER_H
#define LAUTARGETFINDER_H

#include <vector>
#include <QString>
#include <QStringList>
#include <opencv2/core.hpp>

#include "lauprinttarget.h"

struct LAUTargetFinderResult
{
    bool ok = false;
    QString message;                             // plain-language summary, or why it failed

    std::vector<cv::Point2d> sheet;              // the paper's corners
    std::vector<cv::Point2d> block;              // the swatch block's corners
    std::vector<cv::Point2d> centres;            // every swatch, row by row
    double sheetW = 0.0, sheetL = 0.0;           // mm
    double blockW = 0.0, blockL = 0.0;
    double swatchW = 0.0, swatchH = 0.0;
    double edgeFit[4] = { 0, 0, 0, 0 };          // of each block edge's scanlines, the share on its line
    int edgeScanlines[4] = { 0, 0, 0, 0 };
    cv::Mat check;                               // BGR close-up of the sheet with what was found drawn on it

    // findPrint only
    QStringList locations;                       // each centre's patch, the target's SAMPLE_LOC
    int quarterTurns = 0;                        // how the printed page lies on the sheet, clockwise
    double matchScore = 0.0;                     // correlation of the whole patch block with its expected look, 0..1
    int windowsUsed = 0;                         // of the local matches that pinned it down
    double fitRmsMM = 0.0;                       // how well those agree with one straight placement of the page
};

class LAUTargetFinder
{
public:
    // mosaic: CV_8UC1, 0 where the camera never looked.  mmPerPixel: its resolution.
    static LAUTargetFinderResult find(const cv::Mat &mosaic, double mmPerPixel, int rows, int cols, double gapMM);

    // A target from LAUPrintTargetDialog: the paper's edges give the sheet, then the patch block is found on it by its
    // expected look -- each patch's place from the .cht and brightness from the .ti2 -- in any of four orientations,
    // first over the whole sheet and then pinned by local matches at its corners and centre.  White patches and the
    // printed labels don't disturb it, and the page may sit anywhere on the paper, slightly skewed or scaled.
    static LAUTargetFinderResult findPrint(const cv::Mat &mosaic, double mmPerPixel, const LAUPrintTarget &target);
};

#endif // LAUTARGETFINDER_H
