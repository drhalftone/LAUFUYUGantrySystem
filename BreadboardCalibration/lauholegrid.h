/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAUHoleGrid -- C++ port of vision/holegrid.py: breadboard hole detector and lens-aware grid fit.
//
// 1. detectHoles(gray)   -> sub-pixel centres of hole-like ellipses.  The board's illumination is
//    estimated with a grey closing (at 1/4 scale), the image is divided by it, and dark blobs (plus
//    any bright thread highlight / lit hole wall touching them) become candidate holes.  Each
//    candidate's rim is then found along rays searched from the OUTSIDE in and fitted with an ellipse.
//
// 2. growLattice(pts)    -> relative lattice (i, j) for every detection on one regular grid.  Seeded at
//    a central hole with a full "+" of neighbours, flooded outward by linear extrapolation, then refined
//    with a GridModel (homography to the ideal image + radial lens distortion k1, k2) fitted robustly,
//    which re-predicts every cell in view and snaps it to the nearest detection.  Detections whose
//    ellipse differs from their lattice neighbours' (partly hidden holes) are dropped.
//
// Board: 25.4 mm hole pitch.
//
#ifndef LAUHOLEGRID_H
#define LAUHOLEGRID_H

#include <vector>
#include <opencv2/core.hpp>

namespace LAUHoleGrid
{
    static const double PITCH  = 25.4;   // hole pitch, mm
    static const double BRIGHT = 1.4;    // ratio above which a pixel is a highlight (thread / lit wall), not board

    struct Detections {
        std::vector<cv::Point2d> centers;   // px
        std::vector<cv::Size2d>  axes;      // full ellipse axes, px
        std::vector<double>      angles;    // deg
        std::vector<double>      contrast;  // mean ratio inside (0 = black, 1 = board)
        cv::Mat                  ratio;     // flat-fielded image (CV_32F), for debugging
        int count() const { return (int)centers.size(); }
    };

    // Lattice (i, j) -> pixel through a real lens: a homography to the ideal (pinhole) image, then radial
    // distortion (k1, k2, OpenCV's model) about the image centre.  f only sets the units of k1, k2.
    class GridModel
    {
    public:
        cv::Matx33d H = cv::Matx33d::eye();
        double k1 = 0.0, k2 = 0.0;
        cv::Point2d centre;
        double f = 1000.0;

        cv::Point2d ideal(const cv::Point2d &ij) const;
        cv::Point2d operator()(const cv::Point2d &ij) const;
        cv::Matx22d jacobian(const cv::Point2d &ij, double h = 1e-3) const;   // d(pixel)/d(i, j), columns = i, j
        double cornerShift(const cv::Size &size) const;                         // px the lens moves the image corner
    };

    struct Lattice {
        bool valid = false;
        std::vector<cv::Point>   ij;        // relative lattice indices
        std::vector<cv::Point2d> uv;        // px
        std::vector<int>         det;       // index into the detections
        GridModel                model;     // lattice -> pixel including lens distortion
        double                   rms = 0.0; // grid-model residual, px
        std::vector<cv::Point>   expected;  // cells the model puts in view with no detection on them
        int count() const { return (int)ij.size(); }
    };

    Detections detectHoles(const cv::Mat &gray, double minD = 10.0, double maxD = 70.0, double thresh = 0.6);

    // shapes: the detections (their ellipses drop partly hidden holes); may be null
    Lattice growLattice(const std::vector<cv::Point2d> &pts, const cv::Size &imageSize,
                        const Detections *shapes = nullptr, double tol = 0.28);
}

#endif // LAUHOLEGRID_H
