/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "laubreadboardcalibration.h"
#include "lauholegrid.h"

#include <set>
#include <cmath>
#include <algorithm>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QTextStream>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace LAUHoleGrid;

namespace
{
    // percentiles of every 4th pixel in both directions (like numpy's img[::4, ::4])
    void percentiles(const cv::Mat &img32, const std::vector<double> &q, std::vector<double> &out)
    {
        std::vector<float> v;
        v.reserve((img32.rows / 4 + 1) * (img32.cols / 4 + 1));
        for (int r = 0; r < img32.rows; r += 4) {
            const float *p = img32.ptr<float>(r);
            for (int c = 0; c < img32.cols; c += 4) {
                v.push_back(p[c]);
            }
        }
        out.clear();
        for (double pq : q) {
            const size_t k = std::min(v.size() - 1, (size_t)std::llround(pq / 100.0 * (v.size() - 1)));
            std::nth_element(v.begin(), v.begin() + k, v.end());
            out.push_back(v[k]);
        }
    }

    float medianOf(const cv::Mat &img32)
    {
        std::vector<float> v(img32.begin<float>(), img32.end<float>());
        const size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + m, v.end());
        return v[m];
    }

    // log frame, its wide-blurred local level, and the overall level
    void levels(const cv::Mat &raw16, cv::Mat &x, cv::Mat &lg, cv::Mat &local, float &level)
    {
        raw16.convertTo(x, CV_32F, 1.0, 1.0);
        cv::log(x, lg);
        const int s = 8;
        cv::Mat small;
        cv::resize(lg, small, cv::Size(lg.cols / s, lg.rows / s), 0, 0, cv::INTER_AREA);
        cv::GaussianBlur(small, small, cv::Size(0, 0), LAUBreadboardCalibration::NEIGHBOURHOOD_PX / s);
        cv::resize(small, local, lg.size(), 0, 0, cv::INTER_LINEAR);
        level = medianOf(small);
    }

    void header(cv::Mat &v, const std::vector<std::string> &lines, double scale = 1.0)
    {
        const int step = (int)(34 * scale);
        cv::rectangle(v, cv::Point(0, 0), cv::Point(v.cols, step * (int)lines.size() + (int)(14 * scale)), cv::Scalar(0, 0, 0), cv::FILLED);
        for (size_t n = 0; n < lines.size(); n++) {
            cv::putText(v, lines[n], cv::Point((int)(14 * scale), (int)(38 * scale) + step * (int)n), cv::FONT_HERSHEY_SIMPLEX,
                        1.0 * scale, cv::Scalar(255, 255, 255), std::max(1, (int)(2 * scale)), cv::LINE_AA);
        }
    }

    std::string fmt(const char *f, double a, double b = 0, double c = 0, double d = 0, double e = 0)
    {
        char buf[512];
        std::snprintf(buf, sizeof(buf), f, a, b, c, d, e);
        return buf;
    }

    cv::Point ipt(const cv::Point2d &p)
    {
        return cv::Point((int)std::lround(p.x), (int)std::lround(p.y));
    }

    void cross(cv::Mat &v, const cv::Point2d &p, const cv::Scalar &col)
    {
        const int x = (int)std::lround(p.x), y = (int)std::lround(p.y);
        cv::line(v, cv::Point(x - 9, y - 9), cv::Point(x + 9, y + 9), col, 2, cv::LINE_AA);
        cv::line(v, cv::Point(x - 9, y + 9), cv::Point(x + 9, y - 9), col, 2, cv::LINE_AA);
    }
}

// =============================================================================================== levelling
cv::Mat LAUBreadboardCalibration::picture(const cv::Mat &raw16)
{
    cv::Mat x, lg, local;
    float level;
    levels(raw16, x, lg, local, level);
    cv::Mat out = lg - STRENGTH * (local - level);
    std::vector<double> p;
    percentiles(out, { 0.5, 99.8 }, p);
    cv::Mat pic;
    out.convertTo(pic, CV_8U, 245.0 / std::max(p[1] - p[0], 1e-6), 5.0 - p[0] * 245.0 / std::max(p[1] - p[0], 1e-6));
    return pic;
}

cv::Mat LAUBreadboardCalibration::levelLinear(const cv::Mat &raw16)
{
    cv::Mat x, lg, local;
    float level;
    levels(raw16, x, lg, local, level);
    cv::Mat gain;
    cv::exp(-STRENGTH * (local - level), gain);
    cv::Mat out = x.mul(gain);
    std::vector<double> p;
    percentiles(out, { 99.8 }, p);
    return out / std::max(p[0], 1e-6);
}

QImage LAUBreadboardCalibration::toQImage(const cv::Mat &img)
{
    if (img.empty()) {
        return QImage();
    }
    if (img.type() == CV_8UC1) {
        return QImage(img.data, img.cols, img.rows, (int)img.step, QImage::Format_Grayscale8).copy();
    }
    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).copy();
}

// =============================================================================================== calibrate
LAUBreadboardCalibrationResult LAUBreadboardCalibration::calibrate(const cv::Mat &raw16, double exposureMs)
{
    LAUBreadboardCalibrationResult R;
    R.raw = raw16.clone();
    R.exposureMs = exposureMs;
    if (raw16.empty() || raw16.type() != CV_16UC1) {
        R.message = QString("No picture from the camera.");
        return R;
    }
    const int W = raw16.cols, H = raw16.rows;
    R.imageSize = cv::Size(W, H);
    const cv::Mat pic = picture(raw16);

    // ---- holes, detected on the LINEAR levelled frame (hole-to-board ratios intact)
    cv::Mat lin8;
    levelLinear(raw16).convertTo(lin8, CV_8U, 240.0);
    const Detections det = detectHoles(lin8);
    const Lattice lat = growLattice(det.centers, R.imageSize, &det);
    R.holesDetected = det.count();
    if (!lat.valid || lat.count() < 12) {
        R.message = QString("Couldn't find the breadboard's holes in the picture (found %1 hole-like spots). "
                            "Check the camera can see the board and the lighting.").arg(det.count());
        return R;
    }
    R.holesOnGrid = lat.count();
    R.holesExpected = (int)lat.expected.size();
    R.gridRms = lat.rms;

    // ---- planar calibration on the labelled holes
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    for (int n = 0; n < lat.count(); n++) {
        obj.emplace_back((float)(lat.ij[n].x * PITCH), (float)(lat.ij[n].y * PITCH), 0.0f);
        img.emplace_back((float)lat.uv[n].x, (float)lat.uv[n].y);
    }
    const double f0 = NOMINAL_FOCAL_MM / PIXEL_MM;
    cv::Mat K = (cv::Mat_<double>(3, 3) << f0, 0, W / 2.0, 0, f0, H / 2.0, 0, 0, 1);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    const int flags = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO
                      | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3;
    std::vector<cv::Mat> rv, tv;
    double rms = cv::calibrateCamera(std::vector<std::vector<cv::Point3f>>{ obj }, std::vector<std::vector<cv::Point2f>>{ img },
                                     R.imageSize, K, dist, rv, tv, flags);
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rv[0], tv[0], K, dist, proj);
    std::vector<double> err(obj.size());
    for (size_t n = 0; n < obj.size(); n++) {
        err[n] = std::hypot(proj[n].x - img[n].x, proj[n].y - img[n].y);
    }
    // drop gross outliers (> 3x RMS) and refit once
    std::vector<bool> keep(obj.size());
    std::vector<cv::Point3f> objK;
    std::vector<cv::Point2f> imgK;
    for (size_t n = 0; n < obj.size(); n++) {
        keep[n] = err[n] < std::max(3.0 * rms, 1.0);
        if (keep[n]) {
            objK.push_back(obj[n]);
            imgK.push_back(img[n]);
        }
    }
    if (objK.size() < obj.size()) {
        rms = cv::calibrateCamera(std::vector<std::vector<cv::Point3f>>{ objK }, std::vector<std::vector<cv::Point2f>>{ imgK },
                                  R.imageSize, K, dist, rv, tv, flags);
        cv::projectPoints(obj, rv[0], tv[0], K, dist, proj);
        for (size_t n = 0; n < obj.size(); n++) {
            err[n] = std::hypot(proj[n].x - img[n].x, proj[n].y - img[n].y);
        }
    }
    R.holesUsed = (int)objK.size();
    R.rms = rms;
    for (size_t n = 0; n < obj.size(); n++) {
        if (keep[n]) {
            R.maxError = std::max(R.maxError, err[n]);
        }
    }
    R.K = cv::Matx33d((double *)K.ptr());
    for (int k = 0; k < 5; k++) {
        R.dist[k] = dist.at<double>(k);
    }
    cv::Mat Rm;
    cv::Rodrigues(rv[0], Rm);
    const cv::Mat C = -Rm.t() * tv[0];                          // camera centre in the board frame, mm
    const cv::Mat axis = Rm.t() * (cv::Mat_<double>(3, 1) << 0, 0, 1);
    R.tiltDeg = std::acos(std::min(1.0, std::abs(axis.at<double>(2)))) * 180.0 / CV_PI;
    R.heightMM = std::abs(C.at<double>(2));
    R.focalMM = K.at<double>(0, 0) * PIXEL_MM;
    {
        std::vector<cv::Point2d> c0{ cv::Point2d(0, 0) }, c1;
        cv::undistortPoints(c0, c1, K, dist, cv::noArray(), K);
        R.cornerShiftPx = std::hypot(c1[0].x, c1[0].y);
    }

    // ---- picture: holes found, and the calibration's error at each (x50)
    {
        cv::Mat v;
        cv::cvtColor(pic, v, cv::COLOR_GRAY2BGR);
        std::set<int> on(lat.det.begin(), lat.det.end());
        for (int k = 0; k < det.count(); k++) {
            const int r = (int)(std::max(det.axes[k].width, det.axes[k].height) / 2) + 4;
            const bool g = on.count(k) > 0;
            cv::circle(v, ipt(det.centers[k]), r, g ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 165, 255), g ? 2 : 3, cv::LINE_AA);
        }
        for (const cv::Point &c : lat.expected) {
            cross(v, lat.model(cv::Point2d(c)), cv::Scalar(255, 0, 255));
        }
        for (size_t n = 0; n < obj.size(); n++) {
            const cv::Point2d p(img[n]), q(proj[n]);
            cv::line(v, ipt(p), ipt(p + 50.0 * (q - p)), keep[n] ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 0, 128), 2, cv::LINE_AA);
        }
        header(v, { fmt("%.0f holes on the grid   reprojection RMS %.2f px (red lines = error x50)", lat.count(), rms),
                    fmt("f %.2f mm   k1 %+.3f  k2 %+.3f   tilt %.1f deg   height %.0f mm", R.focalMM, R.dist[0], R.dist[1],
                        R.tiltDeg, R.heightMM) });
        R.holes = toQImage(v);
    }

    // ---- lens distortion removed
    const cv::Mat newK = cv::getOptimalNewCameraMatrix(K, dist, R.imageSize, 0);
    cv::Mat und;
    cv::undistort(pic, und, K, dist, newK);
    R.undistorted = toQImage(und);

    // ---- hole columns running into the distance, on the corrected picture
    std::vector<cv::Point2d> uvU;
    cv::undistortPoints(lat.uv, uvU, K, dist, cv::noArray(), newK);
    const cv::Matx22d Jm = lat.model.jacobian(cv::Point2d(0, 0));
    const int ax = (std::abs(Jm(1, 0)) > std::abs(Jm(1, 1))) ? 0 : 1;   // lattice axis whose step is mostly up/down
    struct Line { cv::Point2d c, d; std::vector<int> idx; };
    std::vector<Line> lines;
    std::vector<double> offs;
    std::set<int> keys;
    for (const cv::Point &p : lat.ij) {
        keys.insert(ax == 0 ? p.y : p.x);
    }
    for (int key : keys) {
        Line L;
        for (int n = 0; n < lat.count(); n++) {
            if ((ax == 0 ? lat.ij[n].y : lat.ij[n].x) == key) {
                L.idx.push_back(n);
            }
        }
        if (L.idx.size() < 4) {
            continue;
        }
        // total-least-squares line through the corrected hole centres
        cv::Point2d c(0, 0);
        for (int n : L.idx) {
            c += uvU[n];
        }
        c *= 1.0 / L.idx.size();
        double sxx = 0, sxy = 0, syy = 0;
        for (int n : L.idx) {
            const cv::Point2d e = uvU[n] - c;
            sxx += e.x * e.x; sxy += e.x * e.y; syy += e.y * e.y;
        }
        const double th = 0.5 * std::atan2(2 * sxy, sxx - syy);
        L.c = c;
        L.d = cv::Point2d(std::cos(th), std::sin(th));
        const cv::Point2d nrm(-L.d.y, L.d.x);
        for (int n : L.idx) {
            offs.push_back((uvU[n] - c).dot(nrm));
        }
        lines.push_back(L);
    }
    if (lines.size() >= 2) {
        double ss = 0, mx = 0;
        for (double o : offs) {
            ss += o * o;
            mx = std::max(mx, std::abs(o));
        }
        R.columns = (int)lines.size();
        R.straightRms = std::sqrt(ss / offs.size());
        R.straightMax = mx;
        // vanishing point: least-squares point closest to every line
        cv::Matx22d A(0, 0, 0, 0);
        cv::Vec2d b(0, 0);
        for (const Line &L : lines) {
            const cv::Vec2d nrm(-L.d.y, L.d.x);
            const cv::Matx22d nn = nrm * nrm.t();
            A += nn;
            b += nn * cv::Vec2d(L.c.x, L.c.y);
        }
        const cv::Vec2d vp2 = A.inv() * b;
        const cv::Point2d vp(vp2[0], vp2[1]);
        R.vanishingAbovePx = -vp.y;

        cv::Mat v;
        cv::cvtColor(und, v, cv::COLOR_GRAY2BGR);
        for (size_t n = 0; n < lines.size(); n++) {
            const Line &L = lines[n];
            const cv::Scalar col = (n % 2 == 0) ? cv::Scalar(0, 200, 255) : cv::Scalar(255, 200, 0);
            if (std::abs(L.d.y) > 1e-6) {
                const cv::Point2d p0 = L.c + ((0 - L.c.y) / L.d.y) * L.d, p1 = L.c + ((H - L.c.y) / L.d.y) * L.d;
                cv::line(v, ipt(p0), ipt(p1), col, 2, cv::LINE_AA);
            }
            for (int k : L.idx) {
                cv::circle(v, ipt(uvU[k]), 5, col, cv::FILLED, cv::LINE_AA);
            }
        }
        const std::string where = vp.y < 0 ? fmt("they meet %.0f px above the top of the picture", -vp.y)
                                           : fmt("they meet at (%.0f, %.0f)", vp.x, vp.y);
        header(v, { fmt("%.0f hole columns running into the distance (lens distortion removed)", (double)lines.size()),
                    fmt("holes off their straight line: RMS %.2f px, max %.2f px;   ", R.straightRms, R.straightMax) + where });
        R.lines = toQImage(v);

        // zoomed out, so the meeting point itself is visible
        const int padTop = (int)std::min(std::max(-vp.y + 150.0, 0.0), 6000.0);
        const double s = 1200.0 / (H + padTop);
        cv::Mat canvas(H + padTop, W, CV_8UC3, cv::Scalar(30, 30, 30));
        cv::Mat bottom = canvas(cv::Rect(0, padTop, W, H));
        cv::cvtColor(und, bottom, cv::COLOR_GRAY2BGR);
        const cv::Point2d off(0, padTop);
        for (size_t n = 0; n < lines.size(); n++) {
            const Line &L = lines[n];
            const cv::Scalar col = (n % 2 == 0) ? cv::Scalar(0, 200, 255) : cv::Scalar(255, 200, 0);
            const cv::Point2d p1 = L.c + ((H - L.c.y) / L.d.y) * L.d;
            cv::line(canvas, ipt(vp + off), ipt(p1 + off), col, std::max((int)(3 / s / 2), 2), cv::LINE_AA);
        }
        cv::circle(canvas, ipt(vp + off), (int)(12 / s), cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        cv::Mat small;
        cv::resize(canvas, small, cv::Size(), s, s, cv::INTER_AREA);
        cv::putText(small, "vanishing point", ipt((vp + off) * s + cv::Point2d(14, 6)), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        R.vanishing = toQImage(small);
    }

    R.ok = true;
    R.message = QString("Calibrated from %1 holes. Accuracy %2 px. Lens %3 mm; camera tilted %4°, %5 mm above the board.")
                    .arg(R.holesUsed).arg(R.rms, 0, 'f', 2).arg(R.focalMM, 0, 'f', 2).arg(R.tiltDeg, 0, 'f', 1).arg(R.heightMM, 0, 'f', 0);
    return R;
}

// =============================================================================================== save
bool LAUBreadboardCalibration::boardFootprint(const LAUBreadboardCalibrationResult &R, double *acrossMM, double *alongMM)
{
    if (!R.ok || R.heightMM <= 0.0) {
        return false;
    }
    const double W = R.imageSize.width - 1, H = R.imageSize.height - 1;
    const std::vector<cv::Point2d> corners{ { 0, 0 }, { W, 0 }, { 0, H }, { W, H } };
    std::vector<cv::Point2d> rays;
    cv::undistortPoints(corners, rays, cv::Mat(R.K), cv::Mat(1, 5, CV_64F, const_cast<double *>(R.dist)));

    const double tilt = R.tiltDeg * CV_PI / 180.0;
    double nearAcross = 0.0, nearestDepth = 1e12, lowest = 1e12, highest = -1e12;
    for (const cv::Point2d &ray : rays) {
        const double denominator = std::cos(tilt) + ray.y * std::sin(tilt);
        if (denominator <= 0.05) {
            return false;                                       // this corner looks at the horizon, not the board
        }
        const double depth = R.heightMM / denominator;          // along the optical axis, to the board
        const double reach = depth * (std::sin(tilt) - ray.y * std::cos(tilt));
        lowest = std::min(lowest, reach);
        highest = std::max(highest, reach);
        if (depth < nearestDepth) {
            nearestDepth = depth;
            nearAcross = 2.0 * depth * std::abs(ray.x);
        }
    }
    *acrossMM = nearAcross;
    *alongMM = highest - lowest;
    return true;
}

bool LAUBreadboardCalibration::save(const LAUBreadboardCalibrationResult &R, const QString &folder, QString *error)
{
    QDir dir(folder);
    if (!dir.mkpath(QString("."))) {
        if (error) {
            *error = QString("Can't create %1").arg(folder);
        }
        return false;
    }
    QJsonArray K;
    for (int r = 0; r < 3; r++) {
        K.append(QJsonArray{ R.K(r, 0), R.K(r, 1), R.K(r, 2) });
    }
    QJsonObject o;
    o["K"] = K;
    o["dist"] = QJsonArray{ R.dist[0], R.dist[1], R.dist[2], R.dist[3], R.dist[4] };
    o["rms_px"] = R.rms;
    o["image_size"] = QJsonArray{ R.imageSize.width, R.imageSize.height };
    o["pixel_mm"] = PIXEL_MM;
    o["f_mm"] = R.focalMM;
    o["tilt_deg"] = R.tiltDeg;
    o["camera_height_mm"] = R.heightMM;
    o["n_holes"] = R.holesUsed;
    o["exposure_ms"] = R.exposureMs;
    o["source"] = QString("BreadboardCalibration (Qt)");
    QFile jf(dir.filePath("calibration.json"));
    if (!jf.open(QIODevice::WriteOnly)) {
        if (error) {
            *error = jf.errorString();
        }
        return false;
    }
    jf.write(QJsonDocument(o).toJson());
    jf.close();

    QFile rf(dir.filePath("report.txt"));
    if (rf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream t(&rf);
        t << QString("exposure %1 ms\n").arg(R.exposureMs, 0, 'f', 2);
        t << QString("holes: %1 detected, %2 on the grid (%3 used), %4 expected but not detected\n")
                 .arg(R.holesDetected).arg(R.holesOnGrid).arg(R.holesUsed).arg(R.holesExpected);
        t << QString("grid-model fit (homography + radial): RMS %1 px\n\n").arg(R.gridRms, 0, 'f', 2);
        t << "CALIBRATION (one view, principal point fixed at the centre)\n";
        t << QString("  reprojection RMS %1 px   (max %2 px)\n").arg(R.rms, 0, 'f', 3).arg(R.maxError, 0, 'f', 2);
        t << QString("  focal length %1 px = %2 mm\n").arg(R.K(0, 0), 0, 'f', 1).arg(R.focalMM, 0, 'f', 3);
        t << QString("  distortion k1 %1  k2 %2   -> moves the image corner %3 px\n")
                 .arg(R.dist[0], 0, 'f', 4).arg(R.dist[1], 0, 'f', 4).arg(R.cornerShiftPx, 0, 'f', 0);
        t << QString("  camera %1 mm above the board, optical axis tilted %2 deg from vertical\n\n")
                 .arg(R.heightMM, 0, 'f', 0).arg(R.tiltDeg, 0, 'f', 1);
        t << QString("lines into the distance (lens distortion removed): %1 columns, holes off their line RMS %2 px "
                     "(max %3); they meet %4 px above the top of the picture\n")
                 .arg(R.columns).arg(R.straightRms, 0, 'f', 2).arg(R.straightMax, 0, 'f', 2).arg(R.vanishingAbovePx, 0, 'f', 0);
    }

    if (!R.raw.empty()) {
        QImage raw(R.raw.cols, R.raw.rows, QImage::Format_Grayscale16);
        for (int r = 0; r < R.raw.rows; r++) {
            memcpy(raw.scanLine(r), R.raw.ptr(r), R.raw.cols * sizeof(quint16));
        }
        raw.save(dir.filePath("frame_raw10.png"));
    }
    R.holes.save(dir.filePath("holes.png"));
    R.undistorted.save(dir.filePath("undistorted.png"));
    R.lines.save(dir.filePath("vanishing_lines.png"));
    R.vanishing.save(dir.filePath("vanishing_point.png"));
    return true;
}
