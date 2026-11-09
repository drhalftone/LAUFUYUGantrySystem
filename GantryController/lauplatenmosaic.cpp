/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauplatenmosaic.h"
#include "lauholegrid.h"

#include <set>
#include <cmath>
#include <vector>
#include <algorithm>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QStringList>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace
{
    const double PITCH = LAUHoleGrid::PITCH;
    const double EDGE = 12.7;                    // board edge to the first hole, mm
    const double MM = 0.25;                      // mosaic resolution, mm per pixel

    // hole diameters (px) the detector's defaults were tuned for, at the focal length (px) they were tuned at
    const double HOLE_D_MIN = 10.0, HOLE_D_MAX = 70.0, HOLE_D_F = 6.0 / 3.45e-3;

    // the camera's orientation from its bracket: only says which way the board is turned in the image
    const double CAD_TILT_DEG = 25.07;

    // a lattice position is a real hole if its centre is this much darker than the board around it
    const double HOLE_R_MM = 2.5, RING_IN_MM = 6.0, RING_OUT_MM = 9.0, MAX_HOLE_RATIO = 0.75;
    const double MIN_LINE_FILL = 0.3;            // a row or column is real if this much of it is holes (the rails hide some)
    const double DRAWN_HOLE_R_MM = 3.19;

    // the i1 aperture, as a pinhole ray of the camera: the plate is on the platen and the bracket fixes
    // this direction, so it holds for any lens.  Only a starting point for the measurement.
    const double APERTURE_RAY_X = 0.01344, APERTURE_RAY_Y = -0.01792;
    const double APERTURE_RADIUS_MM = 4.9, APERTURE_FLOOR_MM = 1.0;   // cad/xrite_i1_plate.py
    const double DISC_WINDOW_MM = 10.0, DISC_RES_MM = 0.02;

    const int CALIB_FLAGS = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO
                            | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3;

    struct Frame
    {
        QString file;
        double x = 0.0, y = 0.0;                 // carriage_mm: [rail pair, single rail]
        cv::Mat gray;
    };

    struct Camera
    {
        cv::Matx33d K = cv::Matx33d::eye();
        double dist[5] = { 0, 0, 0, 0, 0 };
        cv::Matx33d R = cv::Matx33d::eye();
        cv::Vec3d C0, ex = cv::Vec3d(1, 0, 0), ey = cv::Vec3d(0, 1, 0);

        cv::Vec3d centre(double x, double y) const
        {
            return C0 + ex * x + ey * y;
        }

        cv::Point2d ray(double xn, double yn) const
        {
            const double r2 = xn * xn + yn * yn;
            const double d = 1.0 + dist[0] * r2 + dist[1] * r2 * r2;
            return cv::Point2d(K(0, 0) * xn * d + K(0, 2), K(1, 1) * yn * d + K(1, 2));
        }

        std::vector<cv::Point2d> toBoard(const std::vector<cv::Point2d> &uv, double x, double y) const
        {
            std::vector<cv::Point2d> rays, out(uv.size());
            cv::undistortPoints(uv, rays, cv::Mat(K), cv::Mat(1, 5, CV_64F, const_cast<double *>(dist)));
            const cv::Vec3d C = centre(x, y);
            const cv::Matx33d Rt = R.t();
            for (size_t n = 0; n < uv.size(); n++) {
                const cv::Vec3d d = Rt * cv::Vec3d(rays[n].x, rays[n].y, 1.0);
                const double s = -C[2] / d[2];
                out[n] = cv::Point2d(C[0] + s * d[0], C[1] + s * d[1]);
            }
            return out;
        }
    };

    struct Indexed
    {
        std::vector<cv::Point> IJ;
        std::vector<cv::Point2d> uv;
    };

    double medianOf(std::vector<double> v)
    {
        if (v.empty()) {
            return 0.0;
        }
        const size_t k = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + k, v.end());
        double m = v[k];
        if (v.size() % 2 == 0) {
            m = 0.5 * (m + *std::max_element(v.begin(), v.begin() + k));
        }
        return m;
    }

    double imageMedian(const cv::Mat &floatImage)
    {
        std::vector<double> v;
        v.reserve(floatImage.total() / 16 + 1);
        for (int r = 0; r < floatImage.rows; r += 4) {
            const float *p = floatImage.ptr<float>(r);
            for (int c = 0; c < floatImage.cols; c += 4) {
                v.push_back(p[c]);
            }
        }
        return medianOf(v);
    }

    cv::Mat readGray(const QString &path)
    {
        QImage image(path);
        if (image.isNull()) {
            return cv::Mat();
        }
        image = image.convertToFormat(QImage::Format_Grayscale8);
        return cv::Mat(image.height(), image.width(), CV_8U, (void *)image.constBits(), image.bytesPerLine()).clone();
    }

    bool writeGray(const cv::Mat &gray, const QString &path)
    {
        return QImage(gray.data, gray.cols, gray.rows, (int)gray.step, QImage::Format_Grayscale8).save(path);
    }

    bool writeColor(const cv::Mat &bgr, const QString &path)
    {
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888).save(path);
    }

    bool writeText(const QString &path, const QString &text)
    {
        QFile file(path);
        return file.open(QIODevice::WriteOnly | QIODevice::Truncate) && file.write(text.toUtf8()) >= 0;
    }

    template <int M, int N> QJsonArray toJson(const cv::Matx<double, M, N> &m)
    {
        QJsonArray a;
        for (int n = 0; n < M * N; n++) {
            a.append(m.val[n]);
        }
        return a;
    }

    template <int M, int N> cv::Matx<double, M, N> fromJson(const QJsonArray &a)
    {
        cv::Matx<double, M, N> m;
        for (int n = 0; n < M * N && n < a.count(); n++) {
            m.val[n] = a.at(n).toDouble();
        }
        return m;
    }

    // ------------------------------------------------------------------------------ the camera from the holes
    const int SYMMETRIES[8][4] = { { 1, 0, 0, 1 }, { 0, -1, 1, 0 }, { -1, 0, 0, -1 }, { 0, 1, -1, 0 },
                                   { 1, 0, 0, -1 }, { -1, 0, 0, 1 }, { 0, 1, 1, 0 }, { 0, -1, -1, 0 } };

    Indexed indexLattice(const LAUHoleGrid::Lattice &lat, const Camera &cam, double x, double y)
    {
        const std::vector<cv::Point2d> prior = cam.toBoard(lat.uv, x, y);
        const int n = lat.count();
        std::vector<cv::Point2d> f(n);
        for (int k = 0; k < n; k++) {
            f[k] = cv::Point2d((prior[k].x - EDGE) / PITCH, (prior[k].y - EDGE) / PITCH);
        }

        double bestAgree = -1.0, bestResid = 0.0;
        std::vector<cv::Point> bestIJ;
        for (const int *D : SYMMETRIES) {
            std::vector<cv::Point> IJ0(n);
            std::vector<double> dx(n), dy(n);
            for (int k = 0; k < n; k++) {
                IJ0[k] = cv::Point(D[0] * lat.ij[k].x + D[1] * lat.ij[k].y, D[2] * lat.ij[k].x + D[3] * lat.ij[k].y);
                dx[k] = f[k].x - IJ0[k].x;
                dy[k] = f[k].y - IJ0[k].y;
            }
            const cv::Point off((int)std::lround(medianOf(dx)), (int)std::lround(medianOf(dy)));
            int agree = 0;
            double resid = 0.0;
            for (int k = 0; k < n; k++) {
                IJ0[k] += off;
                const double ax = std::abs(f[k].x - IJ0[k].x), ay = std::abs(f[k].y - IJ0[k].y);
                agree += (ax < 0.5 && ay < 0.5);
                resid += ax + ay;
            }
            const double fraction = agree / (double)n;
            if (fraction > bestAgree || (fraction == bestAgree && resid < bestResid)) {
                bestAgree = fraction;
                bestResid = resid;
                bestIJ = IJ0;
            }
        }
        return { bestIJ, lat.uv };
    }

    // the nominal camera, from the scan itself: the frame with the most holes calibrates it against its own lattice
    Camera bootstrap(const std::vector<Frame> &frames, const std::vector<LAUHoleGrid::Lattice> &lats, const cv::Matx33d &K0,
                     const cv::Size &size, int signX, int signY, std::vector<Indexed> &indexed)
    {
        int r = 0;
        for (int k = 0; k < (int)lats.size(); k++) {
            if (lats[k].count() > lats[r].count()) {
                r = k;
            }
        }
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
        for (int n = 0; n < lats[r].count(); n++) {
            obj.emplace_back((float)(PITCH * lats[r].ij[n].x), (float)(PITCH * lats[r].ij[n].y), 0.0f);
            img.emplace_back((float)lats[r].uv[n].x, (float)lats[r].uv[n].y);
        }
        cv::Mat K(K0), dist = cv::Mat::zeros(1, 5, CV_64F);
        K = K.clone();
        std::vector<cv::Mat> rv, tv;
        cv::calibrateCamera(std::vector<std::vector<cv::Point3f>>{ obj }, std::vector<std::vector<cv::Point2f>>{ img }, size, K, dist, rv, tv, CALIB_FLAGS);
        cv::Mat Rm;
        cv::Rodrigues(rv[0], Rm);
        const cv::Matx33d RL((double *)Rm.ptr());
        const cv::Vec3d CL = -(RL.t() * cv::Vec3d(tv[0].at<double>(0), tv[0].at<double>(1), tv[0].at<double>(2)));

        const double s = std::sin(CAD_TILT_DEG * CV_PI / 180.0), c = std::cos(CAD_TILT_DEG * CV_PI / 180.0);
        const cv::Matx33d RCad(0, 1, 0, c, 0, -s, -s, 0, -c);

        Camera cam;
        cam.K = cv::Matx33d((double *)K.ptr());
        for (int n = 0; n < 5; n++) {
            cam.dist[n] = dist.at<double>(n);
        }
        cam.ex = cv::Vec3d(signX, 0, 0);
        cam.ey = cv::Vec3d(0, signY, 0);

        double bestAngle = 1e9;
        for (const int *D : SYMMETRIES) {
            const int det = D[0] * D[3] - D[1] * D[2];
            if (det * CL[2] <= 0.0) {
                continue;
            }
            const cv::Matx33d A(D[0], D[1], 0, D[2], D[3], 0, 0, 0, det);
            const cv::Matx33d Rb = RL * A.t();
            cv::Vec3d angle;
            cv::Rodrigues(cv::Mat(Rb * RCad.t()), angle);
            const double off = cv::norm(angle);
            if (off < bestAngle) {
                bestAngle = off;
                cam.R = Rb;
                cam.C0 = A * CL + cv::Vec3d(EDGE, EDGE, 0) - cam.ex * frames[r].x - cam.ey * frames[r].y;
            }
        }

        // index every frame, then make the lowest hole seen the origin (the real board is found later, on the mosaic)
        indexed.clear();
        cv::Point lo(INT_MAX, INT_MAX);
        for (size_t k = 0; k < lats.size(); k++) {
            indexed.push_back(indexLattice(lats[k], cam, frames[k].x, frames[k].y));
            for (const cv::Point &p : indexed.back().IJ) {
                lo.x = std::min(lo.x, p.x);
                lo.y = std::min(lo.y, p.y);
            }
        }
        for (Indexed &i : indexed) {
            for (cv::Point &p : i.IJ) {
                p -= lo;
            }
        }
        cam.C0 -= cv::Vec3d(PITCH * lo.x, PITCH * lo.y, 0);
        return cam;
    }

    // all the frames together: the camera, every frame's pose, then the carriage's straight-line motion
    bool selfCalibrate(const std::vector<Frame> &frames, const std::vector<Indexed> &holes, const cv::Size &size,
                       Camera &cam, double *rmsOut, int *usedOut)
    {
        std::vector<std::vector<cv::Point3f>> obj(holes.size());
        std::vector<std::vector<cv::Point2f>> img(holes.size());
        for (size_t k = 0; k < holes.size(); k++) {
            for (size_t n = 0; n < holes[k].IJ.size(); n++) {
                obj[k].emplace_back((float)(EDGE + PITCH * holes[k].IJ[n].x), (float)(EDGE + PITCH * holes[k].IJ[n].y), 0.0f);
                img[k].emplace_back((float)holes[k].uv[n].x, (float)holes[k].uv[n].y);
            }
        }
        cv::Mat K = cv::Mat(cam.K).clone(), dist = cv::Mat::zeros(1, 5, CV_64F);
        for (int n = 0; n < 5; n++) {
            dist.at<double>(n) = cam.dist[n];
        }
        std::vector<cv::Mat> rv, tv;
        double rms = cv::calibrateCamera(obj, img, size, K, dist, rv, tv, CALIB_FLAGS);

        // drop gross outliers (features on the rails that fell on the lattice) and refit once
        for (size_t k = 0; k < obj.size(); k++) {
            std::vector<cv::Point2f> proj;
            cv::projectPoints(obj[k], rv[k], tv[k], K, dist, proj);
            std::vector<cv::Point3f> o;
            std::vector<cv::Point2f> i;
            for (size_t n = 0; n < obj[k].size(); n++) {
                if (std::hypot(proj[n].x - img[k][n].x, proj[n].y - img[k][n].y) < std::max(3.0 * rms, 1.0)) {
                    o.push_back(obj[k][n]);
                    i.push_back(img[k][n]);
                }
            }
            obj[k] = o;
            img[k] = i;
        }
        for (size_t k = 0; k < obj.size(); k++) {
            if (obj[k].size() < 6) {
                return false;
            }
        }
        rms = cv::calibrateCamera(obj, img, size, K, dist, rv, tv, CALIB_FLAGS);
        *rmsOut = rms;
        *usedOut = 0;
        for (const auto &o : obj) {
            *usedOut += (int)o.size();
        }

        cam.K = cv::Matx33d((double *)K.ptr());
        for (int n = 0; n < 5; n++) {
            cam.dist[n] = dist.at<double>(n);
        }

        // camera centre = C0 + x ex + y ey, least squares over the frames; one mean rotation
        const int N = (int)frames.size();
        cv::Mat A(N, 3, CV_64F), C(N, 3, CV_64F);
        cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
        for (int k = 0; k < N; k++) {
            cv::Mat Rk;
            cv::Rodrigues(rv[k], Rk);
            cv::Mat c = -Rk.t() * tv[k];
            A.at<double>(k, 0) = 1.0;
            A.at<double>(k, 1) = frames[k].x;
            A.at<double>(k, 2) = frames[k].y;
            for (int n = 0; n < 3; n++) {
                C.at<double>(k, n) = c.at<double>(n);
            }
            sum += Rk;
        }
        cv::Mat sol;
        cv::solve(A, C, sol, cv::DECOMP_SVD);
        cv::Mat w, U, Vt;
        cv::SVD::compute(sum, w, U, Vt);
        cv::Mat Rm = U * Vt;
        cam.R = cv::Matx33d((double *)Rm.ptr());
        for (int n = 0; n < 3; n++) {
            cam.C0[n] = sol.at<double>(0, n);
            cam.ex[n] = sol.at<double>(1, n);
            cam.ey[n] = sol.at<double>(2, n);
        }
        return true;
    }

    // ---------------------------------------------------------------------------- mask, flat-field, mosaic
    // The i1 cradle, outlined as pinhole rays of the camera: it is fixed to the camera's bracket, so these hold
    // for any lens.  The window in the cradle is included, since the board seen through it isn't usable.
    const double CRADLE_RAYS[][2] = { { -0.0596, -0.0224 }, { -0.0373, -0.0671 }, { 0.0149, -0.0850 }, { 0.0671, -0.0671 }, { 0.0895, -0.0224 }, { 0.0745, 0.0224 }, { 0.1483, 0.2602 }, { 0.1643, 0.4649 }, { -0.1269, 0.4641 }, { -0.1148, 0.2598 }, { -0.0417, 0.0223 } };

    cv::Mat carriageMask(const Camera &cam, const cv::Size &size, double *fractionOut)
    {
        std::vector<cv::Point> outline;
        for (const auto &ray : CRADLE_RAYS) {
            const cv::Point2d p = cam.ray(ray[0], ray[1]);
            outline.emplace_back((int)std::lround(p.x), (int)std::lround(p.y));
        }
        cv::Mat mask = cv::Mat::zeros(size, CV_8U);
        cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{ outline }, cv::Scalar(255));
        cv::dilate(mask, mask, cv::Mat::ones(21, 21, CV_8U));
        *fractionOut = cv::countNonZero(mask) / (double)mask.total();
        return mask;
    }

    // illumination x vignetting x board finish, with the holes closed over; median across the scan, 1 at its median
    cv::Mat flatField(const std::vector<Frame> &frames)
    {
        const int N = (int)frames.size();
        std::vector<cv::Mat> bg(N);
        const cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(99, 99));
        cv::parallel_for_(cv::Range(0, N), [&](const cv::Range &range) {
            for (int k = range.start; k < range.end; k++) {
                cv::Mat f;
                frames[k].gray.convertTo(f, CV_32F);
                cv::morphologyEx(f, f, cv::MORPH_CLOSE, element);
                cv::GaussianBlur(f, bg[k], cv::Size(0, 0), 16.0);
            }
        });
        cv::Mat ff(bg[0].size(), CV_32F);
        cv::parallel_for_(cv::Range(0, ff.rows), [&](const cv::Range &range) {
            std::vector<float> v(N);
            for (int r = range.start; r < range.end; r++) {
                float *out = ff.ptr<float>(r);
                for (int c = 0; c < ff.cols; c++) {
                    for (int k = 0; k < N; k++) {
                        v[k] = bg[k].at<float>(r, c);
                    }
                    std::nth_element(v.begin(), v.begin() + N / 2, v.end());
                    out[c] = v[N / 2];
                }
            }
        });
        ff /= std::max(imageMedian(ff), 1.0);
        return ff;
    }

    // every mosaic pixel looks up each frame through the camera; the closest view dominates, feathered at the edges
    void renderMosaic(const std::vector<Frame> &frames, const cv::Mat &ff, const Camera &cam, const cv::Mat &mask,
                      int nx, int ny, const std::atomic<bool> &cancel, cv::Mat &mosaic, cv::Mat &covered)
    {
        cv::Mat acc = cv::Mat::zeros(ny, nx, CV_32F), wsum = cv::Mat::zeros(ny, nx, CV_32F);
        const double fx = cam.K(0, 0), fy = cam.K(1, 1), cx = cam.K(0, 2), cy = cam.K(1, 2);
        const double k1 = cam.dist[0], k2 = cam.dist[1];
        for (const Frame &frame : frames) {
            if (cancel) {
                return;
            }
            cv::Mat prep;
            frame.gray.convertTo(prep, CV_32F);
            cv::divide(prep, cv::max(ff, 1e-3), prep);
            prep = cv::min(prep, 255.0);
            const int W = prep.cols, H = prep.rows;
            const cv::Vec3d C = cam.centre(frame.x, frame.y);
            cv::parallel_for_(cv::Range(0, ny), [&](const cv::Range &range) {
                for (int r = range.start; r < range.end; r++) {
                    float *a = acc.ptr<float>(r), *s = wsum.ptr<float>(r);
                    const double Y = (r + 0.5) * MM;
                    for (int c = 0; c < nx; c++) {
                        const cv::Vec3d p = cam.R * cv::Vec3d((c + 0.5) * MM - C[0], Y - C[1], -C[2]);
                        if (p[2] <= 1.0) {
                            continue;
                        }
                        const double xn = p[0] / p[2], yn = p[1] / p[2];
                        const double r2 = xn * xn + yn * yn;
                        if (r2 > 4.0) {
                            continue;
                        }
                        const double d = 1.0 + k1 * r2 + k2 * r2 * r2;
                        const double u = fx * xn * d + cx, v = fy * yn * d + cy;
                        if (u < 0.0 || v < 0.0 || u >= W - 1 || v >= H - 1) {
                            continue;
                        }
                        if (mask.at<uchar>((int)std::lround(v), (int)std::lround(u))) {
                            continue;
                        }
                        const int iu = (int)u, iv = (int)v;
                        const float du = (float)(u - iu), dv = (float)(v - iv);
                        const float *p0 = prep.ptr<float>(iv), *p1 = prep.ptr<float>(iv + 1);
                        const float val = (1 - dv) * ((1 - du) * p0[iu] + du * p0[iu + 1]) + dv * ((1 - du) * p1[iu] + du * p1[iu + 1]);
                        const double edge = std::min(std::min(u, v), std::min(W - u, H - v));
                        const double weight = std::min(edge / 60.0, 1.0) * std::pow(300.0 / p[2], 6.0);
                        a[c] += (float)(weight * val);
                        s[c] += (float)weight;
                    }
                }
            });
        }
        cv::Mat out;
        cv::divide(acc, cv::max(wsum, 1e-9f), out);
        out.convertTo(mosaic, CV_8U);
        covered = (wsum > 0);
    }

    double holePx(double mm)
    {
        return mm / MM - 0.5;
    }

    bool isHole(const cv::Mat &mosaic, const cv::Mat &covered, double xMM, double yMM)
    {
        const int reach = (int)(RING_OUT_MM / MM) + 1;
        const int x = (int)std::lround(holePx(xMM)), y = (int)std::lround(holePx(yMM));
        if (x - reach < 0 || y - reach < 0 || x + reach >= mosaic.cols || y + reach >= mosaic.rows || !covered.at<uchar>(y, x)) {
            return false;
        }
        double inner = 0.0;
        int innerCount = 0;
        std::vector<double> ring;
        for (int dy = -reach; dy <= reach; dy++) {
            for (int dx = -reach; dx <= reach; dx++) {
                if (!covered.at<uchar>(y + dy, x + dx)) {
                    continue;
                }
                const double d = std::hypot(dx, dy) * MM;
                if (d <= HOLE_R_MM) {
                    inner += mosaic.at<uchar>(y + dy, x + dx);
                    innerCount++;
                } else if (d >= RING_IN_MM && d <= RING_OUT_MM) {
                    ring.push_back(mosaic.at<uchar>(y + dy, x + dx));
                }
            }
        }
        return innerCount > 10 && ring.size() > 30 && (inner / innerCount) < MAX_HOLE_RATIO * medianOf(ring);
    }

    // ------------------------------------------------------------------------------------- the i1 aperture
    // The bright disc of board seen through the plate's hole, on a local ortho crop around guess: its centroid, board mm.
    bool findDisc(const cv::Mat &gray, const Camera &cam, double x, double y, const cv::Point2d &guess, cv::Point2d *centre)
    {
        const int n = (int)std::lround(2.0 * DISC_WINDOW_MM / DISC_RES_MM) + 1;
        cv::Mat mapU(n, n, CV_32F), mapV(n, n, CV_32F);
        const cv::Vec3d C = cam.centre(x, y);
        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                const cv::Vec3d p = cam.R * cv::Vec3d(guess.x - DISC_WINDOW_MM + c * DISC_RES_MM - C[0],
                                                      guess.y - DISC_WINDOW_MM + r * DISC_RES_MM - C[1], -C[2]);
                const cv::Point2d uv = cam.ray(p[0] / p[2], p[1] / p[2]);
                mapU.at<float>(r, c) = (float)uv.x;
                mapV.at<float>(r, c) = (float)uv.y;
            }
        }
        cv::Mat L;
        gray.convertTo(L, CV_32F);
        cv::remap(L, L, mapU, mapV, cv::INTER_LINEAR);

        cv::Mat soft, bw;
        cv::GaussianBlur(L, soft, cv::Size(0, 0), 2.0);
        soft.convertTo(soft, CV_8U);
        cv::threshold(soft, bw, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
        cv::Mat lab, stats, centroids;
        const int count = cv::connectedComponentsWithStats(bw, lab, stats, centroids);
        int best = -1;
        double bestDistance = 1e12;
        for (int k = 1; k < count; k++) {
            const int l = stats.at<int>(k, cv::CC_STAT_LEFT), t = stats.at<int>(k, cv::CC_STAT_TOP);
            const int w = stats.at<int>(k, cv::CC_STAT_WIDTH), h = stats.at<int>(k, cv::CC_STAT_HEIGHT);
            if (l > 0 && t > 0 && l + w < n && t + h < n && stats.at<int>(k, cv::CC_STAT_AREA) * DISC_RES_MM * DISC_RES_MM > 20.0) {
                const double d = std::hypot(centroids.at<double>(k, 0) - n / 2.0, centroids.at<double>(k, 1) - n / 2.0);
                if (d < bestDistance) {
                    bestDistance = d;
                    best = k;
                }
            }
        }
        if (best < 0) {
            return false;
        }
        const cv::Mat body = (lab == best);
        cv::Mat region;
        cv::dilate(body, region, cv::Mat::ones(15, 15, CV_8U));

        std::vector<double> outside, inside;
        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                if (!region.at<uchar>(r, c)) {
                    outside.push_back(L.at<float>(r, c));
                }
                if (body.at<uchar>(r, c)) {
                    inside.push_back(L.at<float>(r, c));
                }
            }
        }
        const double black = medianOf(outside);
        const size_t top = (size_t)(0.9 * (inside.size() - 1));
        std::nth_element(inside.begin(), inside.begin() + top, inside.end());
        const double white = std::max(inside[top], black + 1.0);

        double sw = 0.0, sx = 0.0, sy = 0.0;
        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                if (region.at<uchar>(r, c)) {
                    const double wgt = std::min(std::max((L.at<float>(r, c) - black) / (white - black), 0.0), 1.0);
                    sw += wgt;
                    sx += wgt * c;
                    sy += wgt * r;
                }
            }
        }
        *centre = cv::Point2d(guess.x - DISC_WINDOW_MM + sx / sw * DISC_RES_MM, guess.y - DISC_WINDOW_MM + sy / sw * DISC_RES_MM);
        return true;
    }

    bool measureAperture(const std::vector<Frame> &frames, const Camera &cam, cv::Point2d *A0, double *spread, int *used)
    {
        const double ex0 = cam.ex[0], ex1 = cam.ex[1], ey0 = cam.ey[0], ey1 = cam.ey[1];
        struct Seen { double x, y, ax, ay; };
        std::vector<Seen> seen;
        const cv::Point2d pixel = cam.ray(APERTURE_RAY_X, APERTURE_RAY_Y);
        for (const Frame &f : frames) {
            const cv::Point2d guess = cam.toBoard({ pixel }, f.x, f.y)[0];
            cv::Point2d visible;
            if (!findDisc(f.gray, cam, f.x, f.y, guess, &visible)) {
                continue;
            }
            // the plate's rim hides part of the disc: the visible part's centroid sits half the rim's shift off
            const cv::Vec3d C = cam.centre(f.x, f.y);
            cv::Point2d c = visible;
            for (int n = 0; n < 3; n++) {
                const double k = APERTURE_FLOOR_MM / (C[2] - APERTURE_FLOOR_MM);
                c = visible - 0.5 * k * (c - cv::Point2d(C[0], C[1]));
            }
            seen.push_back({ f.x, f.y, c.x - f.x * ex0 - f.y * ey0, c.y - f.x * ex1 - f.y * ey1 });
        }
        if (seen.empty()) {
            return false;
        }
        std::vector<double> ax, ay;
        for (const Seen &s : seen) {
            ax.push_back(s.ax);
            ay.push_back(s.ay);
        }
        cv::Point2d a(medianOf(ax), medianOf(ay));

        // only frames with plain board under the aperture: a dark hole in the disc would bias its centroid
        std::vector<Seen> keep;
        for (int round = 0; round < 3; round++) {
            keep.clear();
            for (const Seen &s : seen) {
                const double px = a.x + s.x * ex0 + s.y * ey0, py = a.y + s.x * ex1 + s.y * ey1;
                const double hx = std::round((px - EDGE) / PITCH) * PITCH + EDGE, hy = std::round((py - EDGE) / PITCH) * PITCH + EDGE;
                if (std::hypot(px - hx, py - hy) > APERTURE_RADIUS_MM + 3.2 + 1.0 && std::hypot(s.ax - a.x, s.ay - a.y) < 1.0) {
                    keep.push_back(s);
                }
            }
            if (keep.empty()) {
                return false;
            }
            cv::Point2d sum(0, 0);
            for (const Seen &s : keep) {
                sum += cv::Point2d(s.ax, s.ay);
            }
            a = sum * (1.0 / keep.size());
        }
        double total = 0.0;
        for (const Seen &s : keep) {
            total += (s.ax - a.x) * (s.ax - a.x) + (s.ay - a.y) * (s.ay - a.y);
        }
        *A0 = a;
        *spread = std::sqrt(total / keep.size());
        *used = (int)keep.size();
        return true;
    }
}

LAUPlatenMosaicResult LAUPlatenMosaic::process(const LAUPlatenMosaicRequest &req)
{
    using LAUHoleGrid::Detections;
    using LAUHoleGrid::Lattice;
    using LAUHoleGrid::detectHoles;
    using LAUHoleGrid::growLattice;
    LAUPlatenMosaicResult result;
    const auto fail = [&result](const QString &text) {
        result.ok = false;
        result.message = text;
        return result;
    };
    const QString cancelled = QString("Merging was cancelled.");

    // ------------------------------------------------------------------------------------- the scan
    QFile file(QDir(req.scanFolder).filePath(QString("scan.json")));
    if (!file.open(QIODevice::ReadOnly)) {
        return fail(QString("Couldn't read the scan in %1").arg(QDir::toNativeSeparators(req.scanFolder)));
    }
    const QJsonObject scan = QJsonDocument::fromJson(file.readAll()).object();
    const cv::Size size(scan["image_size"].toArray().at(0).toInt(), scan["image_size"].toArray().at(1).toInt());
    std::vector<Frame> all;
    emit emitProgress(QString("Loading the scan..."));
    for (const QJsonValue &value : scan["frames"].toArray()) {
        const QJsonObject o = value.toObject();
        Frame f;
        f.file = o["file"].toString();
        f.x = o["carriage_mm"].toArray().at(0).toDouble();
        f.y = o["carriage_mm"].toArray().at(1).toDouble();
        f.gray = readGray(QDir(req.scanFolder).filePath(f.file));
        if (f.gray.empty()) {
            return fail(QString("Couldn't read %1 in the scan.").arg(f.file));
        }
        all.push_back(f);
    }
    if (all.size() < 4) {
        return fail(QString("The scan has only %1 frames, too few to merge.").arg(all.size()));
    }

    QDir out(req.scanFolder);
    out.mkpath(QString("mosaic"));
    out.cd(QString("mosaic"));
    result.folder = out.absolutePath();

    Camera cam;
    cv::Mat mask, flat;
    int nx = 0, ny = 0;
    int I0 = 0, I1 = 0, J0 = 0, J1 = 0;
    cv::Point2d A0;
    bool apertureOk = false;
    QStringList report;
    const bool isBare = req.bareFolder.isEmpty();
    QJsonObject calibration;

    if (isBare) {
        // ----------------------------------------------------------- holes -> camera and carriage model
        const double pixelMM = scan["pixel_mm"].toDouble(5.86e-3);
        const double f0 = scan["focal_mm"].toDouble(6.0) / pixelMM;
        const cv::Matx33d K0(f0, 0, size.width / 2.0, 0, f0, size.height / 2.0, 0, 0, 1);
        const double minD = HOLE_D_MIN * f0 / HOLE_D_F, maxD = HOLE_D_MAX * f0 / HOLE_D_F;

        std::vector<Lattice> found(all.size());
        emit emitProgress(QString("Finding the holes in %1 frames...").arg(all.size()));
        cv::parallel_for_(cv::Range(0, (int)all.size()), [&](const cv::Range &range) {
            for (int k = range.start; k < range.end && !cancelFlag; k++) {
                const Detections det = detectHoles(all[k].gray, minD, maxD);
                found[k] = growLattice(det.centers, size, &det);
            }
        });
        if (cancelFlag) {
            return fail(cancelled);
        }
        std::vector<Frame> frames;
        std::vector<Lattice> lats;
        for (size_t k = 0; k < all.size(); k++) {
            if (found[k].valid && found[k].count() >= 12) {
                frames.push_back(all[k]);
                lats.push_back(found[k]);
            }
        }
        if (frames.size() < 4) {
            return fail(QString("Only %1 frames showed the hole grid, too few to calibrate. Check the lighting and that the platen is clear.").arg(frames.size()));
        }

        const QJsonObject ends = scan["left_end"].toObject();
        const int signX = ends["y_rail_pair"].toString() == QString("motor") ? 1 : -1;
        const int signY = ends["x_single_rail"].toString() == QString("motor") ? 1 : -1;

        emit emitProgress(QString("Calibrating the camera and the carriage from the holes..."));
        std::vector<Indexed> holes;
        cam = bootstrap(frames, lats, K0, size, signX, signY, holes);
        double rms = 0.0;
        int used = 0;
        if (!selfCalibrate(frames, holes, size, cam, &rms, &used)) {
            return fail(QString("Some frames had too few usable holes to calibrate."));
        }
        report << QString("frames with a hole grid: %1 of %2").arg(frames.size()).arg(all.size());
        report << QString("self-calibration: %1 holes, RMS %2 px, focal %3 mm, k1 %4, k2 %5").arg(used).arg(rms, 0, 'f', 3)
                      .arg(cam.K(0, 0) * pixelMM, 0, 'f', 3).arg(cam.dist[0], 0, 'f', 4).arg(cam.dist[1], 0, 'f', 4);
        report << QString("C0 (%1, %2, %3) mm").arg(cam.C0[0], 0, 'f', 2).arg(cam.C0[1], 0, 'f', 2).arg(cam.C0[2], 0, 'f', 2);
        report << QString("ex (%1, %2, %3)  ey (%4, %5, %6)").arg(cam.ex[0], 0, 'f', 4).arg(cam.ex[1], 0, 'f', 4).arg(cam.ex[2], 0, 'f', 4)
                      .arg(cam.ey[0], 0, 'f', 4).arg(cam.ey[1], 0, 'f', 4).arg(cam.ey[2], 0, 'f', 4);

        emit emitProgress(QString("Finding the carriage and the lighting in the frames..."));
        double maskFraction = 0.0;
        mask = carriageMask(cam, size, &maskFraction);
        flat = flatField(frames);
        report << QString("carriage mask: %1 % of each frame").arg(100.0 * maskFraction, 0, 'f', 1);

        int maxI = 0, maxJ = 0;
        for (const Indexed &i : holes) {
            for (const cv::Point &p : i.IJ) {
                maxI = std::max(maxI, p.x);
                maxJ = std::max(maxJ, p.y);
            }
        }
        nx = (int)std::ceil((EDGE + PITCH * (maxI + 2)) / MM);
        ny = (int)std::ceil((EDGE + PITCH * (maxJ + 2)) / MM);

        emit emitProgress(QString("Measuring where the i1 aperture is..."));
        double spread = 0.0;
        int apertureFrames = 0;
        apertureOk = measureAperture(frames, cam, &A0, &spread, &apertureFrames);
        if (apertureOk) {
            report << QString("aperture at carriage (0,0): board (%1, %2) mm, spread RMS %3 mm over %4 frames")
                          .arg(A0.x, 0, 'f', 3).arg(A0.y, 0, 'f', 3).arg(spread, 0, 'f', 3).arg(apertureFrames);
        } else {
            report << QString("aperture: not measured (no frame had a clear view of it)");
        }
        all = frames;
    } else {
        // ------------------------------------------------------------------ the bare scan's calibration
        const QDir bare(QDir(req.bareFolder).filePath(QString("mosaic")));
        QFile cal(bare.filePath(QString("calibration.json")));
        if (!cal.open(QIODevice::ReadOnly)) {
            return fail(QString("There is no platen calibration in %1. Run Calibrate platen first.").arg(QDir::toNativeSeparators(req.bareFolder)));
        }
        const QJsonObject c = QJsonDocument::fromJson(cal.readAll()).object();
        cam.K = fromJson<3, 3>(c["K"].toArray());
        cam.R = fromJson<3, 3>(c["R"].toArray());
        const cv::Matx<double, 5, 1> d = fromJson<5, 1>(c["dist"].toArray());
        for (int n = 0; n < 5; n++) {
            cam.dist[n] = d.val[n];
        }
        cam.C0 = cv::Vec3d(fromJson<3, 1>(c["C0"].toArray()).val);
        cam.ex = cv::Vec3d(fromJson<3, 1>(c["ex"].toArray()).val);
        cam.ey = cv::Vec3d(fromJson<3, 1>(c["ey"].toArray()).val);
        nx = c["mosaic_nx"].toInt();
        ny = c["mosaic_ny"].toInt();
        I0 = c["hole_I0"].toInt();
        I1 = c["hole_I1"].toInt();
        J0 = c["hole_J0"].toInt();
        J1 = c["hole_J1"].toInt();
        apertureOk = c.contains(QString("A0_mm"));
        if (apertureOk) {
            A0 = cv::Point2d(c["A0_mm"].toArray().at(0).toDouble(), c["A0_mm"].toArray().at(1).toDouble());
        }
        mask = readGray(bare.filePath(QString("mask.png")));
        QImage flatImage(bare.filePath(QString("flat.png")));
        if (mask.empty() || flatImage.isNull() || mask.size() != size) {
            return fail(QString("The platen calibration in %1 is incomplete. Run Calibrate platen again.").arg(QDir::toNativeSeparators(req.bareFolder)));
        }
        flatImage = flatImage.convertToFormat(QImage::Format_Grayscale16);
        cv::Mat(flatImage.height(), flatImage.width(), CV_16U, (void *)flatImage.constBits(), flatImage.bytesPerLine()).convertTo(flat, CV_32F, 1.0 / 32768.0);
        report << QString("calibration from %1").arg(QDir::toNativeSeparators(req.bareFolder));
    }

    // ------------------------------------------------------------------------------------- the mosaic
    emit emitProgress(QString("Merging %1 frames into one picture of the platen...").arg(all.size()));
    cv::Mat mosaic, covered;
    renderMosaic(all, flat, cam, mask, nx, ny, cancelFlag, mosaic, covered);
    if (cancelFlag) {
        return fail(cancelled);
    }

    const int columns = (int)((std::max(nx, ny) * MM - EDGE) / PITCH) + 1;
    std::vector<std::vector<bool>> real(columns, std::vector<bool>(columns, false)), inView = real;
    if (isBare) {
        std::vector<int> foundI(columns, 0), viewI(columns, 0), foundJ(columns, 0), viewJ(columns, 0);
        for (int J = 0; J < columns; J++) {
            for (int I = 0; I < columns; I++) {
                const double bx = EDGE + PITCH * I, by = EDGE + PITCH * J;
                const int px = (int)holePx(bx), py = (int)holePx(by);
                const bool view = px >= 0 && py >= 0 && px < nx && py < ny && covered.at<uchar>(py, px);
                const bool hole = isHole(mosaic, covered, bx, by);
                viewI[I] += view; viewJ[J] += view;
                foundI[I] += hole; foundJ[J] += hole;
            }
        }
        I0 = J0 = columns;
        I1 = J1 = -1;
        for (int n = 0; n < columns; n++) {
            if (foundI[n] >= MIN_LINE_FILL * std::max(viewI[n], 1) && foundI[n] > 0) {
                I0 = std::min(I0, n);
                I1 = std::max(I1, n);
            }
            if (foundJ[n] >= MIN_LINE_FILL * std::max(viewJ[n], 1) && foundJ[n] > 0) {
                J0 = std::min(J0, n);
                J1 = std::max(J1, n);
            }
        }
        if (I1 < I0 || J1 < J0) {
            return fail(QString("Couldn't find the platen's holes in the merged picture."));
        }
    }
    int confirmed = 0, total = 0;
    for (int J = J0; J <= J1 && J < columns; J++) {
        for (int I = I0; I <= I1 && I < columns; I++) {
            const double bx = EDGE + PITCH * I, by = EDGE + PITCH * J;
            const int px = (int)holePx(bx), py = (int)holePx(by);
            inView[J][I] = px >= 0 && py >= 0 && px < nx && py < ny && covered.at<uchar>(py, px);
            real[J][I] = isHole(mosaic, covered, bx, by);
            confirmed += real[J][I];
            total++;
        }
    }

    // ---- the gantry's axes.  Mosaic columns are board X (the rail pair), rows board Y (the single rail): swap them,
    // then flip whichever axis runs against its slider.
    const bool flipX = cam.ey[1] < 0.0, flipY = cam.ex[0] < 0.0;
    cv::Mat turned;
    cv::transpose(mosaic, turned);
    if (flipX) {
        cv::flip(turned, turned, 1);
    }
    if (flipY) {
        cv::flip(turned, turned, 0);
    }
    const int nX = J1 - J0 + 1, nY = I1 - I0 + 1;
    const auto latticeOf = [&](int X, int Y) {
        return cv::Point(flipY ? I1 - Y : I0 + Y, flipX ? J1 - X : J0 + X);
    };
    const auto pictureOf = [&](int X, int Y) {
        const cv::Point L = latticeOf(X, Y);
        const double bx = holePx(EDGE + PITCH * L.x), by = holePx(EDGE + PITCH * L.y);
        return cv::Point2d(flipX ? ny - 1 - by : by, flipY ? nx - 1 - bx : bx);
    };
    const auto rounded = [](const cv::Point2d &p) {
        return cv::Point((int)std::lround(p.x), (int)std::lround(p.y));
    };

    cv::Mat image;
    cv::cvtColor(turned, image, cv::COLOR_GRAY2BGR);
    const cv::Scalar green(0, 200, 0), yellow(0, 255, 255), red(0, 0, 255), blue(255, 120, 0), black(0, 0, 0);
    for (int X = 0; X < nX; X++) {
        cv::line(image, rounded(pictureOf(X, 0)), rounded(pictureOf(X, nY - 1)), green, 1, cv::LINE_AA);
    }
    for (int Y = 0; Y < nY; Y++) {
        cv::line(image, rounded(pictureOf(0, Y)), rounded(pictureOf(nX - 1, Y)), green, 1, cv::LINE_AA);
    }
    QStringList inches{ QString("x_in,y_in,board_x_mm,board_y_mm,confirmed_dark") };
    QStringList sliders{ QString("x_in,y_in,slider_x_mm,slider_y_mm,reachable") };
    const double a = cam.ex[0], b = cam.ey[0], c = cam.ex[1], d = cam.ey[1], det = a * d - b * c;
    int reachable = 0;
    for (int Y = 0; Y < nY; Y++) {
        for (int X = 0; X < nX; X++) {
            const cv::Point L = latticeOf(X, Y);
            const double bx = EDGE + PITCH * L.x, by = EDGE + PITCH * L.y;
            const bool hole = L.x < columns && L.y < columns && real[L.y][L.x];
            inches << QString("%1,%2,%3,%4,%5").arg(X).arg(Y).arg(bx, 0, 'f', 2).arg(by, 0, 'f', 2).arg(hole ? 1 : 0);
            if (apertureOk) {
                const double px = bx - A0.x, py = by - A0.y;
                const double pair = (d * px - b * py) / det, single = (a * py - c * px) / det;
                const bool ok = single >= req.sliderXLow && single <= req.sliderXHigh && pair >= req.sliderYLow && pair <= req.sliderYHigh;
                reachable += ok;
                sliders << QString("%1,%2,%3,%4,%5").arg(X).arg(Y).arg(single, 0, 'f', 3).arg(pair, 0, 'f', 3).arg(ok ? 1 : 0);
            }
            if (L.x < columns && L.y < columns && inView[L.y][L.x]) {
                const cv::Point at = rounded(pictureOf(X, Y));
                // with a target down most holes are covered, so red only means something on the bare platen
                cv::circle(image, at, (int)std::lround(DRAWN_HOLE_R_MM / MM), (hole || !isBare) ? yellow : red, 1, cv::LINE_AA);
                const std::string label = QString("%1,%2").arg(X).arg(Y).toStdString();
                const cv::Point textAt(at.x + 7, at.y - 7);
                cv::putText(image, label, textAt, cv::FONT_HERSHEY_SIMPLEX, 0.4, black, 3, cv::LINE_AA);
                cv::putText(image, label, textAt, cv::FONT_HERSHEY_SIMPLEX, 0.4, yellow, 1, cv::LINE_AA);
            }
        }
    }
    const cv::Point origin = rounded(pictureOf(0, 0));
    cv::arrowedLine(image, origin, origin + cv::Point(200, 0), red, 3, cv::LINE_AA, 0, 0.1);
    cv::arrowedLine(image, origin, origin + cv::Point(0, 200), blue, 3, cv::LINE_AA, 0, 0.1);
    cv::putText(image, "X (in)", origin + cv::Point(205, 8), cv::FONT_HERSHEY_SIMPLEX, 0.8, red, 2, cv::LINE_AA);
    cv::putText(image, "Y (in)", origin + cv::Point(-40, 228), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2, cv::LINE_AA);

    // crop to the board: one inch around the holes
    const cv::Point far = rounded(pictureOf(nX - 1, nY - 1));
    const int margin = (int)std::lround(PITCH / MM);
    const int x0 = std::max(std::min(origin.x, far.x) - margin, 0), y0 = std::max(std::min(origin.y, far.y) - margin, 0);
    const int x1 = std::min(std::max(origin.x, far.x) + margin, image.cols), y1 = std::min(std::max(origin.y, far.y) + margin, image.rows);
    const cv::Rect crop(x0, y0, x1 - x0, y1 - y0);

    result.gridImage = out.filePath(QString("platen_mosaic_grid.png"));
    bool saved = writeGray(turned(crop), out.filePath(QString("platen_mosaic.png")));
    saved = writeColor(image(crop), result.gridImage) && saved;
    saved = writeText(out.filePath(QString("holes_inches.csv")), inches.join(QChar('\n')) + QChar('\n')) && saved;
    if (apertureOk) {
        saved = writeText(out.filePath(QString("hole_sliders.csv")), sliders.join(QChar('\n')) + QChar('\n')) && saved;
    }

    if (isBare) {
        calibration["K"] = toJson(cam.K);
        calibration["R"] = toJson(cam.R);
        QJsonArray dist;
        for (double v : cam.dist) {
            dist.append(v);
        }
        calibration["dist"] = dist;
        calibration["C0"] = QJsonArray{ cam.C0[0], cam.C0[1], cam.C0[2] };
        calibration["ex"] = QJsonArray{ cam.ex[0], cam.ex[1], cam.ex[2] };
        calibration["ey"] = QJsonArray{ cam.ey[0], cam.ey[1], cam.ey[2] };
        calibration["mosaic_nx"] = nx;
        calibration["mosaic_ny"] = ny;
        calibration["hole_I0"] = I0;
        calibration["hole_I1"] = I1;
        calibration["hole_J0"] = J0;
        calibration["hole_J1"] = J1;
        if (apertureOk) {
            calibration["A0_mm"] = QJsonArray{ A0.x, A0.y };
        }
        QFile cal(out.filePath(QString("calibration.json")));
        saved = cal.open(QIODevice::WriteOnly | QIODevice::Truncate) && cal.write(QJsonDocument(calibration).toJson()) > 0 && saved;
        saved = writeGray(mask, out.filePath(QString("mask.png"))) && saved;
        cv::Mat flat16;
        flat.convertTo(flat16, CV_16U, 32768.0);
        saved = QImage(flat16.data, flat16.cols, flat16.rows, (int)flat16.step, QImage::Format_Grayscale16).save(out.filePath(QString("flat.png"))) && saved;
    }

    report << QString("holes: %1 x %2 (X 0..%3 in, Y 0..%4 in), lattice columns %5..%6, rows %7..%8; %9 of %10 confirmed dark")
                  .arg(nX).arg(nY).arg(nX - 1).arg(nY - 1).arg(I0).arg(I1).arg(J0).arg(J1).arg(confirmed).arg(total);
    if (apertureOk) {
        report << QString("%1 of %2 holes reachable by the i1 aperture").arg(reachable).arg(nX * nY);
    }
    saved = writeText(out.filePath(QString("report.txt")), report.join(QChar('\n')) + QChar('\n')) && saved;
    if (!saved) {
        return fail(QString("Couldn't save the results in %1").arg(QDir::toNativeSeparators(result.folder)));
    }

    result.ok = true;
    result.message = report.join(QChar('\n'));
    return result;
}
