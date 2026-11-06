/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
#include "lauholegrid.h"

#include <map>
#include <deque>
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace LAUHoleGrid;

namespace
{
    int odd(double n)
    {
        const int k = (int)std::lround(n);
        return k + 1 - (((k % 2) + 2) % 2);
    }

    cv::Mat disc(int d)
    {
        return cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(d, d));
    }

    double norm2(const cv::Point2d &p)
    {
        return std::sqrt(p.x * p.x + p.y * p.y);
    }

    int nearest(const std::vector<cv::Point2d> &pts, const cv::Point2d &q, double *dist)
    {
        int best = -1;
        double bd = std::numeric_limits<double>::max();
        for (int n = 0; n < (int)pts.size(); n++) {
            const double dx = pts[n].x - q.x, dy = pts[n].y - q.y;
            const double d = dx * dx + dy * dy;
            if (d < bd) {
                bd = d;
                best = n;
            }
        }
        if (dist) {
            *dist = std::sqrt(bd);
        }
        return best;
    }

    double median(std::vector<double> v)
    {
        if (v.empty()) {
            return 0.0;
        }
        const size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + m, v.end());
        double hi = v[m];
        if (v.size() % 2 == 0) {
            const double lo = *std::max_element(v.begin(), v.begin() + m);
            return 0.5 * (lo + hi);
        }
        return hi;
    }

    // ------------------------------------------------------------------------------------------ refine
    // Sub-pixel ellipse: where the ratio image crosses `thresh` along rays from the centre, searched from
    // the OUTSIDE in so bright thread highlights inside a tapped hole don't pull the edge inward.
    bool refine(const cv::Mat &ratio, cv::Point2d c, cv::Size2d axes, double thresh, cv::RotatedRect &out, int nRays = 48)
    {
        const double amax = std::max(axes.width, axes.height);
        const double rmax = amax * 0.9 + 3.0;
        const int nR = std::max(2, (int)(rmax * 4.0));
        std::vector<double> rs(nR);
        for (int i = 0; i < nR; i++) {
            rs[i] = rmax * i / (nR - 1);
        }
        cv::Mat xs(nRays, nR, CV_32F), ys(nRays, nR, CV_32F);
        std::vector<double> th(nRays);
        for (int k = 0; k < nRays; k++) {
            th[k] = 2.0 * CV_PI * k / nRays;
            const double ct = std::cos(th[k]), st = std::sin(th[k]);
            for (int i = 0; i < nR; i++) {
                xs.at<float>(k, i) = (float)(c.x + ct * rs[i]);
                ys.at<float>(k, i) = (float)(c.y + st * rs[i]);
            }
        }
        cv::Mat prof;
        cv::remap(ratio, prof, xs, ys, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

        std::vector<cv::Point2f> pts;
        for (int k = 0; k < nRays; k++) {
            const float *p = prof.ptr<float>(k);
            // "not board" = dark, or a highlight inside the hole: fold highlights onto the dark side
            int last = -1;
            for (int i = 0; i < nR; i++) {
                const double q = (p[i] > BRIGHT) ? 0.0 : p[i];
                if (q < thresh) {
                    last = i;
                }
            }
            if (last < 0 || last >= nR - 1) {
                continue;                                  // never dark, or still dark at rmax
            }
            const int i = last + 1;                        // first board sample after the outermost hole one
            double r;
            if (p[i - 1] > BRIGHT) {
                r = rs[i - 1] + 0.5 * (rs[i] - rs[i - 1]); // highlight -> board: no gradient to interpolate
            } else {
                const double f = (thresh - p[i - 1]) / (p[i] - p[i - 1]);
                r = rs[i - 1] + f * (rs[i] - rs[i - 1]);
            }
            pts.emplace_back((float)(c.x + r * std::cos(th[k])), (float)(c.y + r * std::sin(th[k])));
        }
        if ((int)pts.size() < nRays * 0.75) {
            return false;
        }
        out = cv::fitEllipse(pts);
        const double d = std::hypot(out.center.x - c.x, out.center.y - c.y);
        return d < amax * 0.25;
    }

    // ------------------------------------------------------------------------------------- robust fit
    double pitchPx(const std::vector<cv::Point2d> &uv, const std::vector<cv::Point2d> &ij)
    {
        std::vector<double> d;
        for (size_t a = 0; a < ij.size(); a++) {
            double bd = std::numeric_limits<double>::max();
            size_t bk = a;
            for (size_t b = 0; b < ij.size(); b++) {
                if (b == a) {
                    continue;
                }
                const double e = norm2(ij[a] - ij[b]);
                if (e < bd) {
                    bd = e;
                    bk = b;
                }
            }
            if (bk != a && std::abs(bd - 1.0) < 1e-9) {
                d.push_back(norm2(uv[a] - uv[bk]));
            }
        }
        return d.empty() ? 50.0 : median(d);
    }

    GridModel build(const std::vector<double> &x, bool fitK, double k1, double k2, cv::Point2d centre, double f)
    {
        GridModel m;
        m.H = cv::Matx33d(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], 1.0);
        m.k1 = fitK ? x[8] : k1;
        m.k2 = fitK ? x[9] : k2;
        m.centre = centre;
        m.f = f;
        return m;
    }

    void residuals(const GridModel &m, const std::vector<cv::Point2d> &ij, const std::vector<cv::Point2d> &uv, std::vector<double> &r)
    {
        r.resize(2 * ij.size());
        for (size_t n = 0; n < ij.size(); n++) {
            const cv::Point2d p = m(ij[n]);
            r[2 * n] = p.x - uv[n].x;
            r[2 * n + 1] = p.y - uv[n].y;
        }
    }

    double robustCost(const std::vector<double> &r)
    {
        // soft-L1 with f_scale = 1 (scipy's loss="soft_l1"): rho(z) = 2 (sqrt(1 + z) - 1), z = r^2
        double c = 0.0;
        for (double v : r) {
            c += 2.0 * (std::sqrt(1.0 + v * v) - 1.0);
        }
        return c;
    }

    // Robust least squares of GridModel to labelled holes (Levenberg-Marquardt on the soft-L1 cost,
    // Marquardt diagonal scaling -- the counterpart of scipy's least_squares(loss="soft_l1", x_scale="jac")).
    bool fitGridModel(const std::vector<cv::Point2d> &ij, const std::vector<cv::Point2d> &uv, cv::Point2d centre, double f,
                      const cv::Matx33d *H0, double k1, double k2, bool fitK, GridModel &out)
    {
        if (ij.size() < 5) {
            return false;
        }
        cv::Matx33d Hs;
        if (H0) {
            Hs = *H0;
        } else {
            std::vector<cv::Point2f> a, b;
            for (size_t n = 0; n < ij.size(); n++) {
                a.emplace_back((float)ij[n].x, (float)ij[n].y);
                b.emplace_back((float)uv[n].x, (float)uv[n].y);
            }
            cv::Mat Hm = cv::findHomography(a, b, cv::RANSAC, 0.3 * pitchPx(uv, ij));
            if (Hm.empty()) {
                return false;
            }
            Hs = cv::Matx33d((double *)Hm.clone().ptr());
        }
        Hs = Hs * (1.0 / Hs(2, 2));
        std::vector<double> x = { Hs(0, 0), Hs(0, 1), Hs(0, 2), Hs(1, 0), Hs(1, 1), Hs(1, 2), Hs(2, 0), Hs(2, 1) };
        if (fitK) {
            x.push_back(k1);
            x.push_back(k2);
        }
        const int P = (int)x.size();
        const int M = 2 * (int)ij.size();

        std::vector<double> r, r2;
        residuals(build(x, fitK, k1, k2, centre, f), ij, uv, r);
        double cost = robustCost(r);
        double lambda = 1e-3;
        cv::Mat J(M, P, CV_64F);
        for (int iter = 0; iter < 200; iter++) {
            // numerical Jacobian
            for (int p = 0; p < P; p++) {
                std::vector<double> xp = x;
                const double h = std::max(std::abs(x[p]) * 1e-7, 1e-10);
                xp[p] += h;
                residuals(build(xp, fitK, k1, k2, centre, f), ij, uv, r2);
                for (int m = 0; m < M; m++) {
                    J.at<double>(m, p) = (r2[m] - r[m]) / h;
                }
            }
            // IRLS weights of the soft-L1 loss at the current residuals
            cv::Mat W(M, 1, CV_64F), R(M, 1, CV_64F);
            for (int m = 0; m < M; m++) {
                W.at<double>(m) = 1.0 / std::sqrt(1.0 + r[m] * r[m]);
                R.at<double>(m) = r[m];
            }
            cv::Mat JW = J.clone();
            for (int m = 0; m < M; m++) {
                cv::Mat row = JW.row(m);
                row *= W.at<double>(m);
            }
            const cv::Mat A = J.t() * JW;
            const cv::Mat g = JW.t() * R;

            bool improved = false;
            for (int tries = 0; tries < 12; tries++) {
                cv::Mat Al = A.clone();
                for (int p = 0; p < P; p++) {
                    Al.at<double>(p, p) += lambda * std::max(A.at<double>(p, p), 1e-12);
                }
                cv::Mat delta;
                if (!cv::solve(Al, -g, delta, cv::DECOMP_CHOLESKY) && !cv::solve(Al, -g, delta, cv::DECOMP_SVD)) {
                    lambda *= 10.0;
                    continue;
                }
                std::vector<double> xn = x;
                for (int p = 0; p < P; p++) {
                    xn[p] += delta.at<double>(p);
                }
                residuals(build(xn, fitK, k1, k2, centre, f), ij, uv, r2);
                const double cn = robustCost(r2);
                if (cn < cost) {
                    const double rel = (cost - cn) / std::max(cost, 1e-12);
                    x = xn;
                    r = r2;
                    cost = cn;
                    lambda = std::max(lambda / 3.0, 1e-12);
                    improved = true;
                    if (rel < 1e-10) {
                        iter = 1000;   // converged
                    }
                    break;
                }
                lambda *= 4.0;
            }
            if (!improved) {
                break;
            }
        }
        out = build(x, fitK, k1, k2, centre, f);
        return true;
    }

    void cellsInView(const GridModel &m, const std::vector<cv::Point2d> &ij, const cv::Size &size, int margin,
                     std::vector<cv::Point2d> &cells, std::vector<cv::Point2d> &pred)
    {
        double lo0 = 1e9, lo1 = 1e9, hi0 = -1e9, hi1 = -1e9;
        for (const cv::Point2d &c : ij) {
            lo0 = std::min(lo0, c.x);
            hi0 = std::max(hi0, c.x);
            lo1 = std::min(lo1, c.y);
            hi1 = std::max(hi1, c.y);
        }
        cells.clear();
        pred.clear();
        for (int i = (int)lo0 - margin; i <= (int)hi0 + margin; i++) {
            for (int j = (int)lo1 - margin; j <= (int)hi1 + margin; j++) {
                const cv::Point2d c(i, j);
                const cv::Point2d p = m(c);
                if (p.x > -20 && p.y > -20 && p.x < size.width + 20 && p.y < size.height + 20) {
                    cells.push_back(c);
                    pred.push_back(p);
                }
            }
        }
    }

    cv::Matx22d ellipseCov(const cv::Size2d &axes, double angleDeg)
    {
        const double a = axes.width / 2.0, b = axes.height / 2.0;
        const double t = angleDeg * CV_PI / 180.0, c = std::cos(t), s = std::sin(t);
        const cv::Matx22d R(c, -s, s, c);
        const cv::Matx22d D(a * a, 0, 0, b * b);
        return R * D * R.t();
    }

    // Drop detections whose ellipse doesn't match their lattice neighbours' (compared LOCALLY against the
    // median ellipse of the matched holes within `radius` cells -- see holegrid.py for why not globally).
    void pruneShapes(std::vector<int> &det, std::vector<cv::Point2d> &ij, const Detections &shapes,
                     double maxDev = 0.25, double radius = 2.01)
    {
        const int N = (int)det.size();
        std::vector<cv::Matx22d> C(N);
        for (int n = 0; n < N; n++) {
            C[n] = ellipseCov(shapes.axes[det[n]], shapes.angles[det[n]]);
        }
        std::vector<int> keepDet;
        std::vector<cv::Point2d> keepIJ;
        for (int n = 0; n < N; n++) {
            std::vector<int> nb;
            for (int m = 0; m < N; m++) {
                if (m != n && norm2(ij[m] - ij[n]) <= radius) {
                    nb.push_back(m);
                }
            }
            double dev = 0.0;
            if (nb.size() >= 3) {
                cv::Matx22d Cm;
                for (int e = 0; e < 4; e++) {
                    std::vector<double> v;
                    for (int m : nb) {
                        v.push_back(C[m].val[e]);
                    }
                    Cm.val[e] = median(v);
                }
                dev = cv::norm(C[n] - Cm) / std::max(cv::norm(Cm), 1e-12);
            }
            if (dev < maxDev) {
                keepDet.push_back(det[n]);
                keepIJ.push_back(ij[n]);
            }
        }
        det = keepDet;
        ij = keepIJ;
    }

    Lattice fitAndFill(const std::vector<cv::Point2d> &pts, std::vector<int> det, std::vector<cv::Point2d> ij, double tol,
                       const cv::Size &size, const Detections *shapes, int rounds = 4)
    {
        Lattice L;
        const cv::Point2d centre(size.width / 2.0, size.height / 2.0);
        const double f = 0.55 * std::max(size.width, size.height);
        GridModel model;
        bool haveModel = false;
        auto sub = [&](const std::vector<int> &d) {
            std::vector<cv::Point2d> u;
            for (int k : d) {
                u.push_back(pts[k]);
            }
            return u;
        };
        for (int rnd = 0; rnd < rounds; rnd++) {
            // first round: straight homography only; afterwards the lens distortion too
            GridModel m;
            if (!fitGridModel(ij, sub(det), centre, f, haveModel ? &model.H : nullptr,
                              haveModel ? model.k1 : 0.0, haveModel ? model.k2 : 0.0, rnd > 0, m)) {
                return L;
            }
            model = m;
            haveModel = true;
            std::vector<cv::Point2d> cells, pred;
            cellsInView(model, ij, size, 3, cells, pred);
            std::map<int, std::pair<cv::Point2d, double>> best;   // detection -> (cell, distance)
            for (size_t c = 0; c < cells.size(); c++) {
                const cv::Matx22d J = model.jacobian(cells[c]);
                const double loc = std::min(std::hypot(J(0, 0), J(1, 0)), std::hypot(J(0, 1), J(1, 1)));
                double d;
                const int q = nearest(pts, pred[c], &d);
                if (q >= 0 && d < tol * loc) {
                    auto it = best.find(q);
                    if (it == best.end() || d < it->second.second) {
                        best[q] = { cells[c], d };
                    }
                }
            }
            if (best.size() < 6) {
                return L;
            }
            det.clear();
            ij.clear();
            for (const auto &b : best) {
                det.push_back(b.first);
                ij.push_back(b.second.first);
            }
        }
        GridModel m;
        if (!fitGridModel(ij, sub(det), centre, f, &model.H, model.k1, model.k2, true, m)) {
            return L;
        }
        model = m;
        if (shapes) {
            pruneShapes(det, ij, *shapes);
        }
        if (det.size() < 6) {
            return L;
        }
        if (!fitGridModel(ij, sub(det), centre, f, &model.H, model.k1, model.k2, true, m)) {
            return L;
        }
        model = m;

        double ss = 0.0;
        for (size_t n = 0; n < det.size(); n++) {
            const cv::Point2d e = model(ij[n]) - pts[det[n]];
            ss += e.x * e.x + e.y * e.y;
        }
        L.rms = std::sqrt(ss / det.size());
        // cells the model puts inside the image (with a hole's margin) that no detection claimed
        std::vector<cv::Point2d> cells, pred;
        cellsInView(model, ij, size, 6, cells, pred);
        std::map<std::pair<int, int>, bool> have;
        for (const cv::Point2d &c : ij) {
            have[{ (int)std::lround(c.x), (int)std::lround(c.y) }] = true;
        }
        for (size_t c = 0; c < cells.size(); c++) {
            const cv::Point2d &p = pred[c];
            const std::pair<int, int> key((int)std::lround(cells[c].x), (int)std::lround(cells[c].y));
            if (p.x > 15 && p.y > 15 && p.x < size.width - 15 && p.y < size.height - 15 && !have.count(key)) {
                L.expected.emplace_back(key.first, key.second);
            }
        }
        for (size_t n = 0; n < det.size(); n++) {
            L.ij.emplace_back((int)std::lround(ij[n].x), (int)std::lround(ij[n].y));
            L.uv.push_back(pts[det[n]]);
        }
        L.det = det;
        L.model = model;
        L.valid = true;
        return L;
    }
}

// ============================================================================================ GridModel
cv::Point2d GridModel::ideal(const cv::Point2d &ij) const
{
    const double w = H(2, 0) * ij.x + H(2, 1) * ij.y + H(2, 2);
    return cv::Point2d((H(0, 0) * ij.x + H(0, 1) * ij.y + H(0, 2)) / w,
                       (H(1, 0) * ij.x + H(1, 1) * ij.y + H(1, 2)) / w);
}

cv::Point2d GridModel::operator()(const cv::Point2d &ij) const
{
    const cv::Point2d x = (ideal(ij) - centre) * (1.0 / f);
    const double r2 = x.x * x.x + x.y * x.y;
    return centre + f * x * (1.0 + k1 * r2 + k2 * r2 * r2);
}

cv::Matx22d GridModel::jacobian(const cv::Point2d &ij, double h) const
{
    const cv::Point2d p0 = (*this)(ij);
    const cv::Point2d di = ((*this)(ij + cv::Point2d(h, 0)) - p0) * (1.0 / h);
    const cv::Point2d dj = ((*this)(ij + cv::Point2d(0, h)) - p0) * (1.0 / h);
    return cv::Matx22d(di.x, dj.x, di.y, dj.y);
}

double GridModel::cornerShift(const cv::Size &size) const
{
    const cv::Point2d x = (cv::Point2d(size.width, size.height) - centre) * (1.0 / f);
    const double r2 = x.x * x.x + x.y * x.y;
    return f * std::sqrt(r2) * std::abs(k1 * r2 + k2 * r2 * r2);
}

// ============================================================================================ detect
Detections LAUHoleGrid::detectHoles(const cv::Mat &gray, double minD, double maxD, double thresh)
{
    Detections out;
    cv::Mat g;
    gray.convertTo(g, CV_32F);
    const int k = odd(maxD * 1.4);

    // board illumination with the holes closed over -- smooth, so estimated at 1/4 scale
    const int s = 4;
    cv::Mat small, bg;
    cv::resize(g, small, cv::Size(g.cols / s, g.rows / s), 0, 0, cv::INTER_AREA);
    cv::morphologyEx(small, bg, cv::MORPH_CLOSE, disc(odd(k / (double)s)));
    cv::GaussianBlur(bg, bg, cv::Size(0, 0), k / 6.0 / s);
    cv::resize(bg, bg, g.size(), 0, 0, cv::INTER_LINEAR);

    cv::Mat gb, ratio;
    cv::GaussianBlur(g, gb, cv::Size(0, 0), 0.8);
    cv::divide(gb, cv::max(bg, 1.0), ratio);
    out.ratio = ratio;

    // dark blobs, plus any bright highlight (thread stripe, lit hole wall) touching one
    cv::Mat dark = ratio < thresh;
    cv::Mat bright = ratio > BRIGHT;
    cv::Mat nearDark;
    cv::dilate(dark, nearDark, disc(odd(minD * 0.5)));
    cv::Mat mask = dark | (bright & nearDark);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, disc(odd(minD * 0.6)));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, disc(odd(minD * 0.8)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    const double aMin = CV_PI * (minD / 2) * (minD / 2) * 0.5;
    const double aMax = CV_PI * (maxD / 2) * (maxD / 2) * 1.5;
    for (const std::vector<cv::Point> &c : contours) {
        const double area = cv::contourArea(c);
        if (!(aMin < area && area < aMax) || c.size() < 10) {
            continue;
        }
        const cv::RotatedRect e = cv::fitEllipse(c);
        const double A = e.size.width, B = e.size.height;
        if (std::min(A, B) < minD * 0.5 || std::max(A, B) / std::max(std::min(A, B), 1e-6) > 3.5) {
            continue;
        }
        const double fill = area / (CV_PI * A * B / 4.0);
        std::vector<cv::Point> hull;
        cv::convexHull(c, hull);
        const double solidity = area / std::max(cv::contourArea(hull), 1e-6);
        if (!(0.72 < fill && fill < 1.15) || solidity < 0.85) {
            continue;
        }
        const cv::Rect br = cv::boundingRect(c);
        if (br.x == 0 || br.y == 0 || br.x + br.width >= g.cols || br.y + br.height >= g.rows) {
            continue;                                       // cut by the image edge
        }
        cv::RotatedRect r;
        if (!refine(ratio, cv::Point2d(e.center), cv::Size2d(A, B), thresh, r)) {
            continue;
        }
        cv::Mat inside = cv::Mat::zeros(br.size(), CV_8U);
        cv::drawContours(inside, std::vector<std::vector<cv::Point>>{ c }, -1, cv::Scalar(255), cv::FILLED,
                         cv::LINE_8, cv::noArray(), INT_MAX, -br.tl());
        out.centers.emplace_back(r.center.x, r.center.y);
        out.axes.emplace_back(r.size.width, r.size.height);
        out.angles.push_back(r.angle);
        out.contrast.push_back(cv::mean(ratio(br), inside)[0]);
    }
    return out;
}

// ============================================================================================ lattice
Lattice LAUHoleGrid::growLattice(const std::vector<cv::Point2d> &pts, const cv::Size &imageSize, const Detections *shapes, double tol)
{
    Lattice none;
    const int n = (int)pts.size();
    if (n < 6) {
        return none;
    }
    // k nearest neighbours (self first)
    const int K = std::min(7, n);
    std::vector<std::vector<int>> nb(n);
    for (int a = 0; a < n; a++) {
        std::vector<std::pair<double, int>> d;
        for (int b = 0; b < n; b++) {
            d.push_back({ norm2(pts[a] - pts[b]), b });
        }
        std::partial_sort(d.begin(), d.begin() + K, d.end());
        for (int k = 0; k < K; k++) {
            nb[a].push_back(d[k].second);
        }
    }
    const cv::Point2d centre(imageSize.width / 2.0, imageSize.height / 2.0);
    std::vector<int> order(n);
    for (int a = 0; a < n; a++) {
        order[a] = a;
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) { return norm2(pts[a] - centre) < norm2(pts[b] - centre); });

    // seed: a central detection whose 4 nearest neighbours are a full "+" (two opposite pairs)
    int seed = -1;
    cv::Point2d basis[2];
    for (int t = 0; t < std::min(60, n) && seed < 0; t++) {
        const int s = order[t];
        if ((int)nb[s].size() < 5) {
            continue;
        }
        cv::Point2d v4[4];
        for (int k = 0; k < 4; k++) {
            v4[k] = pts[nb[s][k + 1]] - pts[s];
        }
        std::vector<std::pair<int, int>> pairs;
        for (int p = 0; p < 4; p++) {
            for (int q = p + 1; q < 4; q++) {
                if (norm2(v4[p] + v4[q]) < tol * std::min(norm2(v4[p]), norm2(v4[q]))) {
                    pairs.push_back({ p, q });
                }
            }
        }
        if (pairs.size() != 2) {
            continue;
        }
        if (pairs[0].first == pairs[1].first || pairs[0].first == pairs[1].second ||
            pairs[0].second == pairs[1].first || pairs[0].second == pairs[1].second) {
            continue;
        }
        const cv::Point2d a = v4[pairs[0].first], b = v4[pairs[1].first];
        if (std::abs(a.dot(b)) / (norm2(a) * norm2(b)) < 0.5) {
            seed = s;
            basis[0] = a;
            basis[1] = b;
        }
    }
    if (seed < 0) {
        return none;
    }

    // flood outward by linear extrapolation
    std::map<int, std::pair<int, int>> idx;
    std::map<std::pair<int, int>, int> at;
    std::vector<int> insertion;
    idx[seed] = { 0, 0 };
    at[{ 0, 0 }] = seed;
    insertion.push_back(seed);
    std::deque<int> queue{ seed };
    const int steps[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
    auto get = [&](int i, int j) {
        auto it = at.find({ i, j });
        return it == at.end() ? -1 : it->second;
    };
    while (!queue.empty()) {
        const int s = queue.front();
        queue.pop_front();
        const int i = idx[s].first, j = idx[s].second;
        const cv::Point2d p = pts[s];
        for (const auto &st : steps) {
            const int di = st[0], dj = st[1];
            if (get(i + di, j + dj) >= 0) {
                continue;
            }
            cv::Point2d step;
            const int prev = get(i - di, j - dj);
            if (prev >= 0) {
                step = p - pts[prev];
            } else {
                step = di * basis[0] + dj * basis[1];
                const int alt[2][2] = { { i + dj, j + di }, { i - dj, j - di } };
                for (const auto &a : alt) {
                    const int q = get(a[0], a[1]), r = get(a[0] - di, a[1] - dj);
                    if (q >= 0 && r >= 0) {
                        step = pts[q] - pts[r];
                        break;
                    }
                }
            }
            double dist;
            const int q = nearest(pts, p + step, &dist);
            if (q >= 0 && dist < tol * norm2(step) && !idx.count(q)) {
                idx[q] = { i + di, j + dj };
                at[{ i + di, j + dj }] = q;
                insertion.push_back(q);
                queue.push_back(q);
            }
        }
    }
    if (idx.size() < 6) {
        return none;
    }
    std::vector<int> det;
    std::vector<cv::Point2d> ij;
    for (int k : insertion) {
        det.push_back(k);
        ij.emplace_back(idx[k].first, idx[k].second);
    }
    return fitAndFill(pts, det, ij, tol, imageSize, shapes);
}
