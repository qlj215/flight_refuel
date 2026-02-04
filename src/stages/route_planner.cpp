#include "stages/route_planner.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <queue>
#include <limits>

namespace routeplan {

static constexpr double PI = 3.14159265358979323846;

static inline double length(const Vec2 &a) { return std::sqrt(a.x*a.x + a.y*a.y); }
static inline Vec2 operator+(const Vec2 &a, const Vec2 &b) { return {a.x+b.x, a.y+b.y}; }
static inline Vec2 operator-(const Vec2 &a, const Vec2 &b) { return {a.x-b.x, a.y-b.y}; }
static inline Vec2 operator*(const Vec2 &a, double s) { return {a.x*s, a.y*s}; }
static inline Vec2 operator/(const Vec2 &a, double s) { return {a.x/s, a.y/s}; }
static inline double dot(const Vec2 &a, const Vec2 &b) { return a.x*b.x + a.y*b.y; }
static inline double cross(const Vec2 &a, const Vec2 &b) { return a.x*b.y - a.y*b.x; }
static inline Vec2 leftNormal(const Vec2 &v) { return {-v.y, v.x}; }

static inline Vec2 normalize(const Vec2 &a) {
    double L = length(a);
    if (L < 1e-12) return {0,0};
    return a / L;
}

static inline double wrapToPi(double a) {
    while (a <= -PI) a += 2.0*PI;
    while (a >  PI)  a -= 2.0*PI;
    return a;
}

// ======================= Local frame =======================
struct LocalFrame { Vec2 origin, ex, ey; };

static LocalFrame buildLocalFrame(const Vec2 &start, const Vec2 &goal) {
    Vec2 d = normalize(goal - start);
    if (length(d) < 1e-12) d = {1,0};
    Vec2 ex = d;
    Vec2 ey = leftNormal(ex);
    return {start, ex, ey};
}

static Vec2 worldToLocal(const LocalFrame &f, const Vec2 &p) {
    Vec2 v = p - f.origin;
    return {dot(v, f.ex), dot(v, f.ey)};
}
static Vec2 localToWorld(const LocalFrame &f, const Vec2 &p) {
    return f.origin + f.ex*p.x + f.ey*p.y;
}

// ======================= Sampling =======================
static void sampleLine(const Vec2 &from, const Vec2 &to, double step, std::vector<Vec2> &out) {
    Vec2 d = to - from;
    double L = length(d);
    if (L < 1e-10) { out.push_back(to); return; }
    Vec2 v = d / L;
    if (step <= 1e-6) step = 1.0;

    int n = (int)std::floor(L / step);
    for (int i = 1; i <= n; ++i) {
        double s = step * i;
        if (s >= L) break;
        out.push_back(from + v*s);
    }
    out.push_back(to);
}

// ======================= Obstacles =======================
struct Rect { double xmin, xmax, ymin, ymax; };

static inline bool pointInsideRectStrict(const Vec2& p, const Rect& r) {
    const double eps = 1e-12;
    return (p.x > r.xmin + eps && p.x < r.xmax - eps &&
            p.y > r.ymin + eps && p.y < r.ymax - eps);
}

static inline bool segmentIntersectsRectInterior(const Vec2& p0, const Vec2& p1, const Rect& r) {
    if (pointInsideRectStrict(p0, r) || pointInsideRectStrict(p1, r)) return true;

    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;

    auto clip = [&](double p, double q, double &u1, double &u2)->bool {
        if (std::fabs(p) < 1e-15) {
            if (q < 0) return false;
            return true;
        }
        double t = q / p;
        if (p < 0) { if (t > u2) return false; if (t > u1) u1 = t; }
        else       { if (t < u1) return false; if (t < u2) u2 = t; }
        return true;
    };

    double u1 = 0.0, u2 = 1.0;
    if (!clip(-dx, p0.x - r.xmin, u1, u2)) return false;
    if (!clip( dx, r.xmax - p0.x, u1, u2)) return false;
    if (!clip(-dy, p0.y - r.ymin, u1, u2)) return false;
    if (!clip( dy, r.ymax - p0.y, u1, u2)) return false;

    const double epsLen = 1e-10;
    return (u2 - u1 > epsLen);
}

static inline bool segmentHitsAnyRectInterior(const Vec2& a, const Vec2& b, const std::vector<Rect>& rects) {
    double segMinX = std::min(a.x, b.x), segMaxX = std::max(a.x, b.x);
    double segMinY = std::min(a.y, b.y), segMaxY = std::max(a.y, b.y);
    for (const auto& r : rects) {
        if (segMaxX < r.xmin || segMinX > r.xmax || segMaxY < r.ymin || segMinY > r.ymax) continue;
        if (segmentIntersectsRectInterior(a, b, r)) return true;
    }
    return false;
}

static std::vector<Rect> buildInflatedRectsFromPolysLocal(
        const std::vector<std::vector<Vec2>>& dangerPolysLocal,
        double inflate)
{
    std::vector<Rect> rects;
    rects.reserve(dangerPolysLocal.size());
    for (const auto& poly : dangerPolysLocal) {
        if (poly.empty()) continue;
        double xmin =  1e300, xmax = -1e300;
        double ymin =  1e300, ymax = -1e300;
        for (const auto& p : poly) {
            xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
            ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
        }
        rects.push_back({xmin - inflate, xmax + inflate, ymin - inflate, ymax + inflate});
    }
    return rects;
}

static std::vector<Vec2> buildShortestPolylineLocalViaVisibilityGraph(
        const Vec2& startL,
        const Vec2& goalL,
        const std::vector<Rect>& inflatedRects)
{
    for (const auto& r : inflatedRects) {
        if (pointInsideRectStrict(startL, r) || pointInsideRectStrict(goalL, r))
            return {startL, goalL};
    }

    std::vector<Vec2> nodes;
    nodes.reserve(2 + inflatedRects.size() * 4);
    nodes.push_back(startL); // 0
    nodes.push_back(goalL);  // 1
    for (const auto& r : inflatedRects) {
        nodes.push_back({r.xmin, r.ymin});
        nodes.push_back({r.xmin, r.ymax});
        nodes.push_back({r.xmax, r.ymin});
        nodes.push_back({r.xmax, r.ymax});
    }

    int N = (int)nodes.size();
    std::vector<std::vector<std::pair<int,double>>> adj(N);

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            if (!segmentHitsAnyRectInterior(nodes[i], nodes[j], inflatedRects)) {
                double w = length(nodes[j] - nodes[i]);
                adj[i].push_back({j, w});
                adj[j].push_back({i, w});
            }
        }
    }

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<int> prev(N, -1);

    using QN = std::pair<double,int>;
    std::priority_queue<QN, std::vector<QN>, std::greater<QN>> pq;
    dist[0] = 0.0;
    pq.push({0.0, 0});

    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        if (u == 1) break;
        for (auto [v,w] : adj[u]) {
            double nd = d + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (!std::isfinite(dist[1])) return {startL, goalL};

    std::vector<int> idx;
    for (int cur = 1; cur != -1; cur = prev[cur]) idx.push_back(cur);
    std::reverse(idx.begin(), idx.end());

    std::vector<Vec2> poly;
    poly.reserve(idx.size());
    for (int id : idx) poly.push_back(nodes[id]);

    // remove consecutive near-duplicates
    std::vector<Vec2> cleaned;
    cleaned.reserve(poly.size());
    for (const auto& p : poly) {
        if (cleaned.empty() || length(p - cleaned.back()) > 1e-6) cleaned.push_back(p);
    }
    if (cleaned.size() < 2) cleaned = {startL, goalL};
    return cleaned;
}

// ======================= Polyline simplification =======================
static inline bool isProtectedIndex(size_t i, const std::vector<size_t>& prot) {
    for (size_t k : prot) if (k == i) return true;
    return false;
}

static void simplifyPolylineShortcut(
        std::vector<Vec2>& pts,
        const std::vector<Rect>& rects,
        const std::vector<size_t>& protectedIdx)
{
    if (pts.size() <= 2) return;

    bool changed = true;
    while (changed && pts.size() > 2) {
        changed = false;
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            if (isProtectedIndex(i, protectedIdx)) continue;
            if (!segmentHitsAnyRectInterior(pts[i-1], pts[i+1], rects)) {
                pts.erase(pts.begin() + i);
                changed = true;
                break;
            }
        }
    }

    auto almostCollinear = [&](const Vec2& a, const Vec2& b, const Vec2& c)->bool {
        Vec2 ab = b - a;
        Vec2 bc = c - b;
        double lab = length(ab), lbc = length(bc);
        if (lab < 1e-9 || lbc < 1e-9) return true;
        Vec2 u = ab / lab, v = bc / lbc;
        return dot(u, v) > 0.9999;
    };

    for (;;) {
        bool removed = false;
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            if (isProtectedIndex(i, protectedIdx)) continue;
            if (almostCollinear(pts[i-1], pts[i], pts[i+1])) {
                pts.erase(pts.begin() + i);
                removed = true;
                break;
            }
        }
        if (!removed) break;
    }
}

// ======================= Strict fillet (no kinks) =======================
static bool sampleSmoothPathWithFilletStrict(
        const std::vector<Vec2> &poly,
        double R,
        double step,
        std::vector<Vec2> &out)
{
    out.clear();
    if (poly.size() < 2) return false;

    if (poly.size() == 2 || R <= 1e-3) {
        out.push_back(poly[0]);
        sampleLine(poly[0], poly[1], step, out);
        return true;
    }

    const double angle_eps = 2.0 * PI / 180.0;
    const double eps = 1e-9;

    out.push_back(poly[0]);
    Vec2 cur = poly[0];

    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        Vec2 A = cur;
        Vec2 B = poly[i];
        Vec2 C = poly[i+1];

        Vec2 e1 = B - A, e2 = C - B;
        double len1 = length(e1), len2 = length(e2);
        if (len1 < eps || len2 < eps) return false;

        Vec2 v1 = e1 / len1; // A->B
        Vec2 v2 = e2 / len2; // B->C

        double cosTheta = -dot(v1, v2);
        cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
        double theta = std::acos(cosTheta);
        if (theta < angle_eps) {
            sampleLine(cur, B, step, out);
            cur = B;
            continue;
        }

        double turnSign = (cross(v1, v2) > 0.0) ? +1.0 : -1.0;

        Vec2 n1 = normalize(leftNormal(v1));
        Vec2 n2 = normalize(leftNormal(v2));
        if (turnSign < 0.0) { n1 = n1 * -1.0; n2 = n2 * -1.0; }

        Vec2 P1 = B + n1 * R;
        Vec2 P2 = B + n2 * R;

        double denom = cross(v1, v2);
        if (std::fabs(denom) < 1e-12) {
            sampleLine(cur, B, step, out);
            cur = B;
            continue;
        }

        double k1 = cross(P2 - P1, v2) / denom;
        Vec2 center = P1 + v1 * k1;

        Vec2 T1 = center - n1 * R;
        Vec2 T2 = center - n2 * R;

        double u1 = dot(T1 - A, v1);
        double u2 = dot(T2 - B, v2);
        if (!(u1 > 1e-6 && u1 < len1 - 1e-6 && u2 > 1e-6 && u2 < len2 - 1e-6))
            return false;

        sampleLine(cur, T1, step, out);

        double ang0 = std::atan2(T1.y - center.y, T1.x - center.x);
        double ang1 = std::atan2(T2.y - center.y, T2.x - center.x);
        double dAng = wrapToPi(ang1 - ang0);

        if (turnSign > 0.0 && dAng < 0.0) dAng += 2.0*PI;
        if (turnSign < 0.0 && dAng > 0.0) dAng -= 2.0*PI;

        double arcLen = std::fabs(dAng) * R;
        int nArc = std::max(1, (int)std::ceil(arcLen / std::max(step, 1.0)));

        for (int k = 1; k <= nArc; ++k) {
            double r = (double)k / (double)nArc;
            double ang = ang0 + dAng * r;
            out.push_back({center.x + R*std::cos(ang), center.y + R*std::sin(ang)});
        }

        cur = T2;
    }

    sampleLine(cur, poly.back(), step, out);
    return true;
}

// ======================= Spline fallback =======================
static std::vector<Vec2> sampleCatmullRomPath(const std::vector<Vec2> &pts, double step)
{
    std::vector<Vec2> out;
    if (pts.size() < 2) return out;
    out.push_back(pts[0]);

    int N = (int)pts.size();
    for (int i = 0; i < N - 1; ++i) {
        Vec2 P0 = (i == 0 ? pts[0] : pts[i - 1]);
        Vec2 P1 = pts[i];
        Vec2 P2 = pts[i + 1];
        Vec2 P3 = (i + 2 >= N ? pts[N - 1] : pts[i + 2]);

        double L = length(P2 - P1);
        int nSeg = std::max(1, (int)std::ceil(L / std::max(step, 1.0)));

        for (int k = 1; k <= nSeg; ++k) {
            double t  = (double)k / (double)nSeg;
            double t2 = t*t;
            double t3 = t2*t;

            Vec2 p;
            p.x = 0.5 * ((2.0 * P1.x) +
                         (-P0.x + P2.x) * t +
                         (2.0*P0.x - 5.0*P1.x + 4.0*P2.x - P3.x) * t2 +
                         (-P0.x + 3.0*P1.x - 3.0*P2.x + P3.x) * t3);
            p.y = 0.5 * ((2.0 * P1.y) +
                         (-P0.y + P2.y) * t +
                         (2.0*P0.y - 5.0*P1.y + 4.0*P2.y - P3.y) * t2 +
                         (-P0.y + 3.0*P1.y - 3.0*P2.y + P3.y) * t3);

            out.push_back(p);
        }
    }
    return out;
}

// ======================= Altitude profile =======================
static std::vector<double> buildAltitudeProfile(
        const std::vector<double>& s,
        double z0, double z_mid, double z_goal,
        double groundSpeed, double climbRate, double descendRate,
        double finishBeforeEndDist)
{
    size_t N = s.size();
    std::vector<double> z(N, z_mid);
    if (N == 0) return z;

    groundSpeed = std::max(1e-3, groundSpeed);
    climbRate   = std::max(1e-6, climbRate);
    descendRate = std::max(1e-6, descendRate);

    double slopeUp = climbRate / groundSpeed;
    double slopeDn = descendRate / groundSpeed;

    double S = s.back();
    double s_end = std::max(0.0, S - finishBeforeEndDist);

    size_t i_end = 0;
    while (i_end + 1 < N && s[i_end + 1] <= s_end) i_end++;

    for (size_t i = 0; i < N; ++i) z[i] = (s[i] >= s_end ? z_goal : z_mid);
    z[0] = z0;
    z[i_end] = z_goal;

    auto forwardPass = [&]() {
        for (size_t i = 1; i <= i_end; ++i) {
            double ds = s[i] - s[i-1];
            double lo = z[i-1] - slopeDn * ds;
            double hi = z[i-1] + slopeUp * ds;
            z[i] = std::min(std::max(z[i], lo), hi);
        }
    };
    auto backwardPass = [&]() {
        for (size_t ii = i_end; ii-- > 0;) {
            double ds = s[ii+1] - s[ii];
            double lo = z[ii+1] - slopeUp * ds;
            double hi = z[ii+1] + slopeDn * ds;
            z[ii] = std::min(std::max(z[ii], lo), hi);
        }
        z[0] = z0;
    };

    for (int it = 0; it < 4; ++it) {
        forwardPass();
        backwardPass();
        z[i_end] = z_goal;
    }

    for (size_t i = i_end; i < N; ++i) z[i] = z_goal;
    return z;
}

// ============================================================
// Public API
// ============================================================
PlanResult PlanRoute(
    const Pose3D& start,
    const Pose3D& goal,
    const std::vector<std::vector<Vec2>>& dangerPolygonsWorld,
    const PlannerConfig& cfg)
{
    PlanResult res;
    res.status = PlanStatus::Failed;

    if (cfg.step <= 0.0 || cfg.turnRadius <= 0.0 || cfg.groundSpeed <= 0.0) {
        return res;
    }

    Vec2 start2{start.x, start.y};
    Vec2 goal2 {goal.x,  goal.y};

    // Hard final straight-in
    Vec2 dirGoal{ std::cos(goal.heading), std::sin(goal.heading) };
    if (length(dirGoal) < 1e-12) dirGoal = {1,0};

    Vec2 G_entryW = goal2 - dirGoal * cfg.finalStraightLen;

    double approachLen = std::max(cfg.approachLenMinFactorR * cfg.turnRadius,
                                  cfg.approachLenMinFactorStep * cfg.step);
    Vec2 A_w = G_entryW - dirGoal * approachLen;

    // Local frame
    LocalFrame lf = buildLocalFrame(start2, goal2);
    Vec2 startL{0.0, 0.0};
    Vec2 goalL     = worldToLocal(lf, goal2);
    Vec2 entryL    = worldToLocal(lf, G_entryW);
    Vec2 approachL = worldToLocal(lf, A_w);

    // danger polygons -> local
    std::vector<std::vector<Vec2>> dangerLocal;
    dangerLocal.reserve(dangerPolygonsWorld.size());
    for (const auto& polyW : dangerPolygonsWorld) {
        std::vector<Vec2> polyL;
        polyL.reserve(polyW.size());
        for (const auto& p : polyW) polyL.push_back(worldToLocal(lf, p));
        dangerLocal.push_back(std::move(polyL));
    }

    double baseInflate = cfg.safetyMargin + cfg.turnRadius;
    double inflateStep = cfg.inflateStepFactorR * cfg.turnRadius;

    std::vector<Vec2> finalPath2D;
    std::vector<Vec2> finalCoarseWorld;

    bool strictOk = false;
    bool finalStraightBlockedAtLeastOnce = false;

    for (int attempt = 0; attempt < cfg.maxInflateAttempts; ++attempt) {
        double inflate = baseInflate + attempt * inflateStep;
        std::vector<Rect> inflatedRects = buildInflatedRectsFromPolysLocal(dangerLocal, inflate);

        bool finalBlocked =
            segmentHitsAnyRectInterior(approachL, entryL, inflatedRects) ||
            segmentHitsAnyRectInterior(entryL, goalL, inflatedRects);
        if (finalBlocked) finalStraightBlockedAtLeastOnce = true;

        // start -> approach
        std::vector<Vec2> coarseLocal = buildShortestPolylineLocalViaVisibilityGraph(startL, approachL, inflatedRects);

        auto pushIfFar = [&](const Vec2& p) {
            if (coarseLocal.empty() || length(p - coarseLocal.back()) > 1e-6) coarseLocal.push_back(p);
        };
        pushIfFar(approachL);
        pushIfFar(entryL);
        pushIfFar(goalL);

        size_t idxGoal = coarseLocal.size() - 1;
        size_t idxEntry = coarseLocal.size() - 2;
        size_t idxA = coarseLocal.size() - 3;
        std::vector<size_t> protectedIdx{0, idxA, idxEntry, idxGoal};

        simplifyPolylineShortcut(coarseLocal, inflatedRects, protectedIdx);

        // to world
        std::vector<Vec2> coarseWorld;
        coarseWorld.reserve(coarseLocal.size());
        for (const auto& pL : coarseLocal) coarseWorld.push_back(localToWorld(lf, pL));

        std::vector<Vec2> smoothed;
        bool ok = (!finalBlocked) && sampleSmoothPathWithFilletStrict(coarseWorld, cfg.turnRadius, cfg.step, smoothed);

        if (ok && smoothed.size() >= 2) {
            finalPath2D = std::move(smoothed);
            finalCoarseWorld = std::move(coarseWorld);
            strictOk = true;
            break;
        }
    }

    if (!strictOk) {
        if (finalStraightBlockedAtLeastOnce) {
            res.status = PlanStatus::InfeasibleFinalStraight;
            if (!cfg.allowSplineFallback) return res;
        }

        if (!cfg.allowSplineFallback) return res;

        // Fallback: spline smoothing (no kinks)
        double inflate = baseInflate + cfg.maxInflateAttempts * inflateStep;
        std::vector<Rect> inflatedRects = buildInflatedRectsFromPolysLocal(dangerLocal, inflate);

        std::vector<Vec2> coarseLocal = buildShortestPolylineLocalViaVisibilityGraph(startL, approachL, inflatedRects);
        auto pushIfFar = [&](const Vec2& p) {
            if (coarseLocal.empty() || length(p - coarseLocal.back()) > 1e-6) coarseLocal.push_back(p);
        };
        pushIfFar(approachL);
        pushIfFar(entryL);
        pushIfFar(goalL);

        size_t idxGoal = coarseLocal.size() - 1;
        size_t idxEntry = coarseLocal.size() - 2;
        size_t idxA = coarseLocal.size() - 3;
        std::vector<size_t> protectedIdx{0, idxA, idxEntry, idxGoal};

        simplifyPolylineShortcut(coarseLocal, inflatedRects, protectedIdx);

        std::vector<Vec2> coarseWorld;
        for (const auto& pL : coarseLocal) coarseWorld.push_back(localToWorld(lf, pL));
        finalCoarseWorld = coarseWorld;

        finalPath2D = sampleCatmullRomPath(coarseWorld, cfg.step);
        if (finalPath2D.size() < 2) return res;

        res.status = PlanStatus::SuccessSplineFallback;
    } else {
        res.status = PlanStatus::SuccessStrictRadius;
    }

    // Enforce endpoints
    finalPath2D.front() = start2;
    finalPath2D.back()  = goal2;

    // cumulative distance
    size_t N = finalPath2D.size();
    std::vector<double> s(N, 0.0);
    for (size_t i = 1; i < N; ++i) s[i] = s[i-1] + length(finalPath2D[i] - finalPath2D[i-1]);

    // altitude
    std::vector<double> z = buildAltitudeProfile(
        s, start.z, cfg.z_mid, goal.z,
        cfg.groundSpeed, cfg.climbRate, cfg.descendRate,
        cfg.finalStraightLen // finish before end = 2000
    );

    res.route.reserve(N);
    for (size_t i = 0; i < N; ++i) res.route.push_back({finalPath2D[i].x, finalPath2D[i].y, z[i]});

    if (!res.route.empty()) {
        res.route.front().z = start.z;
        res.route.back().z  = goal.z;
    }

    res.coarseWorld = std::move(finalCoarseWorld);
    return res;
}

bool WriteRouteCsv(const char* filename, const std::vector<Waypoint>& route) {
    std::ofstream fout(filename);
    if (!fout.is_open()) return false;
    fout << "x,y,z\n";
    fout.setf(std::ios::fixed);
    fout.precision(3);
    for (const auto& w : route) {
        fout << w.x << "," << w.y << "," << w.z << "\n";
    }
    return true;
}

} // namespace routeplan
