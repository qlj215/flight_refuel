#pragma once
#include <vector>

namespace routeplan {

struct Vec2 {
    double x{0}, y{0};
};

struct Pose3D {
    double x{0}, y{0}, z{0};
    // heading in radians, 0 = +X axis, CCW positive
    double heading{0};
};

struct Waypoint {
    double x{0}, y{0}, z{0};
};

struct PlannerConfig {
    // ===== Fixed / hard constraints =====
    double safetyMargin = 1000.0;       // meters (fixed in your requirement, but kept configurable)
    double turnRadius   = 5000.0;       // meters
    double step         = 1000.0;       // meters between waypoints
    double finalStraightLen = 2000.0;   // meters: from G_entry to goal
    double approachLenMinFactorR = 3.0; // approachLen >= factor * R
    double approachLenMinFactorStep = 5.0; // approachLen >= factor * step

    // ===== Obstacle inflation search to ensure "no kinks" with strict fillet =====
    int    maxInflateAttempts = 10;
    double inflateStepFactorR = 0.25;   // each attempt adds inflate += factor*R

    // ===== Altitude profile constraints =====
    double z_mid = 0.0;      // middle altitude
    double climbRate = 1.0;  // m/s
    double descendRate = 1.0;// m/s
    double groundSpeed = 1.0;// m/s (required to interpret rate limits)

    // ===== Behavior on infeasible strict-radius fillet =====
    // If true: fall back to a smooth spline (no kinks, but radius not guaranteed)
    // If false: return empty route and set status accordingly.
    bool allowSplineFallback = true;
};

enum class PlanStatus {
    SuccessStrictRadius,     // strict radius-R fillet succeeded everywhere
    SuccessSplineFallback,   // no kinks but radius not guaranteed (fallback used)
    InfeasibleFinalStraight, // mandatory A->G_entry->goal intersects inflated obstacles (hard constraint conflict)
    Failed                   // no route produced
};

struct PlanResult {
    PlanStatus status = PlanStatus::Failed;
    std::vector<Waypoint> route;  // output waypoints (x,y,z)
    // Optional debug: coarse polyline control points in world
    std::vector<Vec2> coarseWorld;
};

// dangerPolygonsWorld: each polygon is a list of XY vertices (vertical column obstacle)
PlanResult PlanRoute(
    const Pose3D& start,
    const Pose3D& goal,
    const std::vector<std::vector<Vec2>>& dangerPolygonsWorld,
    const PlannerConfig& cfg);

// Utility: write CSV (x,y,z) quickly
bool WriteRouteCsv(const char* filename, const std::vector<Waypoint>& route);

} // namespace routeplan
