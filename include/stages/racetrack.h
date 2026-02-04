#pragma once
#include <vector>
#include <string>
#include <cstdint>

namespace racetrack {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

enum class ModelType {
  PreferTanker = -1,
  Balanced0    = 0,
  Balanced3    = 3,
  PreferReceiver = 2
};

struct Inputs {
  // Safe zone polygon vertices (meters, x=east, y=north). Must form a simple polygon.
  std::vector<Vec2> safe_polygon;

  // Tanker position (meters)
  Vec2 tanker;

  // Receiver positions (meters)
  std::vector<Vec2> receivers;

  // Mode preference
  ModelType model_type = ModelType::Balanced0;

  // Expected mission time in UTC seconds since Unix epoch
  std::int64_t unix_time_utc = 0;

  // Initial track params from your empirical formula (meters)
  double L0 = 0.0; // straight segment length
  double R0 = 0.0; // semicircle radius
};

struct Outputs {
  double L = 0.0;
  double R = 0.0;
  Vec2 center;
  double heading_deg = 0.0; // angle w.r.t +x (east), CCW positive
};

enum class Status {
  Ok = 0,
  InvalidInput,
  NoFeasibleSolution
};

struct Config {
  // Earth radius used by inverse transform (x,y)->(lat,lon). Must match your forward mapping model.
  double earth_radius_m = 6378137.0;

  // Rectangle orientation candidates: default 12 directions, 30-degree step (0..330)
  int rect_orientation_count = 12;
  double rect_orientation_step_deg = 30.0;

  // Candidate center points in feasible window (structure points + Halton points)
  int halton_points = 60;         // default 60 (total about 70)
  bool add_structure_points = true;

  // Polylabel-like center search precision (meters)
  double polylabel_precision_m = 0.5;

  // Stadium containment tolerance (meters)
  double contain_eps_m = 1e-6;

  // Orientation try sequence around sun-perpendicular
  // Default: [0, +10, -10, +20, -20], then rectangle heading fallback
  std::vector<double> orientation_offsets_deg = {0.0, 10.0, -10.0, 20.0, -20.0};

  // If you need a minimum turning radius, set it here (<=0 means disabled)
  double min_radius_m = -1.0;

  // Allow shrinking to fit rectangle quickly (enabled)
  bool shrink_to_fit_rect = true;
};

struct Result {
  Status status = Status::InvalidInput;
  Outputs out;
  std::string message; // for logging / debugging
};

// Main entry. No IO. Thread-safe if you don't use global state.
Result PlanRacetrack(const Inputs& in, const Config& cfg = Config{});

} // namespace racetrack
