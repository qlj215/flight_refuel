#include "stages/safe_zone_select_stage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace refuel {
namespace {

static Vec3 LLA2XY(const LLA& lla) {
  constexpr double R = 6378137.0;
  const double lat = lla.lat_deg * M_PI / 180.0;
  const double lon = lla.lon_deg * M_PI / 180.0;
  Vec3 xy;
  xy.x = R * lon;
  xy.y = R * std::log(std::tan(M_PI / 4.0 + lat / 2.0));
  xy.z = lla.alt_m;
  return xy;
}

static Vec3 ComputeDemandCentroid(const PlanningContext& ctx) {
  Vec3 c = ctx.tanker.initial_position_xy;
  std::size_t cnt = 1;
  for (const auto& r : ctx.receivers) {
    c.x += r.initial_position_xy.x;
    c.y += r.initial_position_xy.y;
    ++cnt;
  }
  if (cnt > 0) {
    c.x /= static_cast<double>(cnt);
    c.y /= static_cast<double>(cnt);
  }
  return c;
}

static Vec3 PolygonCentroid(const Polygon& poly) {
  Vec3 c{};
  if (poly.vertices_xy.empty()) return c;
  for (const auto& p : poly.vertices_xy) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= static_cast<double>(poly.vertices_xy.size());
  c.y /= static_cast<double>(poly.vertices_xy.size());
  return c;
}

static double Dist2XY(const Vec3& a, const Vec3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

} // namespace

void SafeZoneSelectStage::Run(PlanningContext& ctx) {
  const auto& safe = ctx.mission.refuel_mode.safe_zone;
  ctx.safe_zone_selected = {};
  ctx.safe_zone_selected.chosen_safezone_id = -1;

  if (!safe.is_defined || safe.zones_vertices_lla.empty()) {
    return;
  }

  const std::unordered_set<int> excluded(ctx.excluded_safezone_ids.begin(),
                                         ctx.excluded_safezone_ids.end());
  const Vec3 demand_centroid = ComputeDemandCentroid(ctx);

  double best_score = std::numeric_limits<double>::infinity();
  int best_idx = -1;
  Polygon best_poly;
  int best_id = -1;

  for (std::size_t i = 0; i < safe.zones_vertices_lla.size(); ++i) {
    const int zone_id = (i < safe.zone_ids.size() ? safe.zone_ids[i] : static_cast<int>(i) + 1);
    if (excluded.count(zone_id)) continue;

    Polygon poly;
    for (const auto& v : safe.zones_vertices_lla[i]) {
      poly.vertices_xy.push_back(LLA2XY(v));
    }
    if (poly.vertices_xy.empty()) continue;

    const double score = Dist2XY(PolygonCentroid(poly), demand_centroid);
    if (score < best_score) {
      best_score = score;
      best_idx = static_cast<int>(i);
      best_poly = std::move(poly);
      best_id = zone_id;
    }
  }

  if (best_idx < 0) {
    return;
  }

  ctx.safe_zone_selected.chosen_safezone_id = best_id;
  ctx.safe_zone_selected.safe_zone_polygon_xy = std::move(best_poly);
}

} // namespace refuel
