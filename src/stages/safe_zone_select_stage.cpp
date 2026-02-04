#include "stages/safe_zone_select_stage.hpp"

#include <cmath>

namespace refuel {

namespace {

// TODO：使用与你项目一致的坐标转换（建议复用 CoordinateTransformStage 的实现）
static Vec3 LLA2XY(const LLA& lla) {
  constexpr double R = 6378137.0;
  const double lat = lla.lat_deg * M_PI / 180.0;
  const double lon = lla.lon_deg * M_PI / 180.0;
  Vec3 xy;
  xy.x = R * lon;
  xy.y = R * std::log(std::tan(M_PI/4.0 + lat/2.0));
  xy.z = lla.alt_m;
  return xy;
}


} // namespace

void SafeZoneSelectStage::Run(PlanningContext& ctx) {
  const auto& safe = ctx.mission.refuel_mode.safe_zone;

  if (!safe.is_defined || safe.zones_vertices_lla.empty()) {
    // 未定义安全区：保持默认（chosen_safezone_id=-1，polygon 为空）
    ctx.safe_zone_selected = {};
    ctx.safe_zone_selected.chosen_safezone_id = -1;
    return;
  }

  // TODO：根据 ctx.mission.prefs.primary_pref、tanker/receiver 初始位置等选择安全区。
  // demo/mission4.json 只有一个安全区，所以这里默认选 0 号。
  const int chosen = 0;
  ctx.safe_zone_selected.chosen_safezone_id = chosen + 1; // safezone_id 从 1 开始更直观

  // 将选中安全区顶点从 LLA 转为 XY
  Polygon poly;
  for (const auto& v : safe.zones_vertices_lla.at(chosen)) {
    poly.vertices_xy.push_back(LLA2XY(v));
  }
  ctx.safe_zone_selected.safe_zone_polygon_xy = std::move(poly);
}

} // namespace refuel
