#include "stages/safe_zone_select_stage.hpp"

#include <cmath>

namespace refuel {

namespace {

// TODO：使用与你项目一致的坐标转换（建议复用 CoordinateTransformStage 的实现）
static Vec3 LLA2XY_Stub(const LLA& lla) {
  Vec3 xy{};
  (void)lla;
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
    poly.vertices_xy.push_back(LLA2XY_Stub(v));
  }
  ctx.safe_zone_selected.safe_zone_polygon_xy = std::move(poly);
}

} // namespace refuel
