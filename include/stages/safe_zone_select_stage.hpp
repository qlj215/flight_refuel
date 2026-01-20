#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：是否划分安全区 + 安全区选择
// 流程图位置：滤波预处理 -> 是否划分安全区? -> 安全区选择
//
// 输入：
//   ctx.mission.refuel_mode.safe_zone.is_defined
//   ctx.mission.refuel_mode.safe_zone.zones_vertices_lla
//   ctx.mission.prefs.primary_pref
//   ctx.tanker.initial_position_xy
//   ctx.receivers[i].initial_position_xy
//
// 输出：
//   ctx.safe_zone_selected.safe_zone_polygon_xy
//   ctx.safe_zone_selected.chosen_safezone_id
//
// 备注：
//   - 如果 safe_zone_is_defined=false，则可以让 chosen_safezone_id=-1 且 polygon 为空
//   - 这个阶段只需要“选中哪个安全区”，不必做跑马场几何约束。
// ======================
class SafeZoneSelectStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;
};

} // namespace refuel
