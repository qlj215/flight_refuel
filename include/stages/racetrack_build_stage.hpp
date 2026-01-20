#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：跑马场建立（考虑安全区限制）
// 流程图位置：安全区选择 -> 跑马场建立（考虑安全区限制）
//
// 输入：
//   ctx.safe_zone_selected.safe_zone_polygon_xy（可能为空）
//   ctx.mission.racecourse.orbit_radius / entry_angle 等（以及你们项目的其他参数）
//   ctx.tanker.initial_position_xy
//   ctx.receivers[i].initial_position_xy
//
// 输出（流程图“输出”框）：
//   ctx.racetrack.length_m / radius_m / center_xy / orientation_deg / entrypoints_xy
//   以及 altitude_m / speed_mps（你可以由任务限制或优化策略给出）
// ======================
class RacetrackBuildStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;
};

} // namespace refuel
