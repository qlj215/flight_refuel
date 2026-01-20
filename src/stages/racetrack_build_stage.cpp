#include "stages/racetrack_build_stage.hpp"

namespace refuel {

void RacetrackBuildStage::Run(PlanningContext& ctx) {
  // TODO：根据安全区约束 + 任务参数建立跑马场（racetrack）
  //
  // 你需要至少填出这些关键量（流程图“输出”框）：
  //   ctx.racetrack.length_m
  //   ctx.racetrack.radius_m
  //   ctx.racetrack.center_xy
  //   ctx.racetrack.orientation_deg
  //   ctx.racetrack.entrypoints_xy
  //   ctx.racetrack.altitude_m
  //   ctx.racetrack.speed_mps
  //
  // 说明：
  // - mission4.json 里 racecourse.orbit_radius/entry_angle 提供了部分几何参数
  // - altitude/speed 可能来自 holding_zone/性能限制/优化策略等（demo/output 里 altitude=7000, speed=130）
  // - entrypoints_xy 通常是跑马场进入点（多机序列规划与会合点计算会用到）

  ctx.racetrack = {}; // 清空旧结果

  // ====== “仅用于跑通流程”的默认值（你应替换为真实算法）======
  ctx.racetrack.radius_m = ctx.mission.racecourse.orbit_radius; // demo 输入给的是 5000
  ctx.racetrack.length_m = 0.0;                                 // TODO: 由直线段长度+转弯半径等计算
  ctx.racetrack.orientation_deg = 0.0;                          // TODO: 由安全区/任务方向决定
  ctx.racetrack.altitude_m = 7000.0;                            // TODO: 来自约束或优化
  ctx.racetrack.speed_mps = 130.0;                              // TODO: 来自约束或优化
  ctx.racetrack.center_xy = {};                                 // TODO: 计算 racetrack_center
  ctx.racetrack.entrypoints_xy.clear();                         // TODO: 计算 entrypoints
}

} // namespace refuel
