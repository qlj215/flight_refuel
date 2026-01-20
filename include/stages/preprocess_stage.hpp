#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：滤波、油耗预处理（以及速度边界提取）
// 流程图位置：坐标转换 -> 滤波、油耗预处理
//
// 目标（流程图的“输出”框）：
//   - 计算每架飞机的 min/cruise/max speed（写入 ctx.tanker_speed_bounds / ctx.receiver_speed_bounds）
//   - （可选）对 performance_table / fuel_table 做预处理（插值结构、缓存等）
//
// 输入：
//   ctx.tanker.cruise_perf / ctx.tanker.fuel_table / ctx.tanker.limits / current_status ...
//   ctx.receivers[i].cruise_perf / fuel_table / limits / current_status ...
//
// 输出：
//   ctx.tanker_speed_bounds
//   ctx.receiver_speed_bounds[receiver_id]
// ======================
class PreprocessStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;
};

} // namespace refuel
