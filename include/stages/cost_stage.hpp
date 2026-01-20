#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：代价策略
// 流程图位置：航路规划 -> 代价策略 -> 输出（output.json）
//
// 输入：
//   ctx.trajectories
//   ctx.tanker / receivers 的 fuel_table（用于油耗计算）
//   ctx.mission.prefs.weight_factors（若代价需要加权）
//
// 输出：
//   ctx.cost：
//     tanker_arrival_time_s / tanker_fuel_consumed
//     receiver_arrival_time_s / receiver_fuel_consumed
//     receiver_total_arrival_time_s / receiver_total_fuel_consumed
//
// 对应 demo/output/output.json 的 tanker/receivers/receiver_summary 部分。
// ======================
class CostStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;
};

} // namespace refuel
