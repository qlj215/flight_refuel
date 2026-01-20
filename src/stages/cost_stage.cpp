#include "stages/cost_stage.hpp"

#include <algorithm>

namespace refuel {

void CostStage::Run(PlanningContext& ctx) {
  // TODO：代价策略
  //
  // 你可能会在这里做：
  //   - 从轨迹点推时间：距离/速度积分，或用你规划器的时间戳
  //   - 从 fuel_table 推油耗：分段积分/查表
  //   - 计算总到达时间（例如所有 receiver 里最大到达时间）
  //   - 计算总油耗（例如所有 receiver 油耗求和）
  //
  // 这里只保留字段并给出最基础的汇总逻辑。

  ctx.cost = {};

  // 占位：tanker
  ctx.cost.tanker_arrival_time_s = 0.0; // TODO
  ctx.cost.tanker_fuel_consumed  = 0.0; // TODO

  // receivers
  double max_arrival = 0.0;
  double sum_fuel = 0.0;
  for (const auto& r : ctx.receivers) {
    ctx.cost.receiver_arrival_time_s[r.id] = 0.0; // TODO
    ctx.cost.receiver_fuel_consumed[r.id]  = 0.0; // TODO
    max_arrival = std::max(max_arrival, ctx.cost.receiver_arrival_time_s[r.id]);
    sum_fuel += ctx.cost.receiver_fuel_consumed[r.id];
  }

  ctx.cost.receiver_total_arrival_time_s = max_arrival;
  ctx.cost.receiver_total_fuel_consumed  = sum_fuel;
}

} // namespace refuel
