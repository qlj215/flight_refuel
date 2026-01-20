#include "stages/preprocess_stage.hpp"

namespace refuel {

void PreprocessStage::Run(PlanningContext& ctx) {
  // TODO：这里做“滤波、油耗预处理”，以及从表格中提取速度边界。
  //
  // 典型输出（流程图底部“输出”框）：
  //   ctx.receiver_speed_bounds["RE001"] = {min, cruise, max}
  //   ctx.tanker_speed_bounds            = {min, cruise, max}
  //
  // 你可以从下面字段里取数据：
  //   - ctx.tanker.cruise_perf.altitude_levels / speed_levels / true_airspeed_data
  //   - ctx.tanker.current_status.speed_mps
  //   - ctx.tanker.limits (climb_rate/descent_rate)
  // receivers 同理

  // 先填一个“保底默认值”，方便主流程跑通
  ctx.tanker_speed_bounds = {120.0, 150.0, 300.0};
  for (const auto& r : ctx.receivers) {
    ctx.receiver_speed_bounds[r.id] = {120.0, 170.0, 300.0};
  }
}

} // namespace refuel
