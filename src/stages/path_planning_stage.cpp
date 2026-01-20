#include "stages/path_planning_stage.hpp"

namespace refuel {

void PathPlanningStage::Run(PlanningContext& ctx) {
  // TODO：这里接入你的航路规划器（例如 Hybrid A*）
  //
  // 输入一般包括：
  //   - 初始位置：ctx.tanker.initial_position_xy / ctx.receivers[i].initial_position_xy
  //   - 目标点：  ctx.rendezvous 中的 entry_point/meeting_point/waiting_point 等
  //   - 约束：    安全区、禁飞区、航向/转弯半径、爬升率/下降率
  //   - 新增分支开关（最新流程图）：
  //       ctx.tanker_rt_decision.tanker_on_racetrack
  //       ctx.tanker_rt_decision.need_transfer
  //       ctx.tanker_rt_decision.reduce_rendezvous
  //     典型用法：
  //       - 若 reduce_rendezvous=true：加油机已经在目标跑马场上且无需转场，可省略“进场/转场段”
  //       - 若 need_transfer=true：需要规划从当前点到跑马场 entry_point（或新跑马场）的转场轨迹
  //
  // 输出：
  //   ctx.trajectories：每架飞机一条 Trajectory（points 为离散点 x,y,z）

  ctx.trajectories.clear();

  // ====== 占位输出：仅把初始点写出去，保证 csv 不为空（便于你调试 I/O）======
  {
    Trajectory t;
    t.aircraft_id = ctx.tanker.id;
    t.points.push_back({ctx.tanker.initial_position_xy.x, ctx.tanker.initial_position_xy.y, ctx.tanker.initial_position_lla.alt_m});
    ctx.trajectories.push_back(std::move(t));
  }

  for (const auto& r : ctx.receivers) {
    Trajectory t;
    t.aircraft_id = r.id;
    t.points.push_back({r.initial_position_xy.x, r.initial_position_xy.y, r.initial_position_lla.alt_m});
    ctx.trajectories.push_back(std::move(t));
  }
}

} // namespace refuel
