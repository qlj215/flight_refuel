#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：加油机是否在跑马场上 + 是否需要转场 + 是否减少会合相关计算
// 流程图位置（最新）：
//   确定加油顺序 -> 加油机是否在跑马场上 -> 是否需要转场 -> (同下方四个模式 / 减少会合部分) -> 判断模式 -> 航路规划
//
// 为什么要单独做成一个 Stage？
// - 这是一个“轻量判断”，它本质上只是给后续 RendezvousStage / PathPlanningStage 提供分支开关，
//   不应该把大量几何判断/策略判断散落在多个 Stage 里。
// - 保持接口清晰：后续如果你改变“需要转场”的判据，只改这一处即可。
//
// 输入：
//   ctx.tanker.initial_position_xy
//   ctx.tanker.current_status.heading_deg
//   ctx.racetrack（center/entrypoints/几何参数）
//   ctx.sequence（有些工程会用“下一架受油机/会合顺序”决定是否转场）
//
// 输出：
//   ctx.tanker_rt_decision:
//     - tanker_on_racetrack
//     - need_transfer
//     - reduce_rendezvous
// ======================
class TankerRacetrackDecisionStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;

private:
  // TODO：由你的工程定义“是否需要转场”的判断逻辑。
  // 常见判据示例（仅供参考，具体以你们定义为准）：
  //   - 当前加油机所在跑马场 != 本次规划生成的跑马场（例如 center/altitude/speed 不一致）
  //   - 或者：按 sequence 计算的下一次会合点不在当前跑马场可达范围内，需要重新进场
  static bool DetermineNeedTransfer(const PlanningContext& ctx);
};

} // namespace refuel
