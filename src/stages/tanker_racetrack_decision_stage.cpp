#include "stages/tanker_racetrack_decision_stage.hpp"

#include "stages/sequence_stage.hpp" // 复用 IsTankerOnRacetrack 的几何判断接口

namespace refuel {

void TankerRacetrackDecisionStage::Run(PlanningContext& ctx) {
  // 清空旧结果（保证每次 Run 都从“干净状态”开始）
  ctx.tanker_rt_decision = {};

  // 1) 判断加油机初始位置是否在跑马场上
  //    注意：这里“在跑马场上”的判据由你实现（见 RacetrackBuildStage::IsTankerOnRacetrack）。
  ctx.tanker_rt_decision.tanker_on_racetrack =
      SequenceStage::IsTankerOnRacetrack(ctx.tanker.initial_position_xy, ctx.racetrack);

  // 2) 若不在跑马场上：后续按“正常进场/会合规划”走即可
  if (!ctx.tanker_rt_decision.tanker_on_racetrack) {
    ctx.tanker_rt_decision.need_transfer = true;   // 语义上等价于“需要进场/转场到跑马场”
    ctx.tanker_rt_decision.reduce_rendezvous = false;
    return;
  }

  // 3) 若已在跑马场上：进一步判断是否仍需要“转场”
  //    这里的“转场”定义由你们工程决定（可能是换跑马场/重新进场/改变跑马场几何等）。
  ctx.tanker_rt_decision.need_transfer = DetermineNeedTransfer(ctx);

  // 4) 若无需转场：仍然使用“下方四个模式”的整体框架，但可以减少加油机会合相关计算
  //    典型：tanker_entry_point 直接取当前点，tanker_entry_direction 直接取当前航向，
  //         不再做 entrypoints 选择、场外进场段求解等。
  ctx.tanker_rt_decision.reduce_rendezvous = !ctx.tanker_rt_decision.need_transfer;
}

bool TankerRacetrackDecisionStage::DetermineNeedTransfer(const PlanningContext& ctx) {
  (void)ctx;
  // TODO：按你的定义实现。
  // 默认：认为“在跑马场上就不需要转场”（最小改动，便于你逐步接入真实逻辑）
  return false;
}

} // namespace refuel
