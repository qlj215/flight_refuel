#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：确定加油顺序
// 流程图位置：确定加油顺序 -> 判断加油机是否在跑马场上?
//
// 输入：
//   ctx.racetrack.center_xy（流程图红字 racetrack_center）
//   ctx.receivers[i].initial_position_xy
//   ctx.mission.prefs.primary_pref（以及你们定义的优先级/规则）
//
// 输出：
//   ctx.sequence.sequence
//   ctx.sequence.receiver_to_order
//
// 备注：
//   - 流程图里 receiver 的“priority”来自输入（demo/mission4.json 的 receiver_id 只给了 ID，没有显式 priority）
//     你们工程可能在 receiver_config 或 mission 中另有字段；这里不做强制。
// ======================
class SequenceStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;

  // 供 Pipeline/Sequence 使用：判断 tanker 初始位置是否在跑马场上
  // （实现可以先空着，返回 false 也行）
  static bool IsTankerOnRacetrack(const Vec3& tanker_pos_xy, const RacetrackPlan& rt);
};

} // namespace refuel
