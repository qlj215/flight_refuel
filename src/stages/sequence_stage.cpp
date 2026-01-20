#include "stages/sequence_stage.hpp"

#include <algorithm>

namespace refuel {

void SequenceStage::Run(PlanningContext& ctx) {
  // TODO：实现“确定加油顺序”策略
  //
  // 常见输入：
  //   - receiver 初始位置（ctx.receivers[i].initial_position_xy）
  //   - racetrack_center（ctx.racetrack.center_xy）
  //   - priority/primary_pref 等（ctx.mission.prefs.primary_pref）
  //
  // 常见输出：
  //   - ctx.sequence.sequence：按顺序给出 receiver 的索引（0-based）
  //   - ctx.sequence.receiver_to_order：receiver_id -> 1/2/3...

  ctx.sequence.sequence.clear();
  ctx.sequence.receiver_to_order.clear();

  // ====== 默认策略：按 receivers 在配置文件出现的顺序（仅用于跑通流程）======
  for (int i = 0; i < static_cast<int>(ctx.receivers.size()); ++i) {
    ctx.sequence.sequence.push_back(i);
    ctx.sequence.receiver_to_order[ctx.receivers[i].id] = i + 1;
  }

  // 如果你需要示例 [1,3,2] 这种顺序，可以在这里手动改：
  // ctx.sequence.sequence = {0, 2, 1}; // 0-based 对应 [1,3,2]
}

bool SequenceStage::IsTankerOnRacetrack(const Vec3& tanker_pos_xy, const RacetrackPlan& rt) {
  (void)tanker_pos_xy;
  (void)rt;
  // TODO：判断点是否位于跑马场（或跑马场容差范围内）
  return false;
}

} // namespace refuel
