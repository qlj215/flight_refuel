#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：判断模式 + 会合点/等待点规划（0/1/2/3 四种模式）
// 流程图位置：判断模式 -> 四个分支（0/1/2/3）
//
// 输入：
//   ctx.mission.refuel_mode.mode_type
//   ctx.sequence.sequence / receiver_to_order
//   ctx.racetrack（length/radius/entrypoints/center/speed/altitude）
//   ctx.tanker / receivers 的 cruise_perf / fuel_table / climb/descent rate 等
//
// 输出：
//   ctx.rendezvous：
//     - tanker_meeting_altitude_m / tanker_meeting_speed_mps / tanker_entry_point_xy / tanker_entry_direction_deg
//     - 每个 receiver 的 meeting_altitude/speed/coordinate/waiting_point/cycling_number/sequence
// 这些字段对应 demo/output/output.json 的 tanker/receivers 部分。
// ======================
class RendezvousStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;

private:
  // 新增：当 ctx.tanker_rt_decision.reduce_rendezvous=true 时，
  // 在“四个模式”的框架下，减少加油机会合相关部分（按最新流程图）。
  // 该函数只负责“tanker 侧”字段如何给默认/占位值；receiver 侧仍由各模式函数负责。
  void FillTankerRendezvousByPolicy(PlanningContext& ctx);

  void PlanMode0_BothSpeed(PlanningContext& ctx);
  void PlanMode1_TankerSpeed_ReceiverTimed(PlanningContext& ctx);
  void PlanMode2_TankerTimed_ReceiverSpeed(PlanningContext& ctx);
  void PlanMode3_BothTimed(PlanningContext& ctx);
};

} // namespace refuel
