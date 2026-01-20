#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：航路规划
// 流程图位置：会合点规划 -> 航路规划 -> way points(csv)
//
// 输入：
//   ctx.tanker.initial_position_xy / receivers[i].initial_position_xy
//   ctx.racetrack
//   ctx.rendezvous（会合点/进入点/等待点等）
//
// 输出：
//   ctx.trajectories：每架飞机一条 Trajectory，最终写入 demo/output/tanker01.csv, receiver01.csv...
//
// 注意：demo/output/*.csv 只有 x,y,z 三列，没有时间戳。
// 你们如果内部规划是带时间的，可以在此阶段做一次“采样/离散化”输出。
// ======================
class PathPlanningStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;
};

} // namespace refuel
