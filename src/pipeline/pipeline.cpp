#include "pipeline/pipeline.hpp"

// 具体 Stage
#include "stages/coordinate_transform_stage.hpp"
#include "stages/preprocess_stage.hpp"
#include "stages/safe_zone_select_stage.hpp"
#include "stages/racetrack_build_stage.hpp"
#include "stages/sequence_stage.hpp"
#include "stages/tanker_racetrack_decision_stage.hpp"
#include "stages/rendezvous_stage.hpp"
#include "stages/path_planning_stage.hpp"
#include "stages/cost_stage.hpp"

namespace refuel {

Pipeline::Pipeline() {
  stages_.emplace_back(std::make_unique<CoordinateTransformStage>());
  stages_.emplace_back(std::make_unique<PreprocessStage>());
  stages_.emplace_back(std::make_unique<SafeZoneSelectStage>());
  stages_.emplace_back(std::make_unique<RacetrackBuildStage>());
  stages_.emplace_back(std::make_unique<SequenceStage>());
  // 新增：对应最新流程图
  //   确定加油顺序 -> 加油机是否在跑马场上 -> 是否需要转场 -> (同下方四模式 / 减少会合部分)
  // 该 Stage 只产生分支开关，不做重算法。
  stages_.emplace_back(std::make_unique<TankerRacetrackDecisionStage>());
  stages_.emplace_back(std::make_unique<RendezvousStage>());
  stages_.emplace_back(std::make_unique<PathPlanningStage>());
  stages_.emplace_back(std::make_unique<CostStage>());
}

void Pipeline::Run(PlanningContext& ctx) {
  // Stage index mapping (must match Pipeline::Pipeline() order)
  constexpr std::size_t kSafe    = 2;
  constexpr std::size_t kRtBuild = 3;
  constexpr std::size_t kSeq     = 4;
  constexpr std::size_t kTanker  = 5;
  constexpr std::size_t kRendez  = 6;

  // 1) Run up to (and including) “加油机是否在跑马场上/是否需要转移”判定
  for (std::size_t i = 0; i < stages_.size(); ++i) {
    stages_[i]->Run(ctx);

    if (i == kTanker) {
      // 2) 若“在跑马场上且需要转移”，则按新流程回退到
      //    “不在跑马场 -> 是否划分安全区 -> 跑马场建立 -> 确定加油顺序”
      if (ctx.tanker_rt_decision.tanker_on_racetrack && ctx.tanker_rt_decision.need_transfer) {
        stages_[kSafe]->Run(ctx);
        stages_[kRtBuild]->Run(ctx);
        stages_[kSeq]->Run(ctx);

        // 3) 回退后不再做第二次“是否在跑马场”判断
        //    直接视为“不在跑马场/不启用减会合策略”
        ctx.tanker_rt_decision.tanker_on_racetrack = false;
        ctx.tanker_rt_decision.reduce_rendezvous = false;
      }

      // 4) 从 Rendezvous 起点继续跑（避免再次跑 Safe/Racetrack/Seq/Tanker 判断）
      for (std::size_t j = kRendez; j < stages_.size(); ++j) {
        stages_[j]->Run(ctx);
      }
      return;
    }
  }
}

} // namespace refuel
