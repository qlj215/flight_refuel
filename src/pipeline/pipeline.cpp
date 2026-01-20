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
  for (auto& stage : stages_) {
    stage->Run(ctx);
  }
}

} // namespace refuel
