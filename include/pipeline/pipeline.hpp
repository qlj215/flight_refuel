#pragma once
#include <memory>
#include <vector>
#include "common/types.hpp"
#include "stages/stage_base.hpp"

namespace refuel {

// Pipeline 负责把各 Stage 按流程图顺序串起来。
// 你可以自由替换 Stage 的实现（比如换一套跑马场选址算法、换 Hybrid A* 版本），
// 只要 Stage 的接口不变，主流程无需改动。
class Pipeline {
public:
  Pipeline();
  void Run(PlanningContext& ctx);

private:
  // 顺序严格对应流程图：坐标转换 -> 预处理 -> 安全区选择 -> 跑马场建立 -> 顺序 -> 会合规划 -> 航路 -> 代价
  std::vector<std::unique_ptr<IStage>> stages_;
};

} // namespace refuel
