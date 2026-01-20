#pragma once
#include "common/types.hpp"

namespace refuel {

// 每个“环节”都实现一个 Stage，输入输出都通过 PlanningContext 传递。
// 这样做的好处：
// 1) 主流程清晰：Pipeline 顺序调用各 Stage。
// 2) 环节之间接口固定：你可以随时替换内部算法实现，而不会影响其它模块。
// 3) 方便调试：每一步跑完都能 dump context 中的关键变量。
class IStage {
public:
  virtual ~IStage() = default;
  virtual void Run(PlanningContext& ctx) = 0;
};

} // namespace refuel
