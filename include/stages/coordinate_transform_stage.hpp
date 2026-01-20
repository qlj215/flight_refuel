#pragma once
#include "stages/stage_base.hpp"

namespace refuel {

// ======================
// 环节：坐标转换
// 流程图位置：读取输入 -> 坐标转换
//
// 输入：
//   ctx.mission.safe_zone.zones_vertices_lla（lat/lon）
//   ctx.tanker.initial_position_lla
//   ctx.receivers[i].initial_position_lla
//
// 输出：
//   ctx.tanker.initial_position_xy
//   ctx.receivers[i].initial_position_xy
//   ctx.safe_zone_selected.safe_zone_polygon_xy （如果你选择在此阶段就转换安全区坐标）
//
// 注意：
//   demo/output 的坐标数值较大（如 13285000），说明你们项目可能用了某种投影/平面坐标系。
//   这里不规定实现方式，只要求：最终输出的坐标系与既有工程一致。
// ======================
class CoordinateTransformStage final : public IStage {
public:
  void Run(PlanningContext& ctx) override;
};

} // namespace refuel
