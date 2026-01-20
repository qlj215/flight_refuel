#include "stages/coordinate_transform_stage.hpp"

#include <cmath>

namespace refuel {

namespace {

// TODO(你来实现)：经纬度 -> 平面坐标（x,y）
// 你们 demo/output 的量级非常像 WebMercator(EPSG:3857)：
//   x = R * lon_rad
//   y = R * ln( tan(pi/4 + lat_rad/2) )
// 其中 R = 6378137
//
// 但这里不做任何假设，先留空。
// 你可以：
// 1) 直接替换为你项目已有的坐标转换；或
// 2) 采用 WebMercator/UTM/ENU 等任意一致坐标系。
static Vec3 LLA2XY_Stub(const LLA& lla) {
  Vec3 xy{};
  (void)lla;

  // ===== 示例（若你确定就是 WebMercator，可以取消注释）=====
  // constexpr double R = 6378137.0;
  // const double lat = lla.lat_deg * M_PI / 180.0;
  // const double lon = lla.lon_deg * M_PI / 180.0;
  // xy.x = R * lon;
  // xy.y = R * std::log(std::tan(M_PI/4.0 + lat/2.0));
  // xy.z = lla.alt_m;
  // =========================================================

  return xy;
}

} // namespace

void CoordinateTransformStage::Run(PlanningContext& ctx) {
  // 1) tanker 初始位置
  ctx.tanker.initial_position_xy = LLA2XY_Stub(ctx.tanker.initial_position_lla);

  // 2) receivers 初始位置
  for (auto& r : ctx.receivers) {
    r.initial_position_xy = LLA2XY_Stub(r.initial_position_lla);
  }

  // 3) 安全区顶点坐标转换（这里“只演示接口”）
  // 你可以选择：
  //   A) 在 SafeZoneSelectStage 中再做转换；或
  //   B) 在这里把所有 safe_zone 顶点都转好，供后续选择/约束使用。
  //
  // 目前留空：ctx.safe_zone_selected 在 SafeZoneSelectStage 再填。
}

} // namespace refuel
