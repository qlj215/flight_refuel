#include "stages/cost_stage.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace refuel {

namespace {

// =========================
// 代价解算的基本约定（非常重要）
// =========================
// 1) “时间”计算：
//    - 使用轨迹点序列（WaypointXYZ）累加距离（默认用 3D 距离，包含爬升/下降的 z）。
//    - 用固定速度（会合速度）将距离换算成时间： dt = ds / v。
//
//    注意：这是一种“工程上可控、可解释”的简化模型。
//    - 不区分转弯/加减速等动态过程；
//    - 不使用 route_planner 内部的时间戳（当前 skeleton 也没有输出时间戳）。
//
// 2) “油耗”计算：
//    - 通过 fuel_consumption_table（高度×速度→油耗率）查得油耗率；
//    - 按 segment 的平均高度（(z_i+z_{i+1})/2）作为查表高度；
//    - 油耗 = Σ( dt * fuel_rate )。
//
// 3) 受油机：
//    - 只计算到“会合”为止（包括盘旋）。
//    - PathPlanningStage 已把：转场 → 盘旋圈 → 从等待点到会合点 的点位全部写进 trajectory，
//      因此这里直接积分完整轨迹即可。
//
// 4) 加油机：
//    - 计算到“与最后一架受油机会合”为止。
//    - 加油机进入跑马场后速度固定为 130 m/s（这是本次需求的硬约束）。
//    - 进入跑马场前：使用 ctx.rendezvous.tanker_meeting_speed_mps 作为飞行速度（若无则 fallback）。
//
// 5) fuel table 单位：
//    - demo 输入表的数值量级在 4k~11k，常见含义是 kg/h（每小时耗油量）。
//    - 因为项目未在 types.hpp 中硬编码单位，这里做一个明确的“假设开关”：
//      若你的表实际是 kg/s，把 kAssumeFuelTableIsKgPerHour 改为 false 即可。
//
static constexpr bool   kAssumeFuelTableIsKgPerHour = true;
static constexpr double kTankerOnTrackSpeedMps      = 130.0; // 进入跑马场后的固定速度

static inline double Dist3D(const WaypointXYZ& a, const WaypointXYZ& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static inline bool IsFinite(double v) {
  return std::isfinite(v);
}

static inline double Clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// 在有序数组 levels 中，找到使 levels[i0] <= x <= levels[i1] 的一对索引，并给出插值参数 t∈[0,1]
// - 若 x 在范围外：做 clamp，i0=i1
static inline void FindBracketAndT(const std::vector<double>& levels,
                                   double x,
                                   std::size_t* i0,
                                   std::size_t* i1,
                                   double* t) {
  *i0 = 0;
  *i1 = 0;
  *t = 0.0;
  if (levels.empty()) return;

  if (x <= levels.front()) {
    *i0 = 0;
    *i1 = 0;
    *t = 0.0;
    return;
  }
  if (x >= levels.back()) {
    *i0 = levels.size() - 1;
    *i1 = levels.size() - 1;
    *t = 0.0;
    return;
  }

  // levels 严格递增（输入表满足该特性；若不满足，这里也能“基本工作”，但精度会受影响）
  const auto it = std::upper_bound(levels.begin(), levels.end(), x);
  const std::size_t hi = static_cast<std::size_t>(std::distance(levels.begin(), it));
  const std::size_t lo = (hi == 0) ? 0 : (hi - 1);

  *i0 = lo;
  *i1 = hi;

  const double a = levels[lo];
  const double b = levels[hi];
  const double den = (b - a);
  *t = (std::fabs(den) < 1e-12) ? 0.0 : Clamp((x - a) / den, 0.0, 1.0);
}

// fuel_table 双线性插值：按 (altitude, speed) 获取“油耗率”
// 返回值：
//   - 若 kAssumeFuelTableIsKgPerHour=true：返回 kg/s
//   - 否则：返回表中原单位（通常可认为是 kg/s）
static double LookupFuelRatePerSecond(const FuelConsumptionTable& ft,
                                      double altitude_m,
                                      double speed_mps) {
  // 表为空 / 维度不一致：直接返回 0，不让系统崩。
  if (ft.altitude_levels.empty() || ft.speed_levels.empty() || ft.consumption_data.empty()) return 0.0;

  // consumption_data 应为 [alt][speed]
  const std::size_t A = ft.altitude_levels.size();
  const std::size_t S = ft.speed_levels.size();
  if (ft.consumption_data.size() != A) return 0.0;
  for (const auto& row : ft.consumption_data) {
    if (row.size() != S) return 0.0;
  }

  // 允许 altitude/speed 不在表范围内：做 clamp
  std::size_t a0 = 0, a1 = 0;
  std::size_t s0 = 0, s1 = 0;
  double ta = 0.0, ts = 0.0;
  FindBracketAndT(ft.altitude_levels, altitude_m, &a0, &a1, &ta);
  FindBracketAndT(ft.speed_levels, speed_mps, &s0, &s1, &ts);

  const double f00 = ft.consumption_data[a0][s0];
  const double f01 = ft.consumption_data[a0][s1];
  const double f10 = ft.consumption_data[a1][s0];
  const double f11 = ft.consumption_data[a1][s1];

  // 双线性插值
  const double f0 = (1.0 - ts) * f00 + ts * f01;
  const double f1 = (1.0 - ts) * f10 + ts * f11;
  const double f  = (1.0 - ta) * f0  + ta * f1;

  if (!IsFinite(f) || f < 0.0) return 0.0;

  // 默认认为表是 kg/h，换算到 kg/s。
  return kAssumeFuelTableIsKgPerHour ? (f / 3600.0) : f;
}

static const Trajectory* FindTrajectoryById(const PlanningContext& ctx, const std::string& id) {
  for (const auto& t : ctx.trajectories) {
    if (t.aircraft_id == id) return &t;
  }
  return nullptr;
}

struct IntegratedCost {
  double time_s{0.0};
  double fuel_kg{0.0};
  double dist_m{0.0};
};

// 对一段轨迹（points[i] -> points[i+1]）做积分。
// - speed_mps 必须 > 0，否则返回 0。
// - altitude 用 segment 平均高度。
// - i_begin/i_end 是点索引区间：[i_begin, i_end]（包含端点），至少需要 2 个点。
static IntegratedCost IntegrateTrajectorySegment(const Trajectory& tr,
                                                 std::size_t i_begin,
                                                 std::size_t i_end,
                                                 double speed_mps,
                                                 const FuelConsumptionTable& ft,
                                                 double altitude_fallback_m) {
  IntegratedCost out;
  if (speed_mps <= 1e-9) return out;
  if (tr.points.size() < 2) return out;
  if (i_begin >= tr.points.size()) return out;
  if (i_end >= tr.points.size()) i_end = tr.points.size() - 1;
  if (i_end <= i_begin) return out;

  for (std::size_t i = i_begin; i < i_end; ++i) {
    const auto& a = tr.points[i];
    const auto& b = tr.points[i + 1];
    const double ds = Dist3D(a, b);
    if (!IsFinite(ds) || ds <= 1e-12) continue;

    const double dt = ds / speed_mps;
    if (!IsFinite(dt) || dt <= 0.0) continue;

    // 平均高度（若无效则使用 fallback）
    double alt = 0.5 * (a.z + b.z);
    if (!IsFinite(alt) || alt <= 0.0) alt = altitude_fallback_m;

    const double fr_kgps = LookupFuelRatePerSecond(ft, alt, speed_mps);

    out.dist_m += ds;
    out.time_s += dt;
    out.fuel_kg += dt * fr_kgps;
  }
  return out;
}

// 对加油机轨迹做“分段速度 + 截断时间”的积分。
// - entry_idx：认为从 entry_idx 开始进入跑马场（entry_idx 之前用 pre_speed，之后用 track_speed）。
// - stop_time_s：积分到该时间为止（可能会在一个 segment 内截断）。
static IntegratedCost IntegrateTankerWithStopTime(const Trajectory& tr,
                                                  std::size_t entry_idx,
                                                  double pre_speed_mps,
                                                  double track_speed_mps,
                                                  double stop_time_s,
                                                  const FuelConsumptionTable& ft,
                                                  double altitude_fallback_m) {
  IntegratedCost out;
  if (tr.points.size() < 2) return out;
  if (stop_time_s <= 0.0) return out;
  entry_idx = std::min(entry_idx, tr.points.size() - 1);

  double t_acc = 0.0;

  for (std::size_t i = 0; i + 1 < tr.points.size(); ++i) {
    const double v = (i < entry_idx) ? pre_speed_mps : track_speed_mps;
    if (v <= 1e-9) continue;

    const auto& a = tr.points[i];
    const auto& b = tr.points[i + 1];
    const double ds = Dist3D(a, b);
    if (!IsFinite(ds) || ds <= 1e-12) continue;

    const double dt_full = ds / v;
    if (!IsFinite(dt_full) || dt_full <= 0.0) continue;

    // 平均高度
    double alt = 0.5 * (a.z + b.z);
    if (!IsFinite(alt) || alt <= 0.0) alt = altitude_fallback_m;

    const double fr_kgps = LookupFuelRatePerSecond(ft, alt, v);

    // 是否需要在该段内截断
    if (t_acc + dt_full <= stop_time_s + 1e-12) {
      out.dist_m += ds;
      out.time_s += dt_full;
      out.fuel_kg += dt_full * fr_kgps;
      t_acc += dt_full;
    } else {
      const double remain = std::max(0.0, stop_time_s - t_acc);
      // 假设该段内匀速：截取 remain / dt_full 的距离比例
      const double ratio = Clamp(remain / dt_full, 0.0, 1.0);
      out.dist_m += ds * ratio;
      out.time_s += remain;
      out.fuel_kg += remain * fr_kgps;
      t_acc = stop_time_s;
      break;
    }
  }

  // out.time_s 可能略小于 stop_time_s（例如轨迹点不足导致跑不到 stop_time_s）。
  // 上层可按需要补齐。
  return out;
}

// 用 2D 距离在 tanker 轨迹上找“最接近 entry_point”的点索引，作为进入跑马场的分界。
static std::size_t FindEntryIndexOnTrajectory2D(const Trajectory& tr, const Vec3& entry_xy) {
  if (tr.points.empty()) return 0;
  std::size_t best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < tr.points.size(); ++i) {
    const double dx = tr.points[i].x - entry_xy.x;
    const double dy = tr.points[i].y - entry_xy.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

} // namespace

void CostStage::Run(PlanningContext& ctx) {
  // ---------------
  // 0) 清空旧结果
  // ---------------
  ctx.cost = {};

  // 如果没有轨迹点，本 stage 不做任何计算，但仍保证输出结构完整。
  // 这样 output.json 永远可写，且不会出现 map 缺 key 的情况。
  auto ensure_receiver_keys = [&]() {
    for (const auto& r : ctx.receivers) {
      ctx.cost.receiver_arrival_time_s.emplace(r.id, 0.0);
      ctx.cost.receiver_fuel_consumed.emplace(r.id, 0.0);
    }
  };

  // -------------------------
  // 1) 受油机：逐个积分到会合点
  // -------------------------
  double max_arrival_s = 0.0;
  double sum_fuel_kg   = 0.0;

  for (const auto& r : ctx.receivers) {
    const std::string& rid = r.id;

    // 会合速度：优先用 rendezvous 的 meeting_speed
    double v_mps = 0.0;
    auto it_rr = ctx.rendezvous.receivers.find(rid);
    if (it_rr != ctx.rendezvous.receivers.end() && it_rr->second.meeting_speed_mps > 1e-9) {
      v_mps = it_rr->second.meeting_speed_mps;
    } else if (r.current_status.speed_mps > 1e-9) {
      v_mps = r.current_status.speed_mps;
    } else {
      // 兜底：给一个保守的正值，避免除 0。
      v_mps = 150.0;
    }

    const Trajectory* tr = FindTrajectoryById(ctx, rid);
    if (!tr || tr->points.size() < 2) {
      // 没有轨迹：输出 0
      ctx.cost.receiver_arrival_time_s[rid] = 0.0;
      ctx.cost.receiver_fuel_consumed[rid]  = 0.0;
      continue;
    }

    // 高度 fallback：若轨迹点 z 不可信，用会合高度或 racetrack 高度。
    double alt_fallback = ctx.racetrack.altitude_m;
    if (it_rr != ctx.rendezvous.receivers.end() && it_rr->second.meeting_altitude_m > 1e-6) {
      alt_fallback = it_rr->second.meeting_altitude_m;
    } else if (r.initial_position_lla.alt_m > 1e-6) {
      alt_fallback = r.initial_position_lla.alt_m;
    }

    const IntegratedCost c =
        IntegrateTrajectorySegment(*tr, 0, tr->points.size() - 1, v_mps, r.fuel_table, alt_fallback);

    ctx.cost.receiver_arrival_time_s[rid] = c.time_s;
    ctx.cost.receiver_fuel_consumed[rid]  = c.fuel_kg;

    max_arrival_s = std::max(max_arrival_s, c.time_s);
    sum_fuel_kg   += c.fuel_kg;
  }

  ctx.cost.receiver_total_arrival_time_s = max_arrival_s;
  ctx.cost.receiver_total_fuel_consumed  = sum_fuel_kg;

  // -------------------------
  // 2) 加油机：计算到“最后一架受油机会合”为止
  // -------------------------
  // “最后一架受油机”的定义：
  // - 优先使用 rendezvous.receivers 中的 sequence_order 最大者；
  // - 若 sequence_order 不可用（=0 或缺失），退化为“到达时间最大的受油机”。
  std::string last_rid;
  int last_order = std::numeric_limits<int>::min();
  for (const auto& kv : ctx.rendezvous.receivers) {
    const int ord = kv.second.sequence_order;
    if (ord > last_order) {
      last_order = ord;
      last_rid = kv.first;
    }
  }

  double stop_time_s = 0.0;
  if (!last_rid.empty() && ctx.cost.receiver_arrival_time_s.count(last_rid)) {
    stop_time_s = ctx.cost.receiver_arrival_time_s.at(last_rid);
  }

  // 如果 order 信息不可用（例如全是 0），则采用“最大到达时间”作为加油机终止时间。
  if (stop_time_s <= 0.0) {
    stop_time_s = ctx.cost.receiver_total_arrival_time_s;
  }

  // 取加油机轨迹
  const Trajectory* tanker_tr = FindTrajectoryById(ctx, ctx.tanker.id);
  if (!tanker_tr || tanker_tr->points.size() < 2) {
    // 没有轨迹：保持 0，但仍保证 receiver keys 完整。
    ensure_receiver_keys();
    return;
  }

  // 进入跑马场前速度：优先使用 rendezvous 给出的速度（它通常是 mode0/mode1 求解的加油机定速）。
  double v_pre = ctx.rendezvous.tanker_meeting_speed_mps;
  if (v_pre <= 1e-9) v_pre = ctx.tanker.current_status.speed_mps;
  if (v_pre <= 1e-9) v_pre = kTankerOnTrackSpeedMps; // 最后兜底

  // 进入跑马场的分界点：用 entry_point 在轨迹上找最近点。
  Vec3 entry = ctx.rendezvous.tanker_entry_point_xy;
  // 兼容：如果 entry 未填（0,0），用跑马场第一个进入点兜底（与 PathPlanningStage 逻辑一致）。
  if (std::fabs(entry.x) < 1e-9 && std::fabs(entry.y) < 1e-9 && !ctx.racetrack.entrypoints_xy.empty()) {
    entry = ctx.racetrack.entrypoints_xy.front();
  }
  const std::size_t entry_idx = FindEntryIndexOnTrajectory2D(*tanker_tr, entry);

  // 高度 fallback：优先会合高度，再用跑马场高度。
  double tanker_alt_fallback = (ctx.rendezvous.tanker_meeting_altitude_m > 1e-6)
                                   ? ctx.rendezvous.tanker_meeting_altitude_m
                                   : ctx.racetrack.altitude_m;
  if (tanker_alt_fallback <= 1e-6) tanker_alt_fallback = ctx.tanker.initial_position_lla.alt_m;

  // 对加油机按 stop_time_s 截断积分
  IntegratedCost tc = IntegrateTankerWithStopTime(*tanker_tr,
                                                  entry_idx,
                                                  v_pre,
                                                  kTankerOnTrackSpeedMps,
                                                  stop_time_s,
                                                  ctx.tanker.fuel_table,
                                                  tanker_alt_fallback);

  // 若轨迹长度不足导致 tc.time_s < stop_time_s：
  // 这里做一个合理补齐：假设加油机继续以 130m/s 在跑马场上飞（不需要额外航路点也能补齐时间/油耗）。
  if (tc.time_s + 1e-9 < stop_time_s) {
    const double extra = stop_time_s - tc.time_s;
    const double fr_kgps =
        LookupFuelRatePerSecond(ctx.tanker.fuel_table, tanker_alt_fallback, kTankerOnTrackSpeedMps);
    tc.time_s = stop_time_s;
    tc.fuel_kg += extra * fr_kgps;
    // dist_m 不强求补齐（此时我们没有真实几何路径），保留现有值即可。
  }

  ctx.cost.tanker_arrival_time_s = tc.time_s;
  ctx.cost.tanker_fuel_consumed  = tc.fuel_kg;

  // 确保：即使前面因为缺轨迹导致部分 receiver 未写入，也能补齐 key。
  ensure_receiver_keys();
}

} // namespace refuel
