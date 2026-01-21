#include "stages/rendezvous_stage.hpp"

// mode_type = 0 的“白框流程”接口骨架（只提供接口 + 注释，不含具体算法实现）
#include "mode0/mode0_fixed_speed_search.hpp"
#include "mode1/mode1_fixed_speed_search.hpp"

namespace refuel {

void RendezvousStage::Run(PlanningContext& ctx) {
  // 约定：每次进入该 Stage 都先清空旧结果，避免上一次运行的残留影响输出。
  ctx.rendezvous = {};

  const int mode = ctx.mission.refuel_mode.mode_type;
  switch (mode) {
    case 0:
      PlanMode0_BothSpeed(ctx);
      break;
    case 1:
      PlanMode1_TankerSpeed_ReceiverTimed(ctx);
      break;
    case 2:
      PlanMode2_TankerTimed_ReceiverSpeed(ctx);
      break;
    case 3:
      PlanMode3_BothTimed(ctx);
    default:
      // 未知模式：工程里可选择抛异常/报错，这里为了 demo 兼容，按 mode=0 处理。
      PlanMode0_BothSpeed(ctx);
      break;
  }
}

void RendezvousStage::FillTankerRendezvousByPolicy(PlanningContext& ctx) {
  // 仅在“加油机已在跑马场上且无需转场”的场景启用该策略：
  // - 这对应最新流程图：在下方四个模式的前提下，减少加油机会合相关部分
  // - “最小改动”原则：只强制覆盖 *entry_point / entry_direction*，
  //   而 meeting_altitude / meeting_speed 若已经由模式求解器算出来，则不去覆盖（避免破坏你已有的最优解）。
  if (!ctx.tanker_rt_decision.reduce_rendezvous) {
    return;
  }

  // 1) entry_point / entry_direction：直接取“当前位置/当前航向”
  //    （你也可以改成：投影到跑马场曲线上最近点；但那会增加改动量）
  ctx.rendezvous.tanker_entry_point_xy = ctx.tanker.initial_position_xy;
  ctx.rendezvous.tanker_entry_direction_deg = ctx.tanker.current_status.heading_deg;

  // 2) meeting_altitude / meeting_speed：只在“还没被填充/无效”时提供兜底
  //    这样不会覆盖 best_global 里已经求出来的值。
  if (ctx.rendezvous.tanker_meeting_altitude_m <= 0.0) {
    ctx.rendezvous.tanker_meeting_altitude_m = ctx.racetrack.altitude_m;
  }
  if (ctx.rendezvous.tanker_meeting_speed_mps <= 0.0) {
    ctx.rendezvous.tanker_meeting_speed_mps =
        (ctx.tanker_speed_bounds.cruise_speed_mps > 0.0)
            ? ctx.tanker_speed_bounds.cruise_speed_mps
            : ctx.racetrack.speed_mps;
  }
}

void RendezvousStage::PlanMode0_BothSpeed(PlanningContext& ctx) {
  // -------------------------
  // 0) 从 mission 读取“寻优偏好”
  // -------------------------
  refuel::mode0::Preference pref = refuel::mode0::Preference::kMinFuel;
  if (ctx.mission.prefs.primary_pref == 1) {
    pref = refuel::mode0::Preference::kMinTime;
  }

  // -------------------------
  // 1) 组装 mode0 白框依赖对象 + searcher
  // -------------------------
  refuel::mode0::TankerSpeedDiscretizer tanker_speed_discretizer; // 白框 1
  refuel::mode0::EntryPointComboGenerator combo_generator;         // 白框 2
  refuel::mode0::FeasibleCandidateGenerator candidate_generator;   // 白框 3
  refuel::mode0::HoldingLoopCalculator loop_calculator;            // 白框 4
  refuel::mode0::MeetAltitudeSelector altitude_selector;           // 白框 5
  refuel::mode0::CoarseCostEvaluator cost_evaluator;               // 白框 6
  refuel::mode0::GlobalSelector global_selector;                   // 白框 9

  refuel::mode0::Mode0FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &tanker_speed_discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &candidate_generator;
  deps.loop_calculator = &loop_calculator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  refuel::mode0::Mode0FixedSpeedSearcher searcher(deps);

  // 进入点列表（若为空会自动 fallback 生成）
  const std::vector<refuel::Vec3> entrypoints = refuel::mode0::GetEntryPointsFallback(ctx);

  // -------------------------
  // 2) 加油机 entry_point / entry_direction（“就近切入跑马场”的最简实现）
  // -------------------------
  // 注意：真正的“切入逻辑/入场航段”你们可能另有定义；
  // 这里仅做：选最近进入点 + 方向取从 tanker 初始点指向该点的航向。
  if (!ctx.tanker_rt_decision.reduce_rendezvous && !entrypoints.empty()) {
    const int ep_idx = refuel::mode0::NearestIndex2D(entrypoints, ctx.tanker.initial_position_xy);
    if (ep_idx >= 0) {
      const refuel::Vec3 ep = entrypoints[static_cast<size_t>(ep_idx)];
      ctx.rendezvous.tanker_entry_point_xy = ep;

      const double dx = ep.x - ctx.tanker.initial_position_xy.x;
      const double dy = ep.y - ctx.tanker.initial_position_xy.y;
      // 航向定义：x 向东 y 向北；heading=0 指北，顺时针为正
      const double rad = std::atan2(dx, dy);
      ctx.rendezvous.tanker_entry_direction_deg = std::fmod(rad * 180.0 / 3.14159265358979323846 + 360.0, 360.0);
    } else {
      ctx.rendezvous.tanker_entry_point_xy = ctx.tanker.initial_position_xy;
      ctx.rendezvous.tanker_entry_direction_deg = ctx.tanker.current_status.heading_deg;
    }
  }

  // -------------------------
  // 3) 对每个 receiver 单独跑一次 mode0（匹配当前 receiver 的奔赴速度离散）
  // -------------------------
  // 这样才能保证：
  // - candidate_generator / altitude_selector / cost_evaluator 使用的 ctx.receivers.front() 就是当前 receiver
  // - 输出 rr 与该 receiver 的入口点选择一致
  bool has_any_feasible = false;

  // 用于决定 tanker_meeting_speed/alt（选序号最小的 receiver 的结果）
  std::string primary_rid;
  int primary_order = std::numeric_limits<int>::max();
  refuel::mode0::BestGlobalResult primary_best{};

  for (const auto& r : ctx.receivers) {
    const std::string rid = r.id;

    // 受油机速度边界
    auto itb = ctx.receiver_speed_bounds.find(rid);

    // 构造 receiver 奔赴速度离散（传入 Solve）
    std::vector<double> receiver_tas_candidates_mps;
    if (itb != ctx.receiver_speed_bounds.end()) {
      const double vmin = itb->second.min_speed_mps;
      const double vmax = itb->second.max_speed_mps;
      const double vcruise = itb->second.cruise_speed_mps;

      if (vmax > 1e-6 && vmin > 1e-6 && vmax >= vmin) {
        const double step = 10.0; // 与 tanker 离散一致，便于调试与对齐
        for (double v = vmin; v <= vmax + 1e-9; v += step) {
          receiver_tas_candidates_mps.push_back(v);
        }
        if (vcruise > vmin + 1e-9 && vcruise < vmax - 1e-9) {
          bool exists = false;
          for (double v : receiver_tas_candidates_mps) {
            if (std::fabs(v - vcruise) < 1e-6) { exists = true; break; }
          }
          if (!exists) receiver_tas_candidates_mps.push_back(vcruise);
        }
        std::sort(receiver_tas_candidates_mps.begin(), receiver_tas_candidates_mps.end());
      }
    }

    // 构造一个“单 receiver 的临时 ctx”，对齐 mode0 头文件的默认假设（ctx.receivers.front()）
    PlanningContext local = ctx;
    local.receivers.clear();
    local.receivers.push_back(r);

    local.receiver_speed_bounds.clear();
    if (itb != ctx.receiver_speed_bounds.end()) {
      local.receiver_speed_bounds[rid] = itb->second;
    }

    // 运行 mode0
    const refuel::mode0::BestGlobalResult best_global = searcher.Solve(local, pref, receiver_tas_candidates_mps);

    RendezvousPlan::ReceiverRendezvous rr;
    rr.receiver_id = rid;
    rr.sequence_order = ctx.sequence.receiver_to_order.count(rid) ? ctx.sequence.receiver_to_order.at(rid) : 0;

    if (best_global.feasible) {
      has_any_feasible = true;

      // 输出：meeting speed/alt（来自白框6确定的 candidate）
      rr.meeting_speed_mps = best_global.best.best.meet_tas_mps;
      rr.meeting_altitude_m = best_global.best.best.meet_altitude_m;

      // 会合点 / 等待点：由 combo 选择的 entrypoint_idx 决定
      refuel::Vec3 p{};
      if (!best_global.best.combo.choices.empty() && !entrypoints.empty()) {
        const int ep_idx = best_global.best.combo.choices.front().entrypoint_idx;
        if (ep_idx >= 0 && static_cast<size_t>(ep_idx) < entrypoints.size()) {
          p = entrypoints[static_cast<size_t>(ep_idx)];
        } else {
          // 兜底：离 receiver 最近的点
          const int near = refuel::mode0::NearestIndex2D(entrypoints, r.initial_position_xy);
          if (near >= 0) p = entrypoints[static_cast<size_t>(near)];
        }
      }
      rr.coordinate_xy = p;
      rr.waiting_point_xy = p;
      rr.cycling_number = best_global.best.best.n_loops;

      // 用“序号最小的 receiver”来决定 tanker 的全局 meeting speed/alt（与你之前 output.json 的单值结构一致）
      const int order = rr.sequence_order;
      if (order >= 0 && order < primary_order) {
        primary_order = order;
        primary_rid = rid;
        primary_best = best_global;
      }
    } else {
      // 不可行兜底：保证 output 可写
      rr.meeting_altitude_m = (ctx.racetrack.altitude_m > 1e-6) ? ctx.racetrack.altitude_m : r.initial_position_lla.alt_m;
      rr.meeting_speed_mps = (itb != ctx.receiver_speed_bounds.end()) ? itb->second.cruise_speed_mps : 0.0;

      // 点位：离 receiver 最近的 entrypoint
      if (!entrypoints.empty()) {
        const int near = refuel::mode0::NearestIndex2D(entrypoints, r.initial_position_xy);
        if (near >= 0) {
          rr.coordinate_xy = entrypoints[static_cast<size_t>(near)];
          rr.waiting_point_xy = entrypoints[static_cast<size_t>(near)];
        }
      }
      rr.cycling_number = 0;
    }

    ctx.rendezvous.receivers[rid] = rr;
  }

  // -------------------------
  // 4) Tanker 的 meeting_speed/alt（取 primary receiver 的结果）
  // -------------------------
  if (has_any_feasible && primary_best.feasible) {
    ctx.rendezvous.tanker_meeting_speed_mps = primary_best.best.tanker_tas_mps;
    // meet altitude：取白框5/6的 meet_altitude（与 receiver 的 meet_altitude 对齐）
    ctx.rendezvous.tanker_meeting_altitude_m = primary_best.best.best.meet_altitude_m;
  } else {
    // 总兜底
    ctx.rendezvous.tanker_meeting_altitude_m = (ctx.racetrack.altitude_m > 1e-6) ? ctx.racetrack.altitude_m
                                                                                 : ctx.tanker.initial_position_lla.alt_m;
    ctx.rendezvous.tanker_meeting_speed_mps = (ctx.tanker_speed_bounds.cruise_speed_mps > 1e-6)
                                                  ? ctx.tanker_speed_bounds.cruise_speed_mps
                                                  : ctx.racetrack.speed_mps;
  }

  // -------------------------
  // 5) reduce_rendezvous 策略覆盖（不动 meet_alt/speed，只覆盖 entry_*）
  // -------------------------
  FillTankerRendezvousByPolicy(ctx);
}


void RendezvousStage::PlanMode1_TankerSpeed_ReceiverTimed(PlanningContext& ctx) {
  // -------------------------
  // 0) 从 mission 读取“寻优偏好”
  // -------------------------
  refuel::mode1::Preference pref = refuel::mode1::Preference::kMinFuel;
  if (ctx.mission.prefs.primary_pref == 1) {
    pref = refuel::mode1::Preference::kMinTime;
  }

  // -------------------------
  // 1) 组装 mode1 白框依赖对象 + searcher
  // -------------------------
  refuel::mode1::TankerSpeedDiscretizer tanker_speed_discretizer; // 白框 1
  refuel::mode1::EntryPointComboGenerator combo_generator;         // 白框 2
  refuel::mode1::FeasibleCandidateGenerator candidate_generator;   // 白框 3（解 vr）
  refuel::mode1::MeetAltitudeSelector altitude_selector;           // 白框 5
  refuel::mode1::CoarseCostEvaluator cost_evaluator;               // 白框 6
  refuel::mode1::GlobalSelector global_selector;                   // 白框 9

  refuel::mode1::Mode1FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &tanker_speed_discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &candidate_generator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  refuel::mode1::Mode1FixedSpeedSearcher searcher(deps);

  // 进入点列表（若为空会自动 fallback 生成）
  const std::vector<refuel::Vec3> entrypoints = refuel::mode1::GetEntryPointsFallback(ctx);

  // -------------------------
  // 2) 加油机 entry_point / entry_direction（“就近切入跑马场”的最简实现）
  // -------------------------
  if (!ctx.tanker_rt_decision.reduce_rendezvous && !entrypoints.empty()) {
    const int ep_idx = refuel::mode1::NearestIndex2D(entrypoints, ctx.tanker.initial_position_xy);
    if (ep_idx >= 0) {
      const refuel::Vec3 ep = entrypoints[static_cast<size_t>(ep_idx)];
      ctx.rendezvous.tanker_entry_point_xy = ep;

      const double dx = ep.x - ctx.tanker.initial_position_xy.x;
      const double dy = ep.y - ctx.tanker.initial_position_xy.y;
      // 航向定义：x 向东 y 向北；heading=0 指北，顺时针为正
      const double rad = std::atan2(dx, dy);
      ctx.rendezvous.tanker_entry_direction_deg =
          std::fmod(rad * 180.0 / 3.14159265358979323846 + 360.0, 360.0);
    } else {
      ctx.rendezvous.tanker_entry_point_xy = ctx.tanker.initial_position_xy;
      ctx.rendezvous.tanker_entry_direction_deg = ctx.tanker.current_status.heading_deg;
    }
  }

  // -------------------------
  // 3) 对每个 receiver 单独跑一次 mode1
  // -------------------------
  // 说明：mode1_fixed_speed_search.hpp 的基础实现中：
  // - cost_evaluator 使用 ctx.receivers.front() 作为当前 receiver
  // 因此这里与 mode0 一样，构造“单 receiver 的临时 ctx”来对齐假设。
  bool has_any_feasible = false;

  // 用于决定 tanker_meeting_speed/alt（选序号最小的 receiver 的结果）
  std::string primary_rid;
  int primary_order = std::numeric_limits<int>::max();
  refuel::mode1::BestGlobalResult primary_best{};

  for (const auto& r : ctx.receivers) {
    const std::string rid = r.id;

    // 受油机速度边界（mode1 求解 vr 时需要）
    const auto itb = ctx.receiver_speed_bounds.find(rid);

    // 构造一个“单 receiver 的临时 ctx”，对齐 mode1 头文件的默认假设（ctx.receivers.front()）
    PlanningContext local = ctx;
    local.receivers.clear();
    local.receivers.push_back(r);

    local.receiver_speed_bounds.clear();
    if (itb != ctx.receiver_speed_bounds.end()) {
      local.receiver_speed_bounds[rid] = itb->second;
    }

    // 运行 mode1
    const refuel::mode1::BestGlobalResult best_global = searcher.Solve(local, pref);

    RendezvousPlan::ReceiverRendezvous rr;
    rr.receiver_id = rid;
    rr.sequence_order = ctx.sequence.receiver_to_order.count(rid) ? ctx.sequence.receiver_to_order.at(rid) : 0;

    if (best_global.feasible) {
      has_any_feasible = true;

      // 输出：
      // - receiver 的 meeting_speed_mps：取解出来的奔赴真速 vr（允许小数）
      // - meet altitude：来自白框 5/6
      rr.meeting_speed_mps = best_global.best.best.receiver_tas_mps;
      rr.meeting_altitude_m = best_global.best.best.meet_altitude_m;

      // 会合点 / 等待点：由 combo 选择的 entrypoint_idx 决定
      refuel::Vec3 p{};
      if (!best_global.best.combo.choices.empty() && !entrypoints.empty()) {
        const int ep_idx = best_global.best.combo.choices.front().entrypoint_idx;
        if (ep_idx >= 0 && static_cast<size_t>(ep_idx) < entrypoints.size()) {
          p = entrypoints[static_cast<size_t>(ep_idx)];
        } else {
          // 兜底：离 receiver 最近的点
          const int near = refuel::mode1::NearestIndex2D(entrypoints, r.initial_position_xy);
          if (near >= 0) p = entrypoints[static_cast<size_t>(near)];
        }
      }
      rr.coordinate_xy = p;
      rr.waiting_point_xy = p;
      rr.cycling_number = 0; // mode1 不需要盘旋圈数

      // 用“序号最小的 receiver”来决定 tanker 的全局 meeting speed/alt
      const int order = rr.sequence_order;
      if (order >= 0 && order < primary_order) {
        primary_order = order;
        primary_rid = rid;
        primary_best = best_global;
      }
    } else {
      // 不可行兜底：保证 output 可写
      rr.meeting_altitude_m =
          (ctx.racetrack.altitude_m > 1e-6) ? ctx.racetrack.altitude_m : r.initial_position_lla.alt_m;
      rr.meeting_speed_mps = (itb != ctx.receiver_speed_bounds.end()) ? itb->second.cruise_speed_mps : 0.0;

      // 点位：离 receiver 最近的 entrypoint
      if (!entrypoints.empty()) {
        const int near = refuel::mode1::NearestIndex2D(entrypoints, r.initial_position_xy);
        if (near >= 0) {
          rr.coordinate_xy = entrypoints[static_cast<size_t>(near)];
          rr.waiting_point_xy = entrypoints[static_cast<size_t>(near)];
        }
      }
      rr.cycling_number = 0;
    }

    ctx.rendezvous.receivers[rid] = rr;
  }

  // -------------------------
  // 4) Tanker 的 meeting_speed/alt（取 primary receiver 的结果）
  // -------------------------
  if (has_any_feasible && primary_best.feasible) {
    // tanker meeting speed：取白框 1/9 最终选中的 tanker_tas
    ctx.rendezvous.tanker_meeting_speed_mps = primary_best.best.tanker_tas_mps;
    // meet altitude：与 receiver 的 meet_altitude 对齐
    ctx.rendezvous.tanker_meeting_altitude_m = primary_best.best.best.meet_altitude_m;
  } else {
    // 总兜底
    ctx.rendezvous.tanker_meeting_altitude_m = (ctx.racetrack.altitude_m > 1e-6)
                                                   ? ctx.racetrack.altitude_m
                                                   : ctx.tanker.initial_position_lla.alt_m;
    ctx.rendezvous.tanker_meeting_speed_mps = (ctx.tanker_speed_bounds.cruise_speed_mps > 1e-6)
                                                  ? ctx.tanker_speed_bounds.cruise_speed_mps
                                                  : ctx.racetrack.speed_mps;
  }

  // -------------------------
  // 5) reduce_rendezvous 策略覆盖（不动 meet_alt/speed，只覆盖 entry_*）
  // -------------------------
  FillTankerRendezvousByPolicy(ctx);
}

void RendezvousStage::PlanMode2_TankerTimed_ReceiverSpeed(PlanningContext& ctx) {
  // Mode 2：加油机定时 + 受油机定速
  // TODO：根据你们定义实现

  PlanMode0_BothSpeed(ctx); // 先复用骨架
}

void RendezvousStage::PlanMode3_BothTimed(PlanningContext& ctx) {
  // Mode 3：加油机定时 + 受油机定时
  // TODO：根据你们定义实现

  PlanMode0_BothSpeed(ctx); // 先复用骨架
}

} // namespace refuel
