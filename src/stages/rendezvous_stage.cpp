#include "stages/rendezvous_stage.hpp"

// mode_type = 0 的“白框流程”接口骨架（只提供接口 + 注释，不含具体算法实现）
#include "mode0/mode0_fixed_speed_search.hpp"

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
  // 注意：这里不强行假设 primary_pref 的含义（因为不同项目编码可能不同）。
  // 你可以在这里统一做映射，例如：
  //   primary_pref==0 -> 油耗优先
  //   primary_pref==1 -> 时间优先
  mode0::Preference pref = mode0::Preference::kMinFuel;
  if (ctx.mission.prefs.primary_pref == 1) {
    pref = mode0::Preference::kMinTime;
  }

  // -------------------------
  // 1) 组装“白框环节”的依赖对象
  // -------------------------
  // 这些对象只提供接口（当前均是 TODO 空实现），后续你逐个填算法即可。
  mode0::TankerSpeedDiscretizer tanker_speed_discretizer; // 白框 1
  mode0::EntryPointComboGenerator combo_generator;         // 白框 2
  mode0::FeasibleCandidateGenerator candidate_generator;   // 白框 3
  mode0::HoldingLoopCalculator loop_calculator;            // 白框 4
  mode0::MeetAltitudeSelector altitude_selector;           // 白框 5
  mode0::CoarseCostEvaluator cost_evaluator;               // 白框 6
  mode0::GlobalSelector global_selector;                   // 白框 9

  mode0::Mode0FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &tanker_speed_discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &candidate_generator;
  deps.loop_calculator = &loop_calculator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  mode0::Mode0FixedSpeedSearcher searcher(deps);

  // -------------------------
  // 2) 准备“受油机速度候选集合”（绿色说明：受油机速度也离散化）
  // -------------------------
  // 白框流程里，“受油机速度离散化”不是一个单独白框环节；
  // 它通常作为白框 3 / 白框 6 的内部支撑。
  //
  // 为了让接口完整，这里先在 Stage 层准备一份“公共候选集合”传入搜索器。
  // 真实项目中你可能希望：
  //   - 每个 receiver 用自己独立的候选集合（不同机型/不同 bounds）
  //   - 或候选集合来自性能表的离散网格
  //
  // TODO：把下面这个简单离散逻辑替换成你项目里的真实离散策略。
  std::vector<double> receiver_candidates_mps;
  receiver_candidates_mps.reserve(16);
  if (!ctx.receivers.empty()) {
    const std::string& rid = ctx.receivers.front().id;
    if (ctx.receiver_speed_bounds.count(rid)) {
      const auto& b = ctx.receiver_speed_bounds.at(rid);
      const double step = 5.0; // m/s（示例步长）
      for (double v = b.min_speed_mps; v <= b.max_speed_mps + 1e-9; v += step) {
        receiver_candidates_mps.push_back(v);
      }
    }
  }

  // -------------------------
  // 3) 运行白框搜索（两层遍历 + 组合内选优 + 全局选优）
  // -------------------------
  // 注意：当前各模块都是 TODO 空实现，所以通常会返回 infeasible。
  const mode0::BestGlobalResult best_global = searcher.Solve(ctx, pref, receiver_candidates_mps);

  // -------------------------
  // 4) 把结果写回 ctx.rendezvous（对齐 output.json 字段）
  // -------------------------
  if (best_global.feasible) {
    // ====== Tanker 输出（output.json.tanker）======
    ctx.rendezvous.tanker_meeting_speed_mps = best_global.best.tanker_tas_mps;
    // meet altitude：通常与白框 5/跑马场高度有关；这里示例写 best.best.meet_altitude_m
    ctx.rendezvous.tanker_meeting_altitude_m = best_global.best.best.meet_altitude_m;

    // entry point / entry direction：
    // - 若 ctx.tanker_rt_decision.reduce_rendezvous==true（无需转场且已在跑马场），
    //   则后面会调用 FillTankerRendezvousByPolicy() 强制用“当前点/当前航向”覆盖，
    //   从而“减少加油机会合相关部分”的计算。
    // - 若 reduce_rendezvous==false，则这里应走你原本的“就近切入跑马场”逻辑。
    //
    // TODO：真实实现请在 EntryPointComboGenerator 或单独模块里完成，并写回这里。
    ctx.rendezvous.tanker_entry_point_xy = {};
    ctx.rendezvous.tanker_entry_direction_deg = 0.0;

    // ====== Receivers 输出（output.json.receivers.RE00X）======
    for (const auto& r : ctx.receivers) {
      RendezvousPlan::ReceiverRendezvous rr;
      rr.receiver_id = r.id;

      // sequence_order：来自 SequenceStage 的输出
      rr.sequence_order = ctx.sequence.receiver_to_order.count(r.id) ? ctx.sequence.receiver_to_order.at(r.id) : 0;

      // meeting speed / altitude：来自白框 6/5 的最优 candidate
      rr.meeting_speed_mps = best_global.best.best.receiver_tas_mps;
      rr.meeting_altitude_m = best_global.best.best.meet_altitude_m;

      // coordinate_xy / waiting_point_xy / cycling_number：
      // - coordinate_xy：会合点（通常与 combo 指定的 entrypoint/会合点有关）
      // - waiting_point_xy：等待点（可能是 entrypoint 或跑马场某个点）
      // - cycling_number：白框 4 输出
      //
      // TODO：在你填完白框 2/3/4/5/6 的实现后，这些字段应从 best_global.best.combo / best_global.best.best 中计算得到。
      rr.coordinate_xy = {};
      rr.waiting_point_xy = {};
      rr.cycling_number = best_global.best.best.n_loops;

      ctx.rendezvous.receivers[r.id] = rr;
    }

    // ---- 关键小改动：按最新流程图策略，必要时覆盖 tanker 的 entry 信息（尽量不动 meet_alt/speed）----
    FillTankerRendezvousByPolicy(ctx);
    return;
  }

  // -------------------------
  // 5) fallback：接口演示填充（保证 demo 可输出）
  // -------------------------
  // 当你还没实现算法时，为了让 output.json 能正常写出来，
  // 这里用“巡航速度 + 跑马场高度”的占位填充。
  ctx.rendezvous.tanker_meeting_altitude_m = ctx.racetrack.altitude_m;
  ctx.rendezvous.tanker_meeting_speed_mps = ctx.tanker_speed_bounds.cruise_speed_mps;
  ctx.rendezvous.tanker_entry_point_xy = {};     // TODO: 计算
  ctx.rendezvous.tanker_entry_direction_deg = 0; // TODO: 计算

  for (const auto& r : ctx.receivers) {
    RendezvousPlan::ReceiverRendezvous rr;
    rr.receiver_id = r.id;
    rr.sequence_order = ctx.sequence.receiver_to_order.count(r.id) ? ctx.sequence.receiver_to_order.at(r.id) : 0;
    rr.meeting_altitude_m = ctx.racetrack.altitude_m;
    rr.meeting_speed_mps =
        ctx.receiver_speed_bounds.count(r.id) ? ctx.receiver_speed_bounds.at(r.id).cruise_speed_mps : 0.0;

    rr.coordinate_xy = {};     // TODO: 计算会合点 (x,y)
    rr.waiting_point_xy = {};  // TODO: 计算等待点 (x,y)
    rr.cycling_number = 0;     // TODO: 盘旋圈数

    ctx.rendezvous.receivers[r.id] = rr;
  }

  // ---- 同样在 fallback 下也应用策略：若无需转场，则至少把 tanker entry 填成“当前点/当前航向”----
  FillTankerRendezvousByPolicy(ctx);
}


void RendezvousStage::PlanMode1_TankerSpeed_ReceiverTimed(PlanningContext& ctx) {
  // Mode 1：加油机定速 + 受油机定时（demo/mission4.json mode_type=1）
  //
  // 典型输入还会用到：
  //   ctx.mission.refuel_mode.designated_time（目标时刻）
  // 你们项目可能会把这个时刻换算成“任务开始后多少秒”之类。
  //
  // TODO：同 Mode0，但额外满足“受油机到达时间=指定时间”。

  PlanMode0_BothSpeed(ctx); // 先复用骨架，后续你替换成真实实现
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
