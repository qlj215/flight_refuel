#include "tests/test_framework.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "mode1/mode1_fixed_speed_search.hpp"

using refuel::mode1::Preference;

namespace {

// 小工具：避免依赖测试框架是否提供 NEAR
static bool ExpectNear(double a, double b, double eps = 1e-6) {
  return std::fabs(a - b) <= eps;
}

// =========================
// 1) 单元测试：白框 9 GlobalSelector
// =========================
bool Test_GlobalSelector_PicksMinCost() {
  using namespace refuel::mode1;

  GlobalSelector selector;

  BestUnderTankerSpeed a;
  a.feasible = true;
  a.tanker_tas_mps = 100.0;
  a.best.cost = 50.0;

  BestUnderTankerSpeed b;
  b.feasible = true;
  b.tanker_tas_mps = 200.0;
  b.best.cost = 10.0;

  BestUnderTankerSpeed c;
  c.feasible = false;
  c.tanker_tas_mps = 300.0;
  c.best.cost = 1.0; // infeasible 应被忽略

  std::vector<BestUnderTankerSpeed> all{a, b, c};
  BestGlobalResult g = selector.Select(all, Preference::kMinFuel);

  REFUEL_EXPECT_TRUE(g.feasible);
  REFUEL_EXPECT_EQ(g.best.tanker_tas_mps, 200.0);
  REFUEL_EXPECT_EQ(g.best.best.cost, 10.0);
  return true;
}

// =========================
// 2) 单元测试：白框 1 TankerSpeedDiscretizer
// 目的：验证离散输出包含 [min,max] 网格 + cruise 插入 + 排序稳定
// =========================
bool Test_TankerSpeedDiscretizer_Basic() {
  using namespace refuel::mode1;

  refuel::PlanningContext ctx;
  ctx.tanker_speed_bounds.min_speed_mps = 100.0;
  ctx.tanker_speed_bounds.max_speed_mps = 130.0;
  ctx.tanker_speed_bounds.cruise_speed_mps = 115.0;

  TankerSpeedDiscretizer d;
  TankerSpeedCandidates out = d.Discretize(ctx);

  // step=10: 100 110 120 130，再插入 115 -> 排序后应为 100 110 115 120 130
  REFUEL_EXPECT_EQ(static_cast<int>(out.tanker_tas_mps.size()), 5);
  REFUEL_EXPECT_EQ(out.tanker_tas_mps[0], 100.0);
  REFUEL_EXPECT_EQ(out.tanker_tas_mps[1], 110.0);
  REFUEL_EXPECT_EQ(out.tanker_tas_mps[2], 115.0);
  REFUEL_EXPECT_EQ(out.tanker_tas_mps[3], 120.0);
  REFUEL_EXPECT_EQ(out.tanker_tas_mps[4], 130.0);
  return true;
}

// =========================
// 3) 单元测试：白框 2 EntryPointComboGenerator
// 目的：验证 topK 逻辑 + 组合笛卡尔积数量
// =========================
bool Test_EntryPointComboGenerator_TopKAndCartesian() {
  using namespace refuel::mode1;

  refuel::PlanningContext ctx;

  // 2 架受油机
  refuel::AircraftConfig r1;
  r1.id = "R1";
  r1.initial_position_xy = {0.0, 0.0, 0.0};

  refuel::AircraftConfig r2;
  r2.id = "R2";
  r2.initial_position_xy = {100.0, 0.0, 0.0};

  ctx.receivers = {r1, r2};

  // 4 个进入点
  ctx.racetrack.entrypoints_xy = {
      {10.0, 0.0, 0.0},
      {20.0, 0.0, 0.0},
      {30.0, 0.0, 0.0},
      {1000.0, 0.0, 0.0},
  };

  EntryPointComboGenerator g;
  EntryPointCombos combos = g.Generate(ctx);

  // entrypoints=4 => topK=min(3,4)=3
  // receivers=2 => combos = 3^2 = 9
  REFUEL_EXPECT_EQ(static_cast<int>(combos.combos.size()), 9);

  // 每个 combo 应该有两个 choice，且 receiver_id 分别是 R1/R2（顺序按 all_opts 构造）
  for (const auto& c : combos.combos) {
    REFUEL_EXPECT_EQ(static_cast<int>(c.choices.size()), 2);
    REFUEL_EXPECT_EQ(c.choices[0].receiver_id, std::string("R1"));
    REFUEL_EXPECT_EQ(c.choices[1].receiver_id, std::string("R2"));
    REFUEL_EXPECT_TRUE(c.choices[0].entrypoint_idx >= 0);
    REFUEL_EXPECT_TRUE(c.choices[1].entrypoint_idx >= 0);
  }
  return true;
}

// =========================
// 4) 单元测试：白框 3 FeasibleCandidateGenerator
// 目的：验证：
//  - 解 vr（允许小数），使得 t_receiver == t_tanker
//  - vr bounds 检查（vr out-of-bounds -> 返回 infeasible candidate）
//  - tanker speed bounds / meet_tas bounds 触发时，返回空集合（对齐实现）
// =========================
bool Test_FeasibleCandidateGenerator_SolveVrAndBounds() {
  using namespace refuel::mode1;

  refuel::PlanningContext ctx;

  // tanker/receiver 初始位置
  ctx.tanker.initial_position_xy = {0.0, 0.0, 0.0};

  refuel::AircraftConfig r;
  r.id = "R1";
  r.initial_position_xy = {0.0, 0.0, 0.0};
  ctx.receivers = {r};

  // 进入点：离 tanker 1km
  ctx.racetrack.entrypoints_xy = {{1000.0, 0.0, 0.0}};

  // tanker bounds
  ctx.tanker_speed_bounds.min_speed_mps = 80.0;
  ctx.tanker_speed_bounds.max_speed_mps = 120.0;

  // receiver bounds
  refuel::SpeedBounds rb;
  rb.min_speed_mps = 70.0;
  rb.max_speed_mps = 150.0;
  rb.cruise_speed_mps = 100.0;
  ctx.receiver_speed_bounds["R1"] = rb;

  // combo：R1 选 entry 0
  EntryPointCombo combo;
  combo.choices.push_back({"R1", 0});

  FeasibleCandidateGenerator gen;

  // ---- Case A：可行：tanker_dist=1000, vt=100 => t=10s；receiver_dist=1000 => vr=100
  {
    const double vt = 100.0;
    std::vector<MeetingCandidate> cands = gen.Generate(ctx, vt, combo);

    REFUEL_EXPECT_EQ(static_cast<int>(cands.size()), 1);
    REFUEL_EXPECT_TRUE(cands[0].feasible);
    REFUEL_EXPECT_TRUE(ExpectNear(cands[0].meet_tas_mps, vt));
    REFUEL_EXPECT_TRUE(ExpectNear(cands[0].tanker_arrive_time_s, 10.0));
    REFUEL_EXPECT_TRUE(ExpectNear(cands[0].receiver_arrive_time_s, 10.0));
    REFUEL_EXPECT_TRUE(ExpectNear(cands[0].delta_t_s, 0.0));
    REFUEL_EXPECT_TRUE(ExpectNear(cands[0].receiver_tas_mps, 100.0));
  }

  // ---- Case B：vr out-of-bounds：让 receiver 更远，tanker 更近 -> vr >> vt
  {
    refuel::PlanningContext ctx2 = ctx;
    ctx2.receivers[0].initial_position_xy = {-5000.0, 0.0, 0.0}; // receiver_dist=6000

    // 收紧 receiver bounds：保证 meet_tas=85 合法，但 vr 不合法
    refuel::SpeedBounds rb2;
    rb2.min_speed_mps = 70.0;
    rb2.max_speed_mps = 90.0;
    rb2.cruise_speed_mps = 80.0;
    ctx2.receiver_speed_bounds["R1"] = rb2;

    const double vt = 85.0; // meet_tas=85 within [70,90]
    std::vector<MeetingCandidate> cands = gen.Generate(ctx2, vt, combo);

    // 由于 meet_tas 在 bounds 内，这里会返回一个 infeasible candidate（而不是空集合）
    REFUEL_EXPECT_EQ(static_cast<int>(cands.size()), 1);
    REFUEL_EXPECT_TRUE(!cands[0].feasible);
    // vr = receiver_dist / (tanker_dist/vt) = 6000 / (1000/85) = 510
    REFUEL_EXPECT_TRUE(ExpectNear(cands[0].receiver_tas_mps, 510.0));
  }

  // ---- Case C：tanker speed out of bounds -> 返回空集合
  {
    const double vt = 200.0; // > ctx.tanker_speed_bounds.max
    std::vector<MeetingCandidate> cands = gen.Generate(ctx, vt, combo);
    REFUEL_EXPECT_EQ(static_cast<int>(cands.size()), 0);
  }

  // ---- Case D：meet_tas（=vt）不在 receiver bounds -> 返回空集合
  {
    refuel::PlanningContext ctx3 = ctx;
    refuel::SpeedBounds rb3;
    rb3.min_speed_mps = 70.0;
    rb3.max_speed_mps = 90.0; // vt=100 out
    rb3.cruise_speed_mps = 80.0;
    ctx3.receiver_speed_bounds["R1"] = rb3;

    const double vt = 100.0;
    std::vector<MeetingCandidate> cands = gen.Generate(ctx3, vt, combo);
    REFUEL_EXPECT_EQ(static_cast<int>(cands.size()), 0);
  }

  return true;
}

// =========================
// 5) 单元测试：白框 5 MeetAltitudeSelector
// 目的：
//   A) H_ref 优先规则：优先用 receiver.initial_position_lla.alt_m；否则回退 racetrack.altitude_m
//   B) meet_ias_mps 来自 “(H, IAS)->TAS” 真速表的反查（不再假设 IAS≈TAS）
// =========================
bool Test_MeetAltitudeSelector_UsesInitialAltOrFallback() {
  using namespace refuel::mode1;

  refuel::PlanningContext ctx;

  // --------- 构造 receiver ----------
  refuel::AircraftConfig r;
  r.id = "R1";
  r.initial_position_lla.alt_m = 6000.0;  // H_ref=6000
  ctx.receivers = {r};
  ctx.racetrack.altitude_m = 5000.0;

  // --------- 构造真速表 cruise_performance_table： (H, IAS)->TAS ----------
  // altitude: 5000, 6000, 7000
  // IAS grid: 100, 140, 180
  // 令 TAS = IAS + offset，其中 offset = (alt-5000)/1000 * 10
  //   alt=5000 offset=0
  //   alt=6000 offset=10
  //   alt=7000 offset=20
  {
    auto& cp = ctx.receivers[0].cruise_perf;
    cp.altitude_levels = {5000.0, 6000.0, 7000.0};
    cp.speed_levels    = {100.0, 140.0, 180.0}; // IAS

    cp.true_airspeed_data.resize(cp.altitude_levels.size());
    for (size_t i = 0; i < cp.altitude_levels.size(); ++i) {
      const double alt = cp.altitude_levels[i];
      const double offset = (alt - 5000.0) / 1000.0 * 10.0;
      cp.true_airspeed_data[i].resize(cp.speed_levels.size());
      for (size_t j = 0; j < cp.speed_levels.size(); ++j) {
        const double ias = cp.speed_levels[j];
        cp.true_airspeed_data[i][j] = ias + offset; // TAS
      }
    }
  }

  // --------- 构造油耗表 fuel_consumption_table： (H, TAS)->fuel_rate_kgps ----------
  {
    auto& ft = ctx.receivers[0].fuel_table;
    ft.altitude_levels = {5000.0, 6000.0, 7000.0};
    ft.speed_levels    = {100.0, 140.0, 180.0};  // TAS

    ft.consumption_data.resize(ft.altitude_levels.size());
    for (size_t i = 0; i < ft.altitude_levels.size(); ++i) {
      const double alt = ft.altitude_levels[i];
      ft.consumption_data[i].resize(ft.speed_levels.size());
      for (size_t j = 0; j < ft.speed_levels.size(); ++j) {
        const double tas = ft.speed_levels[j];
        ft.consumption_data[i][j] = 1.0 + 0.0001 * alt + 0.001 * tas; // kg/s（示例）
      }
    }
  }

  // --------- 构造候选：给定 meet_tas_mps，需要白框5反查出 IAS ----------
  // 在 alt=6000 (offset=10) 下：
  //   meet_tas=120 -> IAS=110
  //   meet_tas=160 -> IAS=150
  std::vector<MeetingCandidate> cands(2);
  cands[0].feasible = true; cands[0].meet_tas_mps = 120.0;
  cands[1].feasible = true; cands[1].meet_tas_mps = 160.0;

  MeetAltitudeSelector sel;
  sel.Apply(cands, ctx, "R1");

  // 断言：优先选 H_ref=6000（高度变化=0），并且 IAS 来自反查表（不等于 TAS）
  REFUEL_EXPECT_TRUE(cands[0].feasible);
  REFUEL_EXPECT_EQ(cands[0].meet_altitude_m, 6000.0);
  REFUEL_EXPECT_NEAR(cands[0].meet_ias_mps, 110.0, 1e-6);
  REFUEL_EXPECT_EQ(cands[0].altitude_change_m, 0.0);

  REFUEL_EXPECT_TRUE(cands[1].feasible);
  REFUEL_EXPECT_EQ(cands[1].meet_altitude_m, 6000.0);
  REFUEL_EXPECT_NEAR(cands[1].meet_ias_mps, 150.0, 1e-6);
  REFUEL_EXPECT_EQ(cands[1].altitude_change_m, 0.0);

  // --------- 再测 fallback：receiver 初始高度=0 -> H_ref = racetrack.altitude=5000 ----------
  ctx.receivers[0].initial_position_lla.alt_m = 0.0;

  for (auto& c : cands) {
    c.meet_altitude_m = 0.0;
    c.meet_ias_mps = 0.0;
    c.altitude_change_m = 0.0;
    c.feasible = true;
  }

  // 在 alt=5000 (offset=0) 下：
  //   meet_tas=120 -> IAS=120
  //   meet_tas=160 -> IAS=160
  sel.Apply(cands, ctx, "R1");

  REFUEL_EXPECT_TRUE(cands[0].feasible);
  REFUEL_EXPECT_EQ(cands[0].meet_altitude_m, 5000.0);
  REFUEL_EXPECT_NEAR(cands[0].meet_ias_mps, 120.0, 1e-6);

  REFUEL_EXPECT_TRUE(cands[1].feasible);
  REFUEL_EXPECT_EQ(cands[1].meet_altitude_m, 5000.0);
  REFUEL_EXPECT_NEAR(cands[1].meet_ias_mps, 160.0, 1e-6);

  return true;
}

// =========================
// 6) 单元测试：白框 6 CoarseCostEvaluator
// 目的：验证 pref 不同会选不同候选（fuel vs time）
// =========================
bool Test_CoarseCostEvaluator_PrefAffectsChoice() {
  using namespace refuel::mode1;

  CoarseCostEvaluator eval;
  refuel::PlanningContext ctx;

  // 让 ctx.receivers.front() 存在，避免 receiver_rate 走空指针分支
  refuel::AircraftConfig r;
  r.id = "R1";
  ctx.receivers = {r};

  std::vector<MeetingCandidate> cands;

  // cand1：fuel 小，但 time 大
  // 使用 fallback rate：tanker_rate=2.0，receiver_rate=1.0
  // tanker=10, receiver_adj=100 -> time=max=100, fuel=2*10+100=120
  MeetingCandidate c1;
  c1.feasible = true;
  c1.tanker_arrive_time_s = 10.0;
  c1.receiver_arrive_time_s = 100.0;
  c1.receiver_arrive_time_adjusted_s = 100.0;
  c1.meet_altitude_m = 6000.0;
  c1.meet_tas_mps = 120.0;
  c1.receiver_tas_mps = 120.0;
  cands.push_back(c1);

  // cand2：time 小，但 fuel 大
  // tanker=90, receiver_adj=90 -> time=90, fuel=2*90+90=270
  MeetingCandidate c2;
  c2.feasible = true;
  c2.tanker_arrive_time_s = 90.0;
  c2.receiver_arrive_time_s = 90.0;
  c2.receiver_arrive_time_adjusted_s = 90.0;
  c2.meet_altitude_m = 6000.0;
  c2.meet_tas_mps = 120.0;
  c2.receiver_tas_mps = 120.0;
  cands.push_back(c2);

  {
    auto best_fuel = eval.PickBest(cands, ctx, Preference::kMinFuel);
    REFUEL_EXPECT_TRUE(best_fuel.has_value());
    REFUEL_EXPECT_TRUE(ExpectNear(best_fuel->total_fuel, 120.0)); // 选 cand1
  }
  {
    auto best_time = eval.PickBest(cands, ctx, Preference::kMinTime);
    REFUEL_EXPECT_TRUE(best_time.has_value());
    REFUEL_EXPECT_TRUE(ExpectNear(best_time->total_time, 90.0)); // 选 cand2
  }
  return true;
}

// =========================
// 7) 冒烟测试：空 ctx 不崩溃，且返回 infeasible
// =========================
bool Test_Mode1_Smoke_NoCrashOnEmptyCtx() {
  using namespace refuel::mode1;

  TankerSpeedDiscretizer tanker_speed_discretizer;
  EntryPointComboGenerator combo_generator;
  FeasibleCandidateGenerator candidate_generator;
  MeetAltitudeSelector altitude_selector;
  CoarseCostEvaluator cost_evaluator;
  GlobalSelector global_selector;

  Mode1FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &tanker_speed_discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &candidate_generator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  Mode1FixedSpeedSearcher searcher(deps);

  refuel::PlanningContext ctx; // 空 ctx
  BestGlobalResult best = searcher.Solve(ctx, Preference::kMinFuel);

  REFUEL_EXPECT_TRUE(!best.feasible);
  return true;
}

// =========================
// 8) 真实实现的端到端测试：尽可能覆盖白框 1/2/3/5/6/9
// 目的：不用 fake，构造一个“最小可行 ctx”，验证 Solve() 能走完整流程并选到符合直觉的最优。
// =========================
bool Test_Mode1_EndToEnd_RealImpl_MinimalFeasible() {
  using namespace refuel::mode1;

  // 真实实现依赖
  TankerSpeedDiscretizer tanker_speed_discretizer;
  EntryPointComboGenerator combo_generator;
  FeasibleCandidateGenerator candidate_generator;
  MeetAltitudeSelector altitude_selector;
  CoarseCostEvaluator cost_evaluator;
  GlobalSelector global_selector;

  Mode1FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &tanker_speed_discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &candidate_generator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  Mode1FixedSpeedSearcher searcher(deps);

  refuel::PlanningContext ctx;

  // --- tanker bounds：离散出 90/100/110（step=10）
  ctx.tanker_speed_bounds.min_speed_mps = 90.0;
  ctx.tanker_speed_bounds.cruise_speed_mps = 100.0;
  ctx.tanker_speed_bounds.max_speed_mps = 110.0;

  // --- tanker/receiver 初始位置
  ctx.tanker.initial_position_xy = {0.0, 0.0, 0.0};

  refuel::AircraftConfig r;
  r.id = "R1";
  r.initial_position_xy = {0.0, 0.0, 0.0};
  r.initial_position_lla.alt_m = 6000.0; // 白框 5 会优先使用
  ctx.receivers = {r};

  // --- receiver bounds：允许 meet_tas=vt，且 vr=vt 合法
  refuel::SpeedBounds rb;
  rb.min_speed_mps = 80.0;
  rb.cruise_speed_mps = 100.0;
  rb.max_speed_mps = 120.0;
  ctx.receiver_speed_bounds["R1"] = rb;

  // --- racetrack：两个进入点（第 0 个更近，应该被选中）
  ctx.racetrack.entrypoints_xy = {
      {1000.0, 0.0, 0.0},   // near
      {10000.0, 0.0, 0.0},  // far
  };
  ctx.racetrack.radius_m = 1000.0;
  ctx.racetrack.length_m = 0.0;
  ctx.racetrack.speed_mps = 0.0;
  ctx.racetrack.altitude_m = 5000.0; // fallback 高度（本例不会用到，因为 receiver 初始高度=6000）

  BestGlobalResult best = searcher.Solve(ctx, Preference::kMinFuel);

  REFUEL_EXPECT_TRUE(best.feasible);

  // fuel/time 都随 vt 增大而下降（距离固定，mode1 同时到达）
  REFUEL_EXPECT_EQ(best.best.tanker_tas_mps, 110.0);

  REFUEL_EXPECT_TRUE(!best.best.combo.choices.empty());
  // 进入点应选更近的第 0 个
  REFUEL_EXPECT_EQ(best.best.combo.choices.front().entrypoint_idx, 0);

  // mode1 解 vr：在本例中 receiver_dist==tanker_dist => vr==vt
  REFUEL_EXPECT_NEAR(best.best.best.receiver_tas_mps, 110.0, 1e-6);

  // 白框 5：会合高度取 receiver 初始高度
  REFUEL_EXPECT_EQ(best.best.best.meet_altitude_m, 6000.0);

  // 基本健全性：代价有限
  REFUEL_EXPECT_TRUE(std::isfinite(best.best.best.cost));
  REFUEL_EXPECT_TRUE(best.best.best.cost > 0.0);
  return true;
}

// =========================
// 9) 结构回归测试：带 Fake 的集成测试（验证两层循环/调用次数/组合内选优/全局选优）
// =========================

struct FakeDiscretizer : public refuel::mode1::TankerSpeedDiscretizer {
  mutable int called = 0;
  refuel::mode1::TankerSpeedCandidates Discretize(const refuel::PlanningContext&) const override {
    called++;
    refuel::mode1::TankerSpeedCandidates out;
    out.tanker_tas_mps = {100.0, 200.0};
    return out;
  }
};

struct FakeComboGenerator : public refuel::mode1::EntryPointComboGenerator {
  mutable int called = 0;
  refuel::mode1::EntryPointCombos Generate(const refuel::PlanningContext&) const override {
    called++;
    refuel::mode1::EntryPointCombos out;

    refuel::mode1::EntryPointCombo comboA;
    comboA.choices.push_back({"R1", 0});

    refuel::mode1::EntryPointCombo comboB;
    comboB.choices.push_back({"R1", 1});

    out.combos = {comboA, comboB};
    return out;
  }
};

struct FakeCandidateGenerator : public refuel::mode1::FeasibleCandidateGenerator {
  mutable int called = 0;

  std::vector<refuel::mode1::MeetingCandidate> Generate(
      const refuel::PlanningContext&, double tanker_tas, const refuel::mode1::EntryPointCombo& combo) const override {
    called++;

    std::vector<refuel::mode1::MeetingCandidate> out;

    const int combo_id = combo.choices.empty() ? 0 : combo.choices.front().entrypoint_idx;

    // 返回两个候选（模拟“在 combo 内部选优”）
    for (double vr : {10.0, 20.0}) {
      refuel::mode1::MeetingCandidate c;
      c.feasible = true;
      c.receiver_tas_mps = vr;

      // 用 meet_tas 编码 combo_id（仅测试）
      c.meet_tas_mps = 10000.0 + combo_id;

      // 用 tanker_tas/vr 构造可重复的“时间”
      c.tanker_arrive_time_s = tanker_tas;
      c.receiver_arrive_time_s = vr;
      c.receiver_arrive_time_adjusted_s = vr;
      c.delta_t_s = c.receiver_arrive_time_s - c.tanker_arrive_time_s;

      out.push_back(c);
    }

    return out;
  }
};

struct FakeAltitudeSelector : public refuel::mode1::MeetAltitudeSelector {
  mutable int called = 0;
  void Apply(std::vector<refuel::mode1::MeetingCandidate>& cands,
             const refuel::PlanningContext&, const std::string&) const override {
    called++;
    for (auto& c : cands) {
      if (!c.feasible) continue;
      c.meet_altitude_m = 6000.0;
      c.meet_ias_mps = 120.0;
      c.altitude_change_m = 0.0;
    }
  }
};

struct FakeCostEvaluator : public refuel::mode1::CoarseCostEvaluator {
  mutable int called = 0;

  std::optional<refuel::mode1::MeetingCandidate> PickBest(
      std::vector<refuel::mode1::MeetingCandidate>& cands,
      const refuel::PlanningContext&, Preference pref) const override {
    called++;

    double best_cost = std::numeric_limits<double>::infinity();
    std::optional<refuel::mode1::MeetingCandidate> best;

    for (auto& c : cands) {
      if (!c.feasible) continue;
      const int combo_id = static_cast<int>(std::round(c.meet_tas_mps - 10000.0));

      // 构造确定性 cost（与 mode0 的 fake 测试一致）
      c.total_fuel = c.tanker_arrive_time_s + c.receiver_tas_mps + 1000.0 * combo_id;
      c.total_time = (c.tanker_arrive_time_s * 2.0) + c.receiver_tas_mps + 1000.0 * combo_id;
      c.cost = (pref == Preference::kMinFuel) ? c.total_fuel : c.total_time;

      if (c.cost < best_cost) {
        best_cost = c.cost;
        best = c;
      }
    }

    return best;
  }
};

struct FakeGlobalSelector : public refuel::mode1::GlobalSelector {
  mutable int called = 0;
  refuel::mode1::BestGlobalResult Select(const std::vector<refuel::mode1::BestUnderTankerSpeed>& all,
                                         Preference pref) const override {
    called++;
    return refuel::mode1::GlobalSelector::Select(all, pref);
  }
};

bool Test_Mode1_Integration_WithFakes() {
  using namespace refuel::mode1;

  FakeDiscretizer discretizer;
  FakeComboGenerator combo_generator;
  FakeCandidateGenerator cand_generator;
  FakeAltitudeSelector altitude_selector;
  FakeCostEvaluator cost_evaluator;
  FakeGlobalSelector global_selector;

  Mode1FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &cand_generator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  Mode1FixedSpeedSearcher searcher(deps);

  refuel::PlanningContext ctx;

  // 期望（pref=fuel）：
  // vt=100, combo_id=0, receiver=10 => cost=110 最小
  BestGlobalResult best = searcher.Solve(ctx, Preference::kMinFuel);

  REFUEL_EXPECT_TRUE(best.feasible);
  REFUEL_EXPECT_EQ(best.best.tanker_tas_mps, 100.0);
  REFUEL_EXPECT_TRUE(!best.best.combo.choices.empty());
  REFUEL_EXPECT_EQ(best.best.combo.choices.front().entrypoint_idx, 0);
  REFUEL_EXPECT_EQ(best.best.best.receiver_tas_mps, 10.0);
  REFUEL_EXPECT_EQ(best.best.best.cost, 110.0);

  // 调用次数：vt(2) * combo(2) = 4
  REFUEL_EXPECT_EQ(discretizer.called, 1);
  REFUEL_EXPECT_EQ(combo_generator.called, 1);
  REFUEL_EXPECT_EQ(cand_generator.called, 4);
  REFUEL_EXPECT_EQ(altitude_selector.called, 4);
  REFUEL_EXPECT_EQ(cost_evaluator.called, 4);
  REFUEL_EXPECT_EQ(global_selector.called, 1);

  return true;
}

} // namespace

int main() {
  using refuel::test::TestCase;

  std::vector<TestCase> cases{
      {"GlobalSelector picks min cost", Test_GlobalSelector_PicksMinCost},

      {"[Box1] TankerSpeedDiscretizer basic", Test_TankerSpeedDiscretizer_Basic},
      {"[Box2] EntryPointComboGenerator topK & cartesian", Test_EntryPointComboGenerator_TopKAndCartesian},
      {"[Box3] FeasibleCandidateGenerator solve vr & bounds", Test_FeasibleCandidateGenerator_SolveVrAndBounds},
      {"[Box5] MeetAltitudeSelector uses initial alt/fallback", Test_MeetAltitudeSelector_UsesInitialAltOrFallback},
      {"[Box6] CoarseCostEvaluator pref affects choice", Test_CoarseCostEvaluator_PrefAffectsChoice},

      {"Mode1 smoke test (no crash on empty ctx)", Test_Mode1_Smoke_NoCrashOnEmptyCtx},
      {"Mode1 end-to-end (real impl, minimal feasible)", Test_Mode1_EndToEnd_RealImpl_MinimalFeasible},
      {"Mode1 integration test (with fakes)", Test_Mode1_Integration_WithFakes},
  };

  return refuel::test::RunAll(cases);
}