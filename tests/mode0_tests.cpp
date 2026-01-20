#include "tests/test_framework.hpp"

#include <string>
#include <vector>

#include "mode0/mode0_fixed_speed_search.hpp"

using refuel::mode0::Preference;

namespace {

// =========================
// 1) 单元测试：白框 9 GlobalSelector
// 目的：验证全局选优逻辑没问题（这部分代码不是 TODO，因此可以直接单测）。
// =========================
bool Test_GlobalSelector_PicksMinCost() {
  using namespace refuel::mode0;

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
// 2) 冒烟测试：白框流程“跑通不崩溃”
// 目的：即使各白框实现仍是 TODO（返回空），Solve() 也不应崩溃。
// =========================
bool Test_Mode0_Smoke_NoCrashOnEmptyImpl() {
  using namespace refuel::mode0;

  // 依赖全部使用“默认空实现”
  TankerSpeedDiscretizer tanker_speed_discretizer;
  EntryPointComboGenerator combo_generator;
  FeasibleCandidateGenerator candidate_generator;
  HoldingLoopCalculator loop_calculator;
  MeetAltitudeSelector altitude_selector;
  CoarseCostEvaluator cost_evaluator;
  GlobalSelector global_selector;

  Mode0FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &tanker_speed_discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &candidate_generator;
  deps.loop_calculator = &loop_calculator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  Mode0FixedSpeedSearcher searcher(deps);

  refuel::PlanningContext ctx; // 空 ctx 也应该能安全返回（因为实现里不会真正访问）
  std::vector<double> receiver_candidates;
  BestGlobalResult best = searcher.Solve(ctx, Preference::kMinFuel, receiver_candidates);

  // 由于实现为空，一般 infeasible；但至少要保证函数能返回。
  REFUEL_EXPECT_TRUE(!best.feasible);
  return true;
}

// =========================
// 3) 联合测试（集成测试）：用“假实现”验证白框流程的两层循环 & 选优逻辑
// 目的：不依赖真实算法，把每个白框的输出“钉死”，检查：
//  - 白框 1 的 vt 遍历（外层循环）
//  - 白框 2 的 combo 遍历（内层循环）
//  - 白框 6 的 combo 内选优（受油机真速由 cost 决定）
//  - 白框 9 的全局选优
// =========================

// 下面这些 FakeXXX 都是“测试专用桩”，真实工程里不需要。

struct FakeDiscretizer : public refuel::mode0::TankerSpeedDiscretizer {
  mutable int called = 0;
  refuel::mode0::TankerSpeedCandidates Discretize(const refuel::PlanningContext&) const override {
    called++;
    refuel::mode0::TankerSpeedCandidates out;
    out.tanker_tas_mps = {100.0, 200.0};
    return out;
  }
};

struct FakeComboGenerator : public refuel::mode0::EntryPointComboGenerator {
  mutable int called = 0;
  refuel::mode0::EntryPointCombos Generate(const refuel::PlanningContext&) const override {
    called++;
    refuel::mode0::EntryPointCombos out;

    // comboA: receiver R1 选 entry 0
    refuel::mode0::EntryPointCombo comboA;
    comboA.choices.push_back({"R1", 0});

    // comboB: receiver R1 选 entry 1
    refuel::mode0::EntryPointCombo comboB;
    comboB.choices.push_back({"R1", 1});

    out.combos = {comboA, comboB};
    return out;
  }
};

struct FakeCandidateGenerator : public refuel::mode0::FeasibleCandidateGenerator {
  mutable int called = 0;

  std::vector<refuel::mode0::MeetingCandidate> Generate(
      const refuel::PlanningContext&, double tanker_tas, const refuel::mode0::EntryPointCombo& combo,
      const std::vector<double>& receiver_candidates) const override {
    called++;
    std::vector<refuel::mode0::MeetingCandidate> out;

    // 仅用于测试：把 combo_id 编码进 meet_tas（真实算法不要这样做）
    const int combo_id = combo.choices.empty() ? 0 : combo.choices.front().entrypoint_idx;

    // 为了验证“白框 6 负责选择受油机真速”，这里故意返回多个 candidate
    for (double vr : receiver_candidates) {
      refuel::mode0::MeetingCandidate c;
      c.feasible = true;
      c.receiver_tas_mps = vr;
      c.meet_tas_mps = 10000.0 + combo_id; // combo_id 编码
      c.tanker_arrive_time_s = tanker_tas; // 随便填，loop_calculator 也可不使用
      c.receiver_arrive_time_s = vr;
      c.delta_t_s = c.receiver_arrive_time_s - c.tanker_arrive_time_s;
      out.push_back(c);
    }

    // 如果 receiver_candidates 为空，返回空表示该 combo 不可行
    return out;
  }
};

struct FakeLoopCalculator : public refuel::mode0::HoldingLoopCalculator {
  mutable int called = 0;
  void Apply(std::vector<refuel::mode0::MeetingCandidate>& cands, double loop_period_s) const override {
    called++;
    // 测试用：简单按公式写回（不依赖真实跑马场）
    // loop_period_s 在测试里传 10
    for (auto& c : cands) {
      if (!c.feasible) continue;
      if (c.delta_t_s >= 0) {
        c.n_loops = 0;
        c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s;
      } else {
        c.n_loops = static_cast<int>(std::ceil((-c.delta_t_s) / loop_period_s));
        c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s + c.n_loops * loop_period_s;
      }
    }
  }
};

struct FakeAltitudeSelector : public refuel::mode0::MeetAltitudeSelector {
  mutable int called = 0;
  void Apply(std::vector<refuel::mode0::MeetingCandidate>& cands,
             const refuel::PlanningContext&, const std::string&) const override {
    called++;
    // 测试用：随便填一个高度/表速，让白框 6 有完整字段可用
    for (auto& c : cands) {
      if (!c.feasible) continue;
      c.meet_altitude_m = 6000.0;
      c.meet_ias_mps = 120.0;
      c.altitude_change_m = 0.0;
    }
  }
};

struct FakeCostEvaluator : public refuel::mode0::CoarseCostEvaluator {
  mutable int called = 0;

  std::optional<refuel::mode0::MeetingCandidate> PickBest(
      std::vector<refuel::mode0::MeetingCandidate>& cands,
      const refuel::PlanningContext&, Preference pref) const override {
    called++;
    // 测试用：构造一个确定性的 cost，让我们能预测全局最优。
    // cost = tanker_tas + receiver_tas + 1000*combo_id
    // combo_id 从 meet_tas_mps(10000+combo_id) 解码。

    double best_cost = std::numeric_limits<double>::infinity();
    std::optional<refuel::mode0::MeetingCandidate> best;

    for (auto& c : cands) {
      if (!c.feasible) continue;
      const int combo_id = static_cast<int>(std::round(c.meet_tas_mps - 10000.0));

      // 让 pref 也参与一下：时间优先时就用 total_time；油耗优先用 total_fuel。
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

struct FakeGlobalSelector : public refuel::mode0::GlobalSelector {
  mutable int called = 0;
  refuel::mode0::BestGlobalResult Select(const std::vector<refuel::mode0::BestUnderTankerSpeed>& all,
                                         Preference pref) const override {
    called++;
    return refuel::mode0::GlobalSelector::Select(all, pref);
  }
};

bool Test_Mode0_Integration_WithFakes() {
  using namespace refuel::mode0;

  FakeDiscretizer discretizer;
  FakeComboGenerator combo_generator;
  FakeCandidateGenerator cand_generator;
  FakeLoopCalculator loop_calculator;
  FakeAltitudeSelector altitude_selector;
  FakeCostEvaluator cost_evaluator;
  FakeGlobalSelector global_selector;

  Mode0FixedSpeedSearcher::Dependencies deps;
  deps.tanker_speed_discretizer = &discretizer;
  deps.combo_generator = &combo_generator;
  deps.candidate_generator = &cand_generator;
  deps.loop_calculator = &loop_calculator;
  deps.altitude_selector = &altitude_selector;
  deps.cost_evaluator = &cost_evaluator;
  deps.global_selector = &global_selector;

  Mode0FixedSpeedSearcher searcher(deps);

  refuel::PlanningContext ctx;

  // receiver 候选速度：刻意放 2 个，让白框 6 在这两个里“选最优”
  std::vector<double> receiver_candidates{10.0, 20.0};

  // 期望：
  // vt=100, combo_id=0( entrypoint_idx=0 ), receiver=10 => cost=100+10+0=110 （最小）
  // vt=100, combo_id=0, receiver=20 => cost=120
  // vt=100, combo_id=1, receiver=10 => cost=1110
  // vt=200, combo_id=0, receiver=10 => cost=210
  // 因此全局最优应为 vt=100, combo_id=0, receiver_tas=10
  BestGlobalResult best = searcher.Solve(ctx, Preference::kMinFuel, receiver_candidates);

  REFUEL_EXPECT_TRUE(best.feasible);
  REFUEL_EXPECT_EQ(best.best.tanker_tas_mps, 100.0);
  REFUEL_EXPECT_TRUE(!best.best.combo.choices.empty());
  REFUEL_EXPECT_EQ(best.best.combo.choices.front().entrypoint_idx, 0);
  REFUEL_EXPECT_EQ(best.best.best.receiver_tas_mps, 10.0);
  REFUEL_EXPECT_EQ(best.best.best.cost, 110.0);

  // 顺带验证两层循环触发次数：vt(2) * combo(2) = 4 次 Generate
  REFUEL_EXPECT_EQ(discretizer.called, 1);
  REFUEL_EXPECT_EQ(combo_generator.called, 1);
  REFUEL_EXPECT_EQ(cand_generator.called, 4);
  REFUEL_EXPECT_EQ(cost_evaluator.called, 4);

  return true;
}

} // namespace

int main() {
  using refuel::test::TestCase;

  std::vector<TestCase> cases{
      {"GlobalSelector picks min cost", Test_GlobalSelector_PicksMinCost},
      {"Mode0 smoke test (no crash on empty impl)", Test_Mode0_Smoke_NoCrashOnEmptyImpl},
      {"Mode0 integration test (with fakes)", Test_Mode0_Integration_WithFakes},
  };

  return refuel::test::RunAll(cases);
}
