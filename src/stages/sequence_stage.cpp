#include "stages/sequence_stage.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <type_traits>

namespace refuel {

namespace {

// ---- member detection (C++17 friendly) ----
template <class T, class = void>
struct has_priority_member : std::false_type {};

template <class T>
struct has_priority_member<T, std::void_t<decltype(std::declval<T>().priority)>> : std::true_type {};

template <class T>
constexpr bool has_priority_member_v = has_priority_member<T>::value;

// Returns receiver priority if the receiver object has a `priority` member, otherwise returns 0.
template <class ReceiverT>
int GetReceiverPriority(const ReceiverT& r) {
  if constexpr (has_priority_member_v<ReceiverT>) {
    // Convention: 0 means no priority; 1 is highest; larger number => lower priority.
    return static_cast<int>(r.priority);
  } else {
    return 0;
  }
}

inline int EffectivePriority(int p) {
  // Convention: priority=0 means "no priority" -> place after all positive priorities.
  if (p == 0) return std::numeric_limits<int>::max() / 2;
  return p; // smaller => higher
}

template <class VecT>
inline double Dist2XY(const VecT& a, const VecT& b) {
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return dx * dx + dy * dy;
}

inline std::mt19937_64 MakeRng() {
  std::random_device rd;
  const uint64_t t = static_cast<uint64_t>(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const uint64_t seed = (static_cast<uint64_t>(rd()) << 32) ^ t;
  return std::mt19937_64(seed);
}

} // namespace

void SequenceStage::Run(PlanningContext& ctx) {
  // TODO：实现“确定加油顺序”策略
  //
  // 常见输入：
  //   - receiver 初始位置（ctx.receivers[i].initial_position_xy）
  //   - racetrack_center（ctx.racetrack.center_xy）
  //   - priority/primary_pref 等（ctx.mission.prefs.primary_pref）
  //
  // 常见输出：
  //   - ctx.sequence.sequence：按顺序给出 receiver 的索引（0-based）
  //   - ctx.sequence.receiver_to_order：receiver_id -> 1/2/3...

  ctx.sequence.sequence.clear();
  ctx.sequence.receiver_to_order.clear();

  const int n = static_cast<int>(ctx.receivers.size());
  if (n <= 0) return;

  // 排序规则：
  //  1) 优先级高的排前面（priority=1最高，数字越大越低；priority=0视为无优先级，排在最后）
  //  2) 同优先级按距离 racetrack.center_xy 更近的排前面
  //  3) 优先级和距离都相同则随机

  // 预计算距离平方（避免开方）
  std::vector<double> dist2(n, 0.0);
  for (int i = 0; i < n; ++i) {
    dist2[i] = Dist2XY(ctx.receivers[i].initial_position_xy, ctx.racetrack.center_xy);
  }

  // 平局随机项
  auto rng = MakeRng();
  std::uniform_int_distribution<long long> dist_ll(
      std::numeric_limits<long long>::min(),
      std::numeric_limits<long long>::max());
  std::vector<long long> tie_rand(n);
  for (int i = 0; i < n; ++i) tie_rand[i] = dist_ll(rng);

  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);

  std::stable_sort(order.begin(), order.end(), [&](int i, int j) {
    const int pi = EffectivePriority(GetReceiverPriority(ctx.receivers[i]));
    const int pj = EffectivePriority(GetReceiverPriority(ctx.receivers[j]));
    if (pi != pj) return pi < pj;
    if (dist2[i] != dist2[j]) return dist2[i] < dist2[j];
    return tie_rand[i] < tie_rand[j];
  });

  // 输出：
  //   - ctx.sequence.sequence：按顺序给出 receiver 的索引（0-based）
  //   - ctx.sequence.receiver_to_order：receiver_id -> 1/2/3...
  ctx.sequence.sequence = order;
  for (int pos = 0; pos < n; ++pos) {
    const int idx = order[pos];
    ctx.sequence.receiver_to_order[ctx.receivers[idx].id] = pos + 1;
  }
}

bool SequenceStage::IsTankerOnRacetrack(const Vec3& tanker_pos_xy, const RacetrackPlan& rt) {
  (void)tanker_pos_xy;
  (void)rt;
  // TODO：判断点是否位于跑马场（或跑马场容差范围内）
  return false;
}

} // namespace refuel
