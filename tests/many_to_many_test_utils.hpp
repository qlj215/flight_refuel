#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/types.hpp"

namespace refuel::test {

inline const AircraftConfig* FindTankerById(const PlanningContext& ctx, const std::string& tanker_id) {
  auto it = std::find_if(ctx.tankers.begin(), ctx.tankers.end(), [&](const AircraftConfig& tk) {
    return tk.id == tanker_id;
  });
  return (it == ctx.tankers.end()) ? nullptr : &(*it);
}

inline std::vector<AircraftConfig> PickReceiversByIds(const PlanningContext& ctx,
                                                      const std::vector<std::string>& receiver_ids) {
  std::vector<AircraftConfig> out;
  out.reserve(receiver_ids.size());
  for (const auto& rid : receiver_ids) {
    auto it = std::find_if(ctx.receivers.begin(), ctx.receivers.end(), [&](const AircraftConfig& r) {
      return r.id == rid;
    });
    if (it != ctx.receivers.end()) out.push_back(*it);
  }
  return out;
}

inline PlanningContext MakeOneToManySubContext(const PlanningContext& base,
                                               const RefuelAssignment& asg,
                                               const std::vector<int>& excluded_safezone_ids) {
  const AircraftConfig* tanker = FindTankerById(base, asg.tanker_id);
  if (tanker == nullptr) {
    throw std::runtime_error("Unknown tanker in assignment: " + asg.tanker_id);
  }

  PlanningContext sub = base;
  sub.tanker = *tanker;
  sub.mission.tanker_id = tanker->id;
  sub.mission.tanker_ids = {tanker->id};
  sub.receivers = PickReceiversByIds(base, asg.receiver_ids);
  sub.mission.receiver_ids = asg.receiver_ids;
  sub.mission.branch_kind = BranchKind::kOneToManyFixed;
  sub.excluded_safezone_ids = excluded_safezone_ids;
  return sub;
}

}  // namespace refuel::test
