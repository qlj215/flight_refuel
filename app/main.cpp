#include <algorithm>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "branches/chuying_branch.hpp"
#include "branches/suidui_branch.hpp"
#include "io/demo_io.hpp"
#include "io/output_writer.hpp"
#include "pipeline/pipeline.hpp"

namespace fs = std::filesystem;

namespace {

using refuel::AircraftConfig;
using refuel::BranchKind;
using refuel::PlanningContext;
using refuel::RefuelAssignment;

std::string BranchName(BranchKind kind) {
  switch (kind) {
    case BranchKind::kFollow:
      return "suidui";
    case BranchKind::kIntercept:
      return "chuying";
    case BranchKind::kOneToManyFixed:
      return "one_to_many_fixed";
    case BranchKind::kManyToMany:
      return "many_to_many";
    default:
      return "unknown";
  }
}

const AircraftConfig* FindTanker(const PlanningContext& ctx, const std::string& tanker_id) {
  auto it = std::find_if(ctx.tankers.begin(), ctx.tankers.end(), [&](const AircraftConfig& tk) {
    return tk.id == tanker_id;
  });
  return (it == ctx.tankers.end()) ? nullptr : &(*it);
}

std::vector<AircraftConfig> PickReceivers(const PlanningContext& ctx,
                                          const std::vector<std::string>& receiver_ids) {
  std::vector<AircraftConfig> out;
  for (const auto& rid : receiver_ids) {
    auto it = std::find_if(ctx.receivers.begin(), ctx.receivers.end(), [&](const AircraftConfig& r) {
      return r.id == rid;
    });
    if (it != ctx.receivers.end()) out.push_back(*it);
  }
  return out;
}

void RunSingleBranch(PlanningContext& ctx, const std::string& output_dir) {
  refuel::Pipeline pipe;
  pipe.Run(ctx);
  refuel::io::OutputWriter::WriteAll(ctx, output_dir);
}

void WriteJsonFile(const nlohmann::json& j, const std::string& output_dir, const std::string& filename) {
  fs::create_directories(output_dir);
  std::ofstream ofs(fs::path(output_dir) / filename);
  ofs << j.dump(2) << "\n";
}

void WriteBranchInterfacePlaceholder(const PlanningContext& ctx, const std::string& output_dir) {
  nlohmann::json j;
  j["branch"] = BranchName(ctx.mission.branch_kind);
  j["status"] = "interface_only";
  j["implemented"] = false;
  j["message"] = "This branch currently keeps only the scheduling interface. Internal logic will be implemented later.";
  j["mission_id"] = ctx.mission.mission_id;
  j["tanker_ids"] = ctx.mission.tanker_ids;
  j["receiver_ids"] = ctx.mission.receiver_ids;
  WriteJsonFile(j, output_dir, "branch_interface.json");
}

void WriteManyToManySummary(const nlohmann::json& j, const std::string& output_dir) {
  WriteJsonFile(j, output_dir, "summary.json");
}

std::size_t SafeZoneCount(const PlanningContext& ctx) {
  if (!ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.empty()) {
    return ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.size();
  }
  return ctx.mission.refuel_mode.safe_zone.zone_ids.size();
}

bool ParseDateTime(const std::string& s, std::tm& tm_out) {
  std::tm tm{};
  std::istringstream iss(s);
  iss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
  if (iss.fail()) return false;
  tm_out = tm;
  return true;
}

bool ParseHMS(const std::string& s, int& h, int& m, int& sec) {
  char c1 = '\0', c2 = '\0';
  std::istringstream iss(s);
  iss >> h >> c1 >> m >> c2 >> sec;
  if (iss.fail() || c1 != ':' || c2 != ':') return false;
  return (h >= 0 && h < 24 && m >= 0 && m < 60 && sec >= 0 && sec < 60);
}

std::string FormatTime(std::time_t t) {
  std::tm tmv{};
#if defined(_WIN32)
  localtime_s(&tmv, &t);
#else
  localtime_r(&t, &tmv);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tmv, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

void CheckDesignatedDeadline(const PlanningContext& base,
                             double global_latest_t_s,
                             nlohmann::json& summary) {
  summary["creation_time"] = base.mission.creation_time;
  summary["designated_time"] = base.mission.refuel_mode.designated_time;
  summary["global_latest_receiver_start_refuel_s"] = global_latest_t_s;

  if (base.mission.creation_time.empty() || base.mission.refuel_mode.designated_time.empty()) {
    summary["deadline_check"] = "skipped_missing_time_fields";
    return;
  }

  std::tm creation_tm{};
  if (!ParseDateTime(base.mission.creation_time, creation_tm)) {
    throw std::runtime_error("Failed to parse creation_time: " + base.mission.creation_time);
  }

  int hh = 0, mm = 0, ss = 0;
  if (!ParseHMS(base.mission.refuel_mode.designated_time, hh, mm, ss)) {
    throw std::runtime_error("Failed to parse designated_time(HH:MM:SS): " + base.mission.refuel_mode.designated_time);
  }

  std::tm designated_tm = creation_tm;
  designated_tm.tm_hour = hh;
  designated_tm.tm_min = mm;
  designated_tm.tm_sec = ss;

  std::time_t creation_epoch = std::mktime(&creation_tm);
  std::time_t designated_epoch = std::mktime(&designated_tm);
  if (creation_epoch == static_cast<std::time_t>(-1) || designated_epoch == static_cast<std::time_t>(-1)) {
    throw std::runtime_error("Failed to convert mission times to epoch.");
  }

  const std::time_t actual_epoch = creation_epoch + static_cast<std::time_t>(global_latest_t_s + 0.5);

  summary["actual_latest_refuel_start_time"] = FormatTime(actual_epoch);
  summary["designated_time_absolute"] = FormatTime(designated_epoch);
  summary["deadline_ok"] = (actual_epoch <= designated_epoch);

  if (actual_epoch > designated_epoch) {
    throw std::runtime_error(
        "Designated time violated (replan required): actual_latest_refuel_start_time=" +
        FormatTime(actual_epoch) + ", designated_time=" + FormatTime(designated_epoch));
  }
}

} // namespace

int main(int argc, char** argv) {
  std::string input_dir = "demo/input/0210";
  std::string output_dir = "demo/output/0210";

  if (argc >= 2) input_dir = argv[1];
  if (argc >= 3) output_dir = argv[2];

  try {
    PlanningContext base = refuel::io::DemoIO::LoadContext(input_dir);

    std::cout << "Detected branch: " << BranchName(base.mission.branch_kind) << "\n";

    if (base.mission.branch_kind == BranchKind::kFollow) {
      std::string err;
      if (!refuel::branch::RunSuiduiBranch(base, output_dir, &err)) {
        throw std::runtime_error("Suidui branch failed: " + err);
      }
      std::cout << "Suidui branch output written to: " << output_dir << "\n";
      return 0;
    }

    if (base.mission.branch_kind == BranchKind::kIntercept) {
      std::string err;
      if (!refuel::branch::RunChuyingBranch(base, output_dir, &err)) {
        throw std::runtime_error("Chuying branch failed: " + err);
      }
      std::cout << "Chuying branch output written to: " << output_dir << "\n";
      return 0;
    }

    if (base.mission.branch_kind != BranchKind::kManyToMany) {
      RunSingleBranch(base, output_dir);
      std::cout << "Done. Output written to: " << output_dir << "\n";
      return 0;
    }

    nlohmann::json summary;
    summary["branch"] = BranchName(base.mission.branch_kind);
    summary["input_dir"] = input_dir;
    summary["subtasks"] = nlohmann::json::array();
    summary["warnings"] = nlohmann::json::array();

    const std::size_t zone_count = SafeZoneCount(base);
    summary["safe_zone_count"] = zone_count;

    std::vector<int> excluded_safe_zone_ids;
    std::set<int> used_safe_zone_ids_8000;

    double current_altitude_m = 7000.0;
    summary["altitude_policy"] = "7000->8000->9000(cap)";

    int subtask_idx = 0;
    double global_latest_t_s = 0.0;

    for (const RefuelAssignment& asg : base.mission.assignments) {
      ++subtask_idx;
      const AircraftConfig* tanker = FindTanker(base, asg.tanker_id);
      if (tanker == nullptr) {
        summary["warnings"].push_back("Assignment references unknown tanker: " + asg.tanker_id);
        continue;
      }

      if (zone_count > 0 && excluded_safe_zone_ids.size() >= zone_count) {
        if (current_altitude_m < 8000.0) {
          current_altitude_m = 8000.0;
          excluded_safe_zone_ids.clear();
        } else if (current_altitude_m < 9000.0) {
          if (used_safe_zone_ids_8000.size() >= zone_count) {
            current_altitude_m = 9000.0;
            excluded_safe_zone_ids.clear();
          } else {
            throw std::runtime_error(
                "8000m safe-zone cycle is exhausted before all zones were used once. "
                "Cannot continue with current policy.");
          }
        } else {
          throw std::runtime_error("9000m safe zones are exhausted. Replan required.");
        }
      }

      PlanningContext sub = base;
      sub.tanker = *tanker;
      sub.mission.tanker_id = tanker->id;
      sub.mission.tanker_ids = {tanker->id};
      sub.receivers = PickReceivers(base, asg.receiver_ids);
      sub.mission.receiver_ids = asg.receiver_ids;
      sub.mission.branch_kind = BranchKind::kOneToManyFixed;
      sub.excluded_safezone_ids = excluded_safe_zone_ids;
      sub.mission.racecourse.altitude_m = current_altitude_m;

      if (sub.receivers.empty()) {
        summary["warnings"].push_back("Assignment for tanker " + asg.tanker_id + " has no resolvable receivers.");
        continue;
      }

      const fs::path subdir = fs::path(output_dir) / (std::to_string(subtask_idx) + "_" + tanker->id);
      RunSingleBranch(sub, subdir.string());

      if (sub.safe_zone_selected.chosen_safezone_id >= 0) {
        excluded_safe_zone_ids.push_back(sub.safe_zone_selected.chosen_safezone_id);
        if (current_altitude_m >= 7999.5 && current_altitude_m < 9000.0) {
          used_safe_zone_ids_8000.insert(sub.safe_zone_selected.chosen_safezone_id);
        }
      }

      global_latest_t_s = std::max(global_latest_t_s, sub.cost.receiver_total_arrival_time_s);

      nlohmann::json item;
      item["index"] = subtask_idx;
      item["tanker_id"] = tanker->id;
      item["receiver_ids"] = asg.receiver_ids;
      item["altitude_m_used"] = current_altitude_m;
      item["chosen_safezone_id"] = sub.safe_zone_selected.chosen_safezone_id;
      item["excluded_safezone_ids_after_run"] = excluded_safe_zone_ids;
      item["global_latest_t_s_so_far"] = global_latest_t_s;
      item["output_dir"] = subdir.string();
      summary["subtasks"].push_back(item);
    }

    CheckDesignatedDeadline(base, global_latest_t_s, summary);

    WriteManyToManySummary(summary, output_dir);
    std::cout << "Done. Many-to-many outputs written to: " << output_dir << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
