#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

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

} // namespace

int main(int argc, char** argv) {
  std::string input_dir = "demo/input/0210";
  std::string output_dir = "demo/output/0210";

  if (argc >= 2) input_dir = argv[1];
  if (argc >= 3) output_dir = argv[2];

  try {
    PlanningContext base = refuel::io::DemoIO::LoadContext(input_dir);

    std::cout << "Detected branch: " << BranchName(base.mission.branch_kind) << "\n";

    if (base.mission.branch_kind == BranchKind::kFollow ||
        base.mission.branch_kind == BranchKind::kIntercept) {
      WriteBranchInterfacePlaceholder(base, output_dir);
      std::cout << "Branch interface written to: " << output_dir << "\n";
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

    std::vector<int> excluded_safe_zone_ids;
    int subtask_idx = 0;

    for (const RefuelAssignment& asg : base.mission.assignments) {
      ++subtask_idx;
      const AircraftConfig* tanker = FindTanker(base, asg.tanker_id);
      if (tanker == nullptr) {
        summary["warnings"].push_back("Assignment references unknown tanker: " + asg.tanker_id);
        continue;
      }

      PlanningContext sub = base;
      sub.tanker = *tanker;
      sub.mission.tanker_id = tanker->id;
      sub.mission.tanker_ids = {tanker->id};
      sub.receivers = PickReceivers(base, asg.receiver_ids);
      sub.mission.receiver_ids = asg.receiver_ids;
      sub.mission.branch_kind = BranchKind::kOneToManyFixed;
      sub.excluded_safezone_ids = excluded_safe_zone_ids;

      if (sub.receivers.empty()) {
        summary["warnings"].push_back("Assignment for tanker " + asg.tanker_id + " has no resolvable receivers.");
        continue;
      }

      const fs::path subdir = fs::path(output_dir) / (std::to_string(subtask_idx) + "_" + tanker->id);
      RunSingleBranch(sub, subdir.string());

      if (sub.safe_zone_selected.chosen_safezone_id >= 0) {
        excluded_safe_zone_ids.push_back(sub.safe_zone_selected.chosen_safezone_id);
      }

      nlohmann::json item;
      item["index"] = subtask_idx;
      item["tanker_id"] = tanker->id;
      item["receiver_ids"] = asg.receiver_ids;
      item["chosen_safezone_id"] = sub.safe_zone_selected.chosen_safezone_id;
      item["excluded_safezone_ids_after_run"] = excluded_safe_zone_ids;
      item["output_dir"] = subdir.string();
      summary["subtasks"].push_back(item);
    }

    WriteManyToManySummary(summary, output_dir);
    std::cout << "Done. Many-to-many outputs written to: " << output_dir << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
