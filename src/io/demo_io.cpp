#include "io/demo_io.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace refuel::io {
namespace {

using json = nlohmann::json;

std::string ReadAllText(const fs::path& p) {
  std::ifstream ifs(p, std::ios::in | std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("Failed to open file: " + p.string());
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

std::string ReadAllTextIfExists(const fs::path& p) {
  if (!fs::exists(p)) return {};
  return ReadAllText(p);
}

json ParseJson(const std::string& text, const std::string& hint) {
  try {
    return json::parse(text);
  } catch (const std::exception& e) {
    throw std::runtime_error("JSON parse failed for " + hint + ": " + std::string(e.what()));
  }
}

std::vector<std::string> SplitWS(const std::string& s) {
  std::istringstream iss(s);
  std::vector<std::string> out;
  for (std::string tok; iss >> tok;) out.push_back(tok);
  return out;
}

std::string ToLowerAscii(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool ContainsAscii(const std::string& s, const std::string& needle) {
  return ToLowerAscii(s).find(ToLowerAscii(needle)) != std::string::npos;
}

std::vector<LLA> ParseLLAPolygon(const json& arr) {
  std::vector<LLA> poly;
  if (!arr.is_array()) return poly;
  for (const auto& v : arr) {
    if (v.is_array() && v.size() >= 2 && v[0].is_number() && v[1].is_number()) {
      poly.push_back(LLA{v.at(0).get<double>(), v.at(1).get<double>(), 0.0});
    }
  }
  return poly;
}

void ParseZoneCollection(const json& zones_json,
                         const char* id_key,
                         SafeZoneInput& out_safe) {
  out_safe.zone_ids.clear();
  out_safe.zones_vertices_lla.clear();
  if (!zones_json.is_object()) return;
  out_safe.is_defined = zones_json.value("is_defined", false);
  if (!out_safe.is_defined) return;

  auto push_zone = [&](int default_id, const json& zone_obj, const json& vertices_json) {
    std::vector<LLA> poly = ParseLLAPolygon(vertices_json);
    if (poly.empty()) return;
    const int zone_id = zone_obj.is_object() ? zone_obj.value(id_key, default_id) : default_id;
    out_safe.zone_ids.push_back(zone_id);
    out_safe.zones_vertices_lla.push_back(std::move(poly));
  };

  if (zones_json.contains("zones") && zones_json["zones"].is_array()) {
    int idx = 0;
    for (const auto& zone : zones_json["zones"]) {
      ++idx;
      if (!zone.is_object()) continue;
      push_zone(idx, zone, zone.value("vertices", json::array()));
    }
  }
  if (zones_json.contains("safe_zones_vertices") && zones_json["safe_zones_vertices"].is_array()) {
    int idx = 0;
    for (const auto& zone : zones_json["safe_zones_vertices"]) {
      ++idx;
      if (!zone.is_object()) continue;
      push_zone(idx, zone, zone.value("vertices", json::array()));
    }
  }
  if (out_safe.zones_vertices_lla.empty() && zones_json.contains("vertices")) {
    std::vector<LLA> poly = ParseLLAPolygon(zones_json["vertices"]);
    if (!poly.empty()) {
      out_safe.zone_ids.push_back(1);
      out_safe.zones_vertices_lla.push_back(std::move(poly));
    }
  }
}

void ParseNoFlyZoneCollection(const json& zones_json,
                              const char* id_key,
                              NoFlyZoneInput& out_no_fly) {
  out_no_fly.zone_ids.clear();
  out_no_fly.zones_vertices_lla.clear();
  if (!zones_json.is_object()) return;

  // Support both explicit and implicit no-fly definitions:
  // some mission files provide vertices/zones without is_defined=true.
  const bool has_zone_payload =
      (zones_json.contains("zones") && zones_json["zones"].is_array()) ||
      (zones_json.contains("no_fly_zones_vertices") && zones_json["no_fly_zones_vertices"].is_array()) ||
      zones_json.contains("vertices");
  out_no_fly.is_defined = zones_json.value("is_defined", has_zone_payload);
  if (!out_no_fly.is_defined) return;

  auto push_zone = [&](int default_id, const json& zone_obj, const json& vertices_json) {
    std::vector<LLA> poly = ParseLLAPolygon(vertices_json);
    if (poly.empty()) return;
    const int zone_id = zone_obj.is_object() ? zone_obj.value(id_key, default_id) : default_id;
    out_no_fly.zone_ids.push_back(zone_id);
    out_no_fly.zones_vertices_lla.push_back(std::move(poly));
  };

  if (zones_json.contains("zones") && zones_json["zones"].is_array()) {
    int idx = 0;
    for (const auto& zone : zones_json["zones"]) {
      ++idx;
      if (!zone.is_object()) continue;
      push_zone(idx, zone, zone.value("vertices", json::array()));
    }
  }
  if (zones_json.contains("no_fly_zones_vertices") && zones_json["no_fly_zones_vertices"].is_array()) {
    int idx = 0;
    for (const auto& zone : zones_json["no_fly_zones_vertices"]) {
      ++idx;
      if (!zone.is_object()) continue;
      push_zone(idx, zone, zone.value("vertices", json::array()));
    }
  }
  if (out_no_fly.zones_vertices_lla.empty() && zones_json.contains("vertices")) {
    std::vector<LLA> poly = ParseLLAPolygon(zones_json["vertices"]);
    if (!poly.empty()) {
      out_no_fly.zone_ids.push_back(1);
      out_no_fly.zones_vertices_lla.push_back(std::move(poly));
    }
  }
}

std::vector<fs::path> CollectConfigPaths(const fs::path& dir, const std::string& needle) {
  std::vector<fs::path> out;
  if (!fs::exists(dir)) return out;
  for (const auto& ent : fs::directory_iterator(dir)) {
    if (!ent.is_regular_file()) continue;
    const std::string name = ToLowerAscii(ent.path().filename().string());
    if (name.size() < 5 || name.substr(name.size() - 5) != ".json") continue;
    if (name.find(needle) != std::string::npos && name.find("config") != std::string::npos) {
      out.push_back(ent.path());
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

AircraftConfig ParseAircraft(const json& root, const std::string& id_key, const std::string& type_key) {
  AircraftConfig ac;
  ac.id = root.value(id_key, ac.id);
  ac.type = root.value(type_key, ac.type);

  const json init = root.value("initial_position", json::object());
  ac.initial_position_lla = LLA{
      init.value("latitude", 0.0),
      init.value("longitude", 0.0),
      init.value("altitude", 0.0),
  };

  const json cpt = root.value("cruise_performance_table", json::object());
  ac.cruise_perf.altitude_levels = cpt.value("altitude_levels", ac.cruise_perf.altitude_levels);
  ac.cruise_perf.speed_levels = cpt.value("speed_levels", ac.cruise_perf.speed_levels);
  ac.cruise_perf.true_airspeed_data = cpt.value("true_airspeed_data", ac.cruise_perf.true_airspeed_data);

  const json ft = root.value("fuel_consumption_table", json::object());
  ac.fuel_table.altitude_levels = ft.value("altitude_levels", ac.fuel_table.altitude_levels);
  ac.fuel_table.speed_levels = ft.value("speed_levels", ac.fuel_table.speed_levels);
  ac.fuel_table.consumption_data = ft.value("consumption_data", ac.fuel_table.consumption_data);

  const json cs = root.value("current_status", json::object());
  ac.current_status.fuel_kg = cs.value("fuel_kg", ac.current_status.fuel_kg);
  ac.current_status.heading_deg = cs.value("heading_deg", ac.current_status.heading_deg);
  ac.current_status.speed_mps = cs.value("speed_mps", ac.current_status.speed_mps);
  ac.current_status.max_fuel_kg = cs.value("max_fuel_kg", ac.current_status.max_fuel_kg);
  ac.current_status.priority = cs.value("priority", ac.current_status.priority);
  ac.current_status.timestamp = cs.value("timestamp", ac.current_status.timestamp);

  const json lim = root.value("performance_limits", json::object());
  ac.limits.climb_rate = lim.value("climb_rate", ac.limits.climb_rate);
  ac.limits.descent_rate = lim.value("descent_rate", ac.limits.descent_rate);
  ac.limits.max_altitude = lim.value("max_altitude", ac.limits.max_altitude);
  ac.limits.max_bank_angle_deg = lim.value("max_bank_angle_deg", ac.limits.max_bank_angle_deg);
  if (lim.contains("max_bank_angle") && lim["max_bank_angle"].is_number()) {
    ac.limits.max_bank_angle_deg = lim["max_bank_angle"].get<double>();
  }
  ac.limits.fuel_transfer_speed = lim.value("fuel_transfer_speed", ac.limits.fuel_transfer_speed);

  ac.max_fuel_kg = root.value("max_fuel_kg", ac.max_fuel_kg);
  if (ac.max_fuel_kg <= 0.0) ac.max_fuel_kg = ac.current_status.max_fuel_kg;

  ac.priority = root.value("priority", ac.priority);
  if (ac.priority == 0) ac.priority = ac.current_status.priority;

  ac.fuel_transfer_speed = root.value("fuel_transfer_speed", ac.fuel_transfer_speed);
  if (ac.fuel_transfer_speed <= 0.0) ac.fuel_transfer_speed = ac.limits.fuel_transfer_speed;
  ac.fuel_transfer_speed_kgps = ac.fuel_transfer_speed;

  return ac;
}

BranchKind DeduceBranchKind(const PlanningContext& ctx) {
  std::unordered_set<std::string> tanker_ids(ctx.mission.tanker_ids.begin(), ctx.mission.tanker_ids.end());
  for (const auto& a : ctx.mission.assignments) {
    if (!a.tanker_id.empty()) tanker_ids.insert(a.tanker_id);
  }
  const bool multi_tanker = tanker_ids.size() > 1 || ctx.tankers.size() > 1;

  const std::string sub = ToLowerAscii(ctx.mission.refuel_mode.sub_mode);
  if (sub.find("suidui") != std::string::npos || sub.find("follow") != std::string::npos ||
      ctx.mission.refuel_mode.sub_mode.find("随队") != std::string::npos) {
    return BranchKind::kFollow;
  }
  if (sub.find("chuying") != std::string::npos || sub.find("intercept") != std::string::npos ||
      sub.find("escort") != std::string::npos ||
      ctx.mission.refuel_mode.sub_mode.find("出迎") != std::string::npos) {
    return BranchKind::kIntercept;
  }
  if (sub.find("dingdian") != std::string::npos || sub.find("fixed") != std::string::npos ||
      ctx.mission.refuel_mode.sub_mode.find("定点") != std::string::npos) {
    return multi_tanker ? BranchKind::kManyToMany : BranchKind::kOneToManyFixed;
  }
  return multi_tanker ? BranchKind::kManyToMany : BranchKind::kOneToManyFixed;
}

} // namespace

PlanningContext DemoIO::LoadContext(const std::string& input_dir) {
  PlanningContext ctx;
  const fs::path dir(input_dir);

  // ---------- locate mission ----------
  std::string mission_text = ReadAllTextIfExists(dir / "mission1.json");
  if (mission_text.empty()) mission_text = ReadAllTextIfExists(dir / "mission4.json");
  if (mission_text.empty()) {
    for (const auto& ent : fs::directory_iterator(dir)) {
      if (!ent.is_regular_file()) continue;
      const auto name = ToLowerAscii(ent.path().filename().string());
      if (ContainsAscii(name, "mission") && name.size() >= 5 && name.substr(name.size() - 5) == ".json") {
        mission_text = ReadAllText(ent.path());
        break;
      }
    }
  }
  if (mission_text.empty()) {
    throw std::runtime_error("No mission json found in: " + dir.string());
  }

  // ---------- parse mission ----------
  const json mission_root = ParseJson(mission_text, "mission");
  const json mission0 = (mission_root.is_array() && !mission_root.empty()) ? mission_root.at(0) : mission_root;

  ctx.mission.mission_id = mission0.value("mission_id", ctx.mission.mission_id);
  ctx.mission.creation_time = mission0.value("creation_time", ctx.mission.creation_time);

  const json units = mission0.value("units", json::object());
  ctx.mission.tanker_ids = SplitWS(units.value("tanker_id", ""));
  if (!ctx.mission.tanker_ids.empty()) ctx.mission.tanker_id = ctx.mission.tanker_ids.front();
  ctx.mission.receiver_ids = SplitWS(units.value("receiver_id", ""));

  const json refuel_mode = mission0.value("refuel_mode", json::object());
  ctx.mission.refuel_mode.mode_type = refuel_mode.value("mode_type", ctx.mission.refuel_mode.mode_type);

  const json* rm_src_ptr = &refuel_mode;
  if (refuel_mode.contains("timing_params") && refuel_mode["timing_params"].is_object()) {
    rm_src_ptr = &refuel_mode["timing_params"];
  }
  const json& rm_src = *rm_src_ptr;

  ctx.mission.refuel_mode.sub_mode = rm_src.value("sub_mode", ctx.mission.refuel_mode.sub_mode);
  ctx.mission.refuel_mode.designated_time =
      rm_src.value("designated_time", ctx.mission.refuel_mode.designated_time);

  ParseZoneCollection(rm_src.value("safe_zone", json::object()), "safe_zone_id",
                      ctx.mission.refuel_mode.safe_zone);
  if (ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.empty()) {
    ParseZoneCollection(rm_src.value("safe_zones", json::object()), "safe_zone_id",
                        ctx.mission.refuel_mode.safe_zone);
  }

  const json opt_prefs = mission0.value("optimization_prefs", json::object());
  ctx.mission.prefs.primary_pref = opt_prefs.value("primary_pref", ctx.mission.prefs.primary_pref);
  ctx.mission.prefs.secondary_pref = opt_prefs.value("secondary_pref", ctx.mission.prefs.secondary_pref);
  ctx.mission.prefs.weight_factors = opt_prefs.value("weight_factors", ctx.mission.prefs.weight_factors);

  const json holding_zone = mission0.value("holding_zone", json::object());
  const json entry_rules = holding_zone.value("entry_rules", json::object());
  ctx.mission.racecourse.orbit_radius =
      entry_rules.value("orbit_radius", ctx.mission.racecourse.orbit_radius);

  const json racetrack = mission0.value("racetrack", json::object());
  ctx.mission.racecourse.entry_angle_deg =
      racetrack.value("angle", ctx.mission.racecourse.entry_angle_deg);
  ctx.mission.racecourse.turn_direction =
      racetrack.value("turn_direction", ctx.mission.racecourse.turn_direction);
  ctx.mission.racecourse.altitude_m = racetrack.value("altitude", ctx.mission.racecourse.altitude_m);
  ctx.mission.racecourse.speed_mps = racetrack.value("speed", ctx.mission.racecourse.speed_mps);

  const json op_area = mission0.value("operation_area", json::object());
  ctx.mission.operation_area.boundary_lla = ParseLLAPolygon(op_area.value("boundary", json::array()));
  ctx.mission.operation_area.altitude_limits =
      op_area.value("altitude_limits", ctx.mission.operation_area.altitude_limits);
  ParseNoFlyZoneCollection(op_area.value("no_fly_zones", json::object()), "no_fly_zone_id",
                           ctx.mission.operation_area.no_fly_zones);

  // ---------- parse tankers ----------
  std::vector<fs::path> tanker_paths = CollectConfigPaths(dir, "tanker");
  if (tanker_paths.empty()) {
    const fs::path fallback = dir / "tanker_config.json";
    if (fs::exists(fallback)) tanker_paths.push_back(fallback);
  }
  ctx.tankers.clear();
  ctx.tankers.reserve(tanker_paths.size());
  for (const auto& p : tanker_paths) {
    const json root = ParseJson(ReadAllText(p), "tanker_config:" + p.filename().string());
    ctx.tankers.push_back(ParseAircraft(root, "tanker_id", "tanker_type"));
  }
  if (ctx.tankers.empty()) {
    throw std::runtime_error("No tanker config json found in: " + dir.string());
  }

  auto tanker_it = std::find_if(ctx.tankers.begin(), ctx.tankers.end(), [&](const AircraftConfig& tk) {
    return !ctx.mission.tanker_id.empty() && tk.id == ctx.mission.tanker_id;
  });
  ctx.tanker = (tanker_it != ctx.tankers.end()) ? *tanker_it : ctx.tankers.front();
  if (ctx.mission.tanker_id.empty()) ctx.mission.tanker_id = ctx.tanker.id;
  if (ctx.mission.tanker_ids.empty()) ctx.mission.tanker_ids.push_back(ctx.tanker.id);

  // ---------- parse receivers ----------
  std::vector<fs::path> receiver_paths = CollectConfigPaths(dir, "receiver");
  ctx.receivers.clear();
  ctx.receivers.reserve(receiver_paths.size());
  for (const auto& p : receiver_paths) {
    const json root = ParseJson(ReadAllText(p), "receiver_config:" + p.filename().string());
    ctx.receivers.push_back(ParseAircraft(root, "receiver_id", "receiver_type"));
  }

  if (!ctx.mission.receiver_ids.empty() && !ctx.receivers.empty()) {
    std::unordered_set<std::string> keep(ctx.mission.receiver_ids.begin(), ctx.mission.receiver_ids.end());
    std::vector<AircraftConfig> filtered;
    filtered.reserve(ctx.receivers.size());
    for (const auto& r : ctx.receivers) {
      if (keep.count(r.id)) filtered.push_back(r);
    }
    if (!filtered.empty()) ctx.receivers = std::move(filtered);
  }

  // ---------- parse assignment ----------
  const std::string assignment_text = ReadAllTextIfExists(dir / "assignment.json");
  ctx.mission.assignments.clear();
  if (!assignment_text.empty()) {
    const json assign_root = ParseJson(assignment_text, "assignment");
    const json assignments = assign_root.value("assignments", json::array());
    if (assignments.is_array()) {
      for (const auto& item : assignments) {
        if (!item.is_object()) continue;
        if (!item.contains("tanker") || !item.contains("assigned_events")) continue;
        RefuelAssignment a;
        a.tanker_id = item.value("tanker", "");
        if (item["assigned_events"].is_array()) {
          for (const auto& rid : item["assigned_events"]) {
            if (rid.is_string()) a.receiver_ids.push_back(rid.get<std::string>());
          }
        }
        if (!a.tanker_id.empty() && !a.receiver_ids.empty()) {
          ctx.mission.assignments.push_back(std::move(a));
        }
      }
    }
  }

  if (ctx.mission.assignments.empty() && !ctx.tanker.id.empty() && !ctx.mission.receiver_ids.empty()) {
    RefuelAssignment a;
    a.tanker_id = ctx.tanker.id;
    a.receiver_ids = ctx.mission.receiver_ids;
    ctx.mission.assignments.push_back(std::move(a));
  }

  ctx.mission.branch_kind = DeduceBranchKind(ctx);
  return ctx;
}

} // namespace refuel::io
