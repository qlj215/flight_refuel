#include "io/demo_io.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <type_traits>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace refuel::io {

static std::string ReadAllText(const fs::path& p) {
  std::ifstream ifs(p, std::ios::in | std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("Failed to open file: " + p.string());
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

static std::string ReadAllTextIfExists(const fs::path& p) {
  if (!fs::exists(p)) return {};
  return ReadAllText(p);
}

static std::vector<std::string> SplitWS(const std::string& s) {
  std::istringstream iss(s);
  std::vector<std::string> out;
  for (std::string tok; iss >> tok;) out.push_back(tok);
  return out;
}

static nlohmann::json ParseJson(const std::string& text, const std::string& hint) {
  try {
    return nlohmann::json::parse(text);
  } catch (const std::exception& e) {
    throw std::runtime_error("JSON parse failed for " + hint + ": " + std::string(e.what()));
  }
}

// --------- SFINAE helpers (compile-safe optional assignments) ---------
template <typename T, typename = void>
struct has_member_secondary_pref : std::false_type {};
template <typename T>
struct has_member_secondary_pref<T, std::void_t<decltype(std::declval<T&>().secondary_pref)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_turn_direction : std::false_type {};
template <typename T>
struct has_member_turn_direction<T, std::void_t<decltype(std::declval<T&>().turn_direction)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_current_status : std::false_type {};
template <typename T>
struct has_member_current_status<T, std::void_t<decltype(std::declval<T&>().current_status)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_limits : std::false_type {};
template <typename T>
struct has_member_limits<T, std::void_t<decltype(std::declval<T&>().limits)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_cruise_perf : std::false_type {};
template <typename T>
struct has_member_cruise_perf<T, std::void_t<decltype(std::declval<T&>().cruise_perf)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_fuel_table : std::false_type {};
template <typename T>
struct has_member_fuel_table<T, std::void_t<decltype(std::declval<T&>().fuel_table)>> : std::true_type {};

template <typename TableT, typename = void>
struct has_member_altitude_levels : std::false_type {};
template <typename TableT>
struct has_member_altitude_levels<TableT, std::void_t<decltype(std::declval<TableT&>().altitude_levels)>> : std::true_type {};

template <typename TableT, typename = void>
struct has_member_speed_levels : std::false_type {};
template <typename TableT>
struct has_member_speed_levels<TableT, std::void_t<decltype(std::declval<TableT&>().speed_levels)>> : std::true_type {};

template <typename TableT, typename = void>
struct has_member_true_airspeed_data : std::false_type {};
template <typename TableT>
struct has_member_true_airspeed_data<TableT, std::void_t<decltype(std::declval<TableT&>().true_airspeed_data)>> : std::true_type {};

template <typename TableT, typename = void>
struct has_member_consumption_data : std::false_type {};
template <typename TableT>
struct has_member_consumption_data<TableT, std::void_t<decltype(std::declval<TableT&>().consumption_data)>> : std::true_type {};

template <typename StatusT, typename = void>
struct has_member_fuel_kg : std::false_type {};
template <typename StatusT>
struct has_member_fuel_kg<StatusT, std::void_t<decltype(std::declval<StatusT&>().fuel_kg)>> : std::true_type {};

template <typename StatusT, typename = void>
struct has_member_heading_deg : std::false_type {};
template <typename StatusT>
struct has_member_heading_deg<StatusT, std::void_t<decltype(std::declval<StatusT&>().heading_deg)>> : std::true_type {};

template <typename StatusT, typename = void>
struct has_member_speed_mps : std::false_type {};
template <typename StatusT>
struct has_member_speed_mps<StatusT, std::void_t<decltype(std::declval<StatusT&>().speed_mps)>> : std::true_type {};

template <typename StatusT, typename = void>
struct has_member_timestamp : std::false_type {};
template <typename StatusT>
struct has_member_timestamp<StatusT, std::void_t<decltype(std::declval<StatusT&>().timestamp)>> : std::true_type {};

template <typename LimitsT, typename = void>
struct has_member_climb_rate : std::false_type {};
template <typename LimitsT>
struct has_member_climb_rate<LimitsT, std::void_t<decltype(std::declval<LimitsT&>().climb_rate)>> : std::true_type {};

template <typename LimitsT, typename = void>
struct has_member_descent_rate : std::false_type {};
template <typename LimitsT>
struct has_member_descent_rate<LimitsT, std::void_t<decltype(std::declval<LimitsT&>().descent_rate)>> : std::true_type {};

template <typename LimitsT, typename = void>
struct has_member_max_altitude : std::false_type {};
template <typename LimitsT>
struct has_member_max_altitude<LimitsT, std::void_t<decltype(std::declval<LimitsT&>().max_altitude)>> : std::true_type {};

template <typename LimitsT, typename = void>
struct has_member_max_bank_angle_deg : std::false_type {};
template <typename LimitsT>
struct has_member_max_bank_angle_deg<LimitsT, std::void_t<decltype(std::declval<LimitsT&>().max_bank_angle_deg)>> : std::true_type {};

PlanningContext DemoIO::LoadContext(const std::string& input_dir) {
  PlanningContext ctx;

  const fs::path dir(input_dir);

  // JSON 解析并填充 ctx（所有值来自 input_dir 下的 *.json）
  using json = nlohmann::json;

  // ---------- locate files ----------
  std::string mission_text = ReadAllTextIfExists(dir / "mission1.json");
  if (mission_text.empty()) mission_text = ReadAllTextIfExists(dir / "mission4.json");
  if (mission_text.empty()) {
    throw std::runtime_error("No mission json found in: " + dir.string());
  }

  std::string tanker_text = ReadAllTextIfExists(dir / "tankers01_config.json");
  if (tanker_text.empty()) tanker_text = ReadAllTextIfExists(dir / "tanker_config.json");
  if (tanker_text.empty()) {
    throw std::runtime_error("No tanker config json found in: " + dir.string());
  }

  std::vector<fs::path> receiver_paths;
  for (const auto* name : {"receivers01_config.json", "receivers02_config.json", "receivers03_config.json"}) {
    const fs::path p = dir / name;
    if (fs::exists(p)) receiver_paths.push_back(p);
  }
  if (receiver_paths.empty()) {
    // fallback: scan directory for receivers*_config.json
    for (const auto& ent : fs::directory_iterator(dir)) {
      if (!ent.is_regular_file()) continue;
      const auto fname = ent.path().filename().string();
      if (fname.find("receivers") == 0 && fname.find("_config.json") != std::string::npos) {
        receiver_paths.push_back(ent.path());
      }
    }
    std::sort(receiver_paths.begin(), receiver_paths.end());
  }

  // ---------- parse mission ----------
  const json mission_root = ParseJson(mission_text, "mission");
  const json mission0 = (mission_root.is_array() && !mission_root.empty()) ? mission_root.at(0) : mission_root;

  ctx.mission.mission_id = mission0.value("mission_id", ctx.mission.mission_id);

  const json units = mission0.value("units", json::object());
  const auto tanker_ids = SplitWS(units.value("tanker_id", ""));
  if (!tanker_ids.empty()) ctx.mission.tanker_id = tanker_ids.front();
  ctx.mission.receiver_ids = SplitWS(units.value("receiver_id", ""));

  // refuel_mode: 新版 types.hpp 里 sub_mode / designated_time 直接挂在 refuel_mode 下
  // 但为了兼容旧 JSON，仍支持 refuel_mode.timing_params.*
  const json refuel_mode = mission0.value("refuel_mode", json::object());
  ctx.mission.refuel_mode.mode_type =
      refuel_mode.value("mode_type", ctx.mission.refuel_mode.mode_type);

  // 选择 sub_mode / designated_time / safe_zone 的来源对象：
  // 1) 优先 refuel_mode.timing_params（旧结构）
  // 2) 否则 refuel_mode（新结构）
  const json* rm_src_ptr = &refuel_mode;
  if (refuel_mode.contains("timing_params") && refuel_mode["timing_params"].is_object()) {
    rm_src_ptr = &refuel_mode["timing_params"];
  }
  const json& rm_src = *rm_src_ptr;

  ctx.mission.refuel_mode.sub_mode =
      rm_src.value("sub_mode", ctx.mission.refuel_mode.sub_mode);
  ctx.mission.refuel_mode.designated_time =
      rm_src.value("designated_time", ctx.mission.refuel_mode.designated_time);

  // safe_zone：新版 RefuelModeInput.safe_zone 直接存在
  // 兼容两种结构：
  // - refuel_mode.timing_params.safe_zone (旧)
  // - refuel_mode.safe_zone (新)
  const json safe_zone = rm_src.value("safe_zone", json::object());
  ctx.mission.refuel_mode.safe_zone.is_defined = safe_zone.value("is_defined", false);
  ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.clear();

  if (ctx.mission.refuel_mode.safe_zone.is_defined) {
    // 兼容：safe_zone.zones[i].vertices
    if (safe_zone.contains("zones") && safe_zone["zones"].is_array()) {
      for (const auto& zone : safe_zone["zones"]) {
        if (!zone.is_object()) continue;
        if (!zone.contains("vertices") || !zone["vertices"].is_array()) continue;

        std::vector<LLA> poly;
        for (const auto& v : zone["vertices"]) {
          if (v.is_array() && v.size() >= 2) {
            poly.push_back(LLA{v.at(0).get<double>(), v.at(1).get<double>(), 0.0});
          }
        }
        if (!poly.empty()) ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.push_back(std::move(poly));
      }
    }
    // 兼容：safe_zone.vertices
    else if (safe_zone.contains("vertices") && safe_zone["vertices"].is_array()) {
      std::vector<LLA> poly;
      for (const auto& v : safe_zone["vertices"]) {
        if (v.is_array() && v.size() >= 2) {
          poly.push_back(LLA{v.at(0).get<double>(), v.at(1).get<double>(), 0.0});
        }
      }
      if (!poly.empty()) ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.push_back(std::move(poly));
    }
  }

  const json opt_prefs = mission0.value("optimization_prefs", json::object());
  ctx.mission.prefs.primary_pref = opt_prefs.value("primary_pref", ctx.mission.prefs.primary_pref);
  if constexpr (has_member_secondary_pref<decltype(ctx.mission.prefs)>::value) {
    ctx.mission.prefs.secondary_pref =
        opt_prefs.value("secondary_pref", ctx.mission.prefs.secondary_pref);
  }
  ctx.mission.prefs.weight_factors =
      opt_prefs.value("weight_factors", ctx.mission.prefs.weight_factors);

  // racecourse：orbit_radius 从 holding_zone.entry_rules.orbit_radius 取，entry_angle / turn_direction 从 racetrack 取
  const json holding_zone = mission0.value("holding_zone", json::object());
  const json entry_rules = holding_zone.value("entry_rules", json::object());
  ctx.mission.racecourse.orbit_radius =
      entry_rules.value("orbit_radius", ctx.mission.racecourse.orbit_radius);

  const json racetrack = mission0.value("racetrack", json::object());
  ctx.mission.racecourse.entry_angle_deg =
      racetrack.value("angle", ctx.mission.racecourse.entry_angle_deg);
  if constexpr (has_member_turn_direction<decltype(ctx.mission.racecourse)>::value) {
    ctx.mission.racecourse.turn_direction =
        racetrack.value("turn_direction", ctx.mission.racecourse.turn_direction);
  }

  // ---------- parse tanker ----------
  const json tanker_root = ParseJson(tanker_text, "tanker_config");
  ctx.tanker.id = tanker_root.value("tanker_id", ctx.tanker.id);
  ctx.tanker.type = tanker_root.value("tanker_type", ctx.tanker.type);

  const json t_init = tanker_root.value("initial_position", json::object());
  ctx.tanker.initial_position_lla = LLA{
      t_init.value("latitude", 0.0),
      t_init.value("longitude", 0.0),
      t_init.value("altitude", 0.0),
  };

  if constexpr (has_member_cruise_perf<decltype(ctx.tanker)>::value) {
    const json cpt = tanker_root.value("cruise_performance_table", json::object());
    auto& cp = ctx.tanker.cruise_perf;
    if constexpr (has_member_altitude_levels<decltype(cp)>::value)
      cp.altitude_levels = cpt.value("altitude_levels", cp.altitude_levels);
    if constexpr (has_member_speed_levels<decltype(cp)>::value)
      cp.speed_levels = cpt.value("speed_levels", cp.speed_levels);
    if constexpr (has_member_true_airspeed_data<decltype(cp)>::value)
      cp.true_airspeed_data = cpt.value("true_airspeed_data", cp.true_airspeed_data);
  }

  if constexpr (has_member_fuel_table<decltype(ctx.tanker)>::value) {
    const json ft = tanker_root.value("fuel_consumption_table", json::object());
    auto& ftab = ctx.tanker.fuel_table;
    if constexpr (has_member_altitude_levels<decltype(ftab)>::value)
      ftab.altitude_levels = ft.value("altitude_levels", ftab.altitude_levels);
    if constexpr (has_member_speed_levels<decltype(ftab)>::value)
      ftab.speed_levels = ft.value("speed_levels", ftab.speed_levels);
    if constexpr (has_member_consumption_data<decltype(ftab)>::value)
      ftab.consumption_data = ft.value("consumption_data", ftab.consumption_data);
  }

  if constexpr (has_member_current_status<decltype(ctx.tanker)>::value) {
    const json cs = tanker_root.value("current_status", json::object());
    auto& st = ctx.tanker.current_status;
    if constexpr (has_member_fuel_kg<decltype(st)>::value)
      st.fuel_kg = cs.value("fuel_kg", st.fuel_kg);
    if constexpr (has_member_heading_deg<decltype(st)>::value)
      st.heading_deg = cs.value("heading_deg", st.heading_deg);
    if constexpr (has_member_speed_mps<decltype(st)>::value)
      st.speed_mps = cs.value("speed_mps", st.speed_mps);
    if constexpr (has_member_timestamp<decltype(st)>::value)
      st.timestamp = cs.value("timestamp", st.timestamp);
  }

  if constexpr (has_member_limits<decltype(ctx.tanker)>::value) {
    const json lim = tanker_root.value("performance_limits", json::object());
    auto& l = ctx.tanker.limits;

    if constexpr (has_member_climb_rate<decltype(l)>::value)
      l.climb_rate = lim.value("climb_rate", l.climb_rate);
    if constexpr (has_member_descent_rate<decltype(l)>::value)
      l.descent_rate = lim.value("descent_rate", l.descent_rate);
    if constexpr (has_member_max_altitude<decltype(l)>::value)
      l.max_altitude = lim.value("max_altitude", l.max_altitude);

    // 新字段：max_bank_angle_deg（兼容旧 key：max_bank_angle）
    if constexpr (has_member_max_bank_angle_deg<decltype(l)>::value) {
      l.max_bank_angle_deg = lim.value("max_bank_angle_deg", l.max_bank_angle_deg);
      if (lim.contains("max_bank_angle") && lim["max_bank_angle"].is_number()) {
        l.max_bank_angle_deg = lim["max_bank_angle"].get<double>();
      }
    }

    // 注意：新版 PerformanceLimits 没有 fuel_transfer_speed，这里不再解析
  }

  // ---------- parse receivers ----------
  ctx.receivers.clear();
  ctx.receivers.reserve(receiver_paths.size());

  for (const auto& rp : receiver_paths) {
    const json r = ParseJson(ReadAllText(rp), "receiver_config:" + rp.filename().string());

    decltype(ctx.receivers)::value_type recv{};
    recv.id = r.value("receiver_id", recv.id);
    recv.type = r.value("receiver_type", recv.type);

    const json r_init = r.value("initial_position", json::object());
    recv.initial_position_lla = LLA{
        r_init.value("latitude", 0.0),
        r_init.value("longitude", 0.0),
        r_init.value("altitude", 0.0),
    };

    if constexpr (has_member_cruise_perf<decltype(recv)>::value) {
      const json cpt = r.value("cruise_performance_table", json::object());
      auto& cp = recv.cruise_perf;
      if constexpr (has_member_altitude_levels<decltype(cp)>::value)
        cp.altitude_levels = cpt.value("altitude_levels", cp.altitude_levels);
      if constexpr (has_member_speed_levels<decltype(cp)>::value)
        cp.speed_levels = cpt.value("speed_levels", cp.speed_levels);
      if constexpr (has_member_true_airspeed_data<decltype(cp)>::value)
        cp.true_airspeed_data = cpt.value("true_airspeed_data", cp.true_airspeed_data);
    }

    if constexpr (has_member_fuel_table<decltype(recv)>::value) {
      const json ft = r.value("fuel_consumption_table", json::object());
      auto& ftab = recv.fuel_table;
      if constexpr (has_member_altitude_levels<decltype(ftab)>::value)
        ftab.altitude_levels = ft.value("altitude_levels", ftab.altitude_levels);
      if constexpr (has_member_speed_levels<decltype(ftab)>::value)
        ftab.speed_levels = ft.value("speed_levels", ftab.speed_levels);
      if constexpr (has_member_consumption_data<decltype(ftab)>::value)
        ftab.consumption_data = ft.value("consumption_data", ftab.consumption_data);
    }

    if constexpr (has_member_current_status<decltype(recv)>::value) {
      const json cs = r.value("current_status", json::object());
      auto& st = recv.current_status;
      if constexpr (has_member_fuel_kg<decltype(st)>::value)
        st.fuel_kg = cs.value("fuel_kg", st.fuel_kg);
      if constexpr (has_member_heading_deg<decltype(st)>::value)
        st.heading_deg = cs.value("heading_deg", st.heading_deg);
      if constexpr (has_member_speed_mps<decltype(st)>::value)
        st.speed_mps = cs.value("speed_mps", st.speed_mps);
      if constexpr (has_member_timestamp<decltype(st)>::value)
        st.timestamp = cs.value("timestamp", st.timestamp);
    }

    if constexpr (has_member_limits<decltype(recv)>::value) {
      const json lim = r.value("performance_limits", json::object());
      auto& l = recv.limits;

      if constexpr (has_member_climb_rate<decltype(l)>::value)
        l.climb_rate = lim.value("climb_rate", l.climb_rate);
      if constexpr (has_member_descent_rate<decltype(l)>::value)
        l.descent_rate = lim.value("descent_rate", l.descent_rate);
      if constexpr (has_member_max_altitude<decltype(l)>::value)
        l.max_altitude = lim.value("max_altitude", l.max_altitude);

      // 新字段：max_bank_angle_deg（兼容旧 key：max_bank_angle）
      if constexpr (has_member_max_bank_angle_deg<decltype(l)>::value) {
        l.max_bank_angle_deg = lim.value("max_bank_angle_deg", l.max_bank_angle_deg);
        if (lim.contains("max_bank_angle") && lim["max_bank_angle"].is_number()) {
          l.max_bank_angle_deg = lim["max_bank_angle"].get<double>();
        }
      }

      // 注意：新版 PerformanceLimits 没有 fuel_transfer_speed，这里不再解析
    }

    ctx.receivers.push_back(std::move(recv));
  }

  return ctx;
}

} // namespace refuel::io
