#include "io/demo_io.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

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

PlanningContext DemoIO::LoadContext(const std::string& input_dir) {
  PlanningContext ctx;

  // =========================
  // 你需要在这里完成 JSON 解析
  // =========================
  //
  // 1) mission4.json（数组，demo 里只有 1 个 mission）
  //    - mission_id                -> ctx.mission.mission_id
  //    - units.tanker_id           -> ctx.mission.tanker_id
  //    - units.receiver_id         -> ctx.mission.receiver_ids（按空格 split）
  //    - refuel_mode.mode_type     -> ctx.mission.refuel_mode.mode_type
  //    - refuel_mode.timing_params.sub_mode / designated_time
  //    - ...safe_zone.is_defined / safe_zone.zones[i].vertices[[lat,lon],...]
  //    - optimization_prefs.primary_pref / secondary_pref / weight_factors
  //    - racecourse.orbit_radius / entry_angle / turn_direction
  //
  // 2) tankers01_config.json
  //    - tanker_id / tanker_type
  //    - cruise_performance_table.* -> ctx.tanker.cruise_perf.*
  //    - fuel_consumption_table.*   -> ctx.tanker.fuel_table.*
  //    - initial_position.{lat,lon,alt} -> ctx.tanker.initial_position_lla
  //    - current_status.{fuel_kg,heading_deg,speed_mps,timestamp} -> ctx.tanker.current_status
  //    - performance_limits.{climb_rate,descent_rate,max_altitude,max_bank_angle} -> ctx.tanker.limits
  //
  // 3) receiversXX_config.json（多个）
  //    - receiver_id / receiver_type
  //    - cruise_performance_table / fuel_consumption_table / initial_position / current_status / performance_limits
  //    - 填入 ctx.receivers
  //
  // 读取示例（先把字符串读出来，解析由你补全）：
  const fs::path dir(input_dir);
  const auto mission_text  = ReadAllText(dir / "mission4.json");
  const auto tanker_text   = ReadAllText(dir / "tankers01_config.json");
  const auto recv1_text    = ReadAllText(dir / "receivers01_config.json");
  const auto recv2_text    = ReadAllText(dir / "receivers02_config.json");
  const auto recv3_text    = ReadAllText(dir / "receivers03_config.json");

  (void)mission_text;
  (void)tanker_text;
  (void)recv1_text;
  (void)recv2_text;
  (void)recv3_text;

  // TODO: JSON 解析并填充 ctx
  // 当前先给一个“可运行的默认值”，便于你先跑通 Pipeline 框架。
  ctx.mission.mission_id = "RF-01";
  ctx.mission.tanker_id  = "TK001";
  ctx.mission.receiver_ids = {"RE001", "RE002", "RE003"};
  ctx.mission.refuel_mode.mode_type = 1;
  ctx.mission.prefs.primary_pref = 4;
  ctx.mission.prefs.weight_factors = {0.5, 0.5, 0.0};
  ctx.mission.racecourse.orbit_radius = 5000;
  ctx.mission.racecourse.entry_angle_deg = 30;

  ctx.tanker.id = "TK001";
  ctx.tanker.type = "KC-135";
  ctx.tanker.initial_position_lla = {29.278728, 119.913487, 5000};

  ctx.receivers.resize(3);
  ctx.receivers[0].id = "RE001";
  ctx.receivers[0].type = "F-16";
  ctx.receivers[0].initial_position_lla = {29.192483, 116.140339, 5000};

  ctx.receivers[1].id = "RE002";
  ctx.receivers[1].type = "F-16";
  ctx.receivers[1].initial_position_lla = {27.858468, 116.017797, 5000};

  ctx.receivers[2].id = "RE003";
  ctx.receivers[2].type = "F-16";
  ctx.receivers[2].initial_position_lla = {27.844521, 116.009685, 5000};

  // safe_zone：默认 is_defined=true 并给一个矩形（与你 demo/mission4.json 一致）
  ctx.mission.refuel_mode.safe_zone.is_defined = true;
  ctx.mission.refuel_mode.safe_zone.zones_vertices_lla = {
    {
      {29.0, 119.5, 0}, {29.0, 121.4, 0}, {29.8, 121.4, 0}, {29.8, 119.5, 0}
    }
  };

  return ctx;
}

} // namespace refuel::io
