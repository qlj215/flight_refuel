#include "io/output_writer.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace fs = std::filesystem;

namespace refuel::io {

static void EnsureDir(const fs::path& p) {
  if (!fs::exists(p)) {
    fs::create_directories(p);
  }
}

void OutputWriter::WriteAll(const PlanningContext& ctx, const std::string& output_dir) {
  EnsureDir(output_dir);
  WriteOutputJson(ctx, (fs::path(output_dir) / "output.json").string());
  WriteTrajectoriesCsv(ctx, output_dir);
}

static void WriteJsonArray2(std::ostream& os, const Vec3& v_xy) {
  // output.json 的 entry_point/coordinate/waiting_point 是 [x, y] 两维数组
  os << "[" << std::fixed << std::setprecision(2) << v_xy.x << ", " << v_xy.y << "]";
}

void OutputWriter::WriteOutputJson(const PlanningContext& ctx, const std::string& output_path) {
  std::ofstream ofs(output_path);
  if (!ofs) throw std::runtime_error("Failed to write: " + output_path);

  ofs << "{\n";

  // racetrack
  ofs << "  \"racetrack\": {\n";
  ofs << "    \"length\": " << ctx.racetrack.length_m << ",\n";
  ofs << "    \"radius\": " << ctx.racetrack.radius_m << ",\n";
  ofs << "    \"orientation_deg\": " << ctx.racetrack.orientation_deg << ",\n";
  ofs << "    \"altitude_m\": " << ctx.racetrack.altitude_m << ",\n";
  ofs << "    \"speed\": " << ctx.racetrack.speed_mps << "\n";
  ofs << "  },\n";

  // tanker
  ofs << "  \"tanker\": {\n";
  ofs << "    \"meeting_altitude_m\": " << ctx.rendezvous.tanker_meeting_altitude_m << ",\n";
  ofs << "    \"speed\": " << ctx.rendezvous.tanker_meeting_speed_mps << ",\n";
  ofs << "    \"entry_point\": ";
  WriteJsonArray2(ofs, ctx.rendezvous.tanker_entry_point_xy);
  ofs << ",\n";
  ofs << "    \"entry_direction_deg\": " << ctx.rendezvous.tanker_entry_direction_deg << ",\n";
  ofs << "    \"arrival_time_s\": " << ctx.cost.tanker_arrival_time_s << ",\n";
  ofs << "    \"fuel_consumed\": " << ctx.cost.tanker_fuel_consumed << "\n";
  ofs << "  },\n";

  // receivers
  ofs << "  \"receivers\": {\n";
  bool first = true;
  for (const auto& recv_cfg : ctx.receivers) {
    const auto& rid = recv_cfg.id;
    auto it = ctx.rendezvous.receivers.find(rid);
    if (it == ctx.rendezvous.receivers.end()) continue;

    if (!first) ofs << ",\n";
    first = false;

    const auto& r = it->second;
    ofs << "    \"" << rid << "\": {\n";
    ofs << "      \"sequence\": " << r.sequence_order << ",\n";
    ofs << "      \"meeting_altitude_m\": " << r.meeting_altitude_m << ",\n";
    ofs << "      \"speed\": " << r.meeting_speed_mps << ",\n";
    ofs << "      \"coordinate\": ";
    WriteJsonArray2(ofs, r.coordinate_xy);
    ofs << ",\n";
    ofs << "      \"waiting_point\": ";
    WriteJsonArray2(ofs, r.waiting_point_xy);
    ofs << ",\n";
    ofs << "      \"cycling_number\": " << r.cycling_number << ",\n";

    // arrival_time_s / fuel_consumed
    const double at = (ctx.cost.receiver_arrival_time_s.count(rid) ? ctx.cost.receiver_arrival_time_s.at(rid) : 0.0);
    const double fc = (ctx.cost.receiver_fuel_consumed.count(rid) ? ctx.cost.receiver_fuel_consumed.at(rid) : 0.0);
    ofs << "      \"arrival_time_s\": " << at << ",\n";
    ofs << "      \"fuel_consumed\": " << fc << "\n";
    ofs << "    }";
  }
  ofs << "\n  },\n";

  // receiver_summary
  ofs << "  \"receiver_summary\": {\n";
  ofs << "    \"total_arrival_time_s\": " << ctx.cost.receiver_total_arrival_time_s << ",\n";
  ofs << "    \"total_fuel_consumed\": " << ctx.cost.receiver_total_fuel_consumed << "\n";
  ofs << "  }\n";

  ofs << "}\n";
}

static void WriteCsv(const fs::path& p, const std::vector<WaypointXYZ>& pts) {
  std::ofstream ofs(p);
  if (!ofs) throw std::runtime_error("Failed to write: " + p.string());
  ofs << "x,y,z\n";
  ofs << std::fixed << std::setprecision(2);
  for (const auto& w : pts) {
    ofs << w.x << "," << w.y << "," << w.z << "\n";
  }
}

void OutputWriter::WriteTrajectoriesCsv(const PlanningContext& ctx, const std::string& output_dir) {
  const fs::path outdir(output_dir);
  EnsureDir(outdir);

  // demo/output 的命名规律是：
  //   tanker01.csv
  //   receiver01.csv / receiver02.csv / receiver03.csv ...
  //
  // 这里仅做“接口示例”：按 ctx.trajectories 写；如果为空，就输出空文件头。
  auto find_traj = [&](const std::string& id) -> const Trajectory* {
    for (const auto& t : ctx.trajectories) if (t.aircraft_id == id) return &t;
    return nullptr;
  };

  // tanker
  {
    const Trajectory* t = find_traj(ctx.tanker.id);
    const auto* pts = t ? &t->points : nullptr;
    WriteCsv(outdir / "tanker01.csv", pts ? *pts : std::vector<WaypointXYZ>{});
  }

  // receivers: 按 ctx.receivers 的顺序映射 receiver01/02/03
  for (size_t i = 0; i < ctx.receivers.size(); ++i) {
    const auto& rid = ctx.receivers[i].id;
    const Trajectory* tr = find_traj(rid);

    char name[64];
    std::snprintf(name, sizeof(name), "receiver%02zu.csv", i + 1);

    const auto* pts = tr ? &tr->points : nullptr;
    WriteCsv(outdir / name, pts ? *pts : std::vector<WaypointXYZ>{});
  }
}

} // namespace refuel::io
