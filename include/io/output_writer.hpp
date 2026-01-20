#pragma once
#include <string>
#include "common/types.hpp"

namespace refuel::io {

// OutputWriter 负责把 ctx 中的“最终产物”写回 demo/output：
// 1) output.json：包含 racetrack/tanker/receivers/receiver_summary（见 demo/output/output.json）
// 2) *.csv：每架飞机的轨迹点 x,y,z（见 demo/output/tanker01.csv, receiver01.csv...）
class OutputWriter {
public:
  static void WriteAll(const PlanningContext& ctx, const std::string& output_dir);

  // 分开暴露接口，方便你只写某一种输出进行调试
  static void WriteOutputJson(const PlanningContext& ctx, const std::string& output_path);
  static void WriteTrajectoriesCsv(const PlanningContext& ctx, const std::string& output_dir);
};

} // namespace refuel::io
