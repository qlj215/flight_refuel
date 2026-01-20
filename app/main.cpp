#include <iostream>
#include <string>

#include "io/demo_io.hpp"
#include "io/output_writer.hpp"
#include "pipeline/pipeline.hpp"

int main(int argc, char** argv) {
  // 默认使用 demo/input 和 demo/output
  // 你可以运行：
  //   ./refuel_demo
  // 或指定路径：
  //   ./refuel_demo /path/to/demo/input /path/to/demo/output
  std::string input_dir  = "demo/input";
  std::string output_dir = "demo/output";

  if (argc >= 2) input_dir = argv[1];
  if (argc >= 3) output_dir = argv[2];

  try {
    // 1) 读取输入（接口骨架：需要你补齐 JSON 解析）
    refuel::PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir);

    // 2) 跑完整流程（按流程图顺序调用各 Stage）
    refuel::Pipeline pipe;
    pipe.Run(ctx);

    // 3) 输出（output.json + csv）
    refuel::io::OutputWriter::WriteAll(ctx, output_dir);

    std::cout << "Done. Output written to: " << output_dir << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
