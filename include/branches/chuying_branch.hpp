#pragma once

#include <string>

#include "common/types.hpp"

namespace refuel::branch {

// 运行“出迎(chuying)”分支。
//
// 输入：
//   base      - 已由 DemoIO 解析完成的任务上下文（坐标转换尚未执行）
//   outputDir - 输出目录
//
// 输出：
//   - outputDir/output.json
//   - 轨迹 CSV（单受油机时：tanker01.csv / receiver01.csv；多受油机时按 receiver_id 命名）
//
// 返回：
//   true  - 成功
//   false - 失败（可通过 errorMsg 查看失败原因）
bool RunChuyingBranch(const PlanningContext& base,
                      const std::string& outputDir,
                      std::string* errorMsg = nullptr);

} // namespace refuel::branch
