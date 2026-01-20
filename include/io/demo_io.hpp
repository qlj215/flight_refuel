#pragma once
#include <string>
#include "common/types.hpp"

namespace refuel::io {

// 这个 DemoIO 只负责“把 demo/input 里的文件读进 PlanningContext”。
// 由于你要求“只暴露接口、实现空着”，这里提供清晰的字段映射说明，
// 但默认不做真正解析（你把 TODO 填上就能工作）。
//
// demo/input 结构：
//   demo/input/mission4.json
//   demo/input/tankers01_config.json
//   demo/input/receivers01_config.json
//   demo/input/receivers02_config.json
//   demo/input/receivers03_config.json
class DemoIO {
public:
  // input_dir: 例如 "demo/input"
  // 返回：PlanningContext 里 mission/tanker/receivers 填好（或填默认值）
  static PlanningContext LoadContext(const std::string& input_dir);

private:
  // 你可以按自己工程需要拆更细：LoadMission / LoadTanker / LoadReceiver...
};

} // namespace refuel::io
