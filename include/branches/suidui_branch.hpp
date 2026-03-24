#pragma once

#include <string>

#include "common/types.hpp"

namespace refuel::branch {

bool RunSuiduiBranch(const PlanningContext& base,
                     const std::string& outputDir,
                     std::string* errorMsg = nullptr);

} // namespace refuel::branch
