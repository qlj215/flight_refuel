#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace refuel::test {

struct TestCase {
  std::string name;
  std::function<bool()> fn;
};

inline int RunAll(const std::vector<TestCase>& cases) {
  int failed = 0;
  for (const auto& tc : cases) {
    bool ok = false;
    try {
      ok = tc.fn();
    } catch (const std::exception& e) {
      std::cerr << "[EXCEPTION] " << tc.name << ": " << e.what() << "\n";
      ok = false;
    } catch (...) {
      std::cerr << "[EXCEPTION] " << tc.name << ": unknown exception\n";
      ok = false;
    }

    if (ok) {
      std::cout << "[PASS] " << tc.name << "\n";
    } else {
      std::cout << "[FAIL] " << tc.name << "\n";
      failed++;
    }
  }

  if (failed == 0) {
    std::cout << "All tests passed (" << cases.size() << ").\n";
    return 0;
  }
  std::cout << failed << " test(s) failed out of " << cases.size() << ".\n";
  return 1;
}

// ----------- Minimal expectation macros -----------

#define REFUEL_EXPECT_TRUE(expr) \
  do { \
    if (!(expr)) { \
      std::cerr << "Expectation failed: " << #expr \
                << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
      return false; \
    } \
  } while (0)

#define REFUEL_EXPECT_EQ(a, b) \
  do { \
    auto _a = (a); \
    auto _b = (b); \
    if (!(_a == _b)) { \
      std::cerr << "Expectation failed: " << #a << " == " << #b \
                << " (got " << _a << " vs " << _b << ")" \
                << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
      return false; \
    } \
  } while (0)

#define REFUEL_EXPECT_NEAR(a, b, eps) \
  do { \
    auto _a = (a); \
    auto _b = (b); \
    auto _eps = (eps); \
    if ((_a > _b ? _a - _b : _b - _a) > _eps) { \
      std::cerr << "Expectation failed: |" << #a << " - " << #b << "| <= " << #eps \
                << " (got " << _a << " vs " << _b << ")" \
                << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
      return false; \
    } \
  } while (0)

} // namespace refuel::test
