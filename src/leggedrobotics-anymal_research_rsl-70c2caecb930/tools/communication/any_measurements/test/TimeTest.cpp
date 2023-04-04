#include <gtest/gtest.h>

#include "any_measurements/Time.hpp"

TEST(TimeTest, FormatTimezone) {  // NOLINT
  std::string timeStr;
  any_measurements::Time time(111111, 0);

  timeStr = time.formatTimezone("%Y-%m-%d_%H-%M-%S %Z", "UTC");
  EXPECT_EQ(timeStr, "1970-01-02_06-51-51 UTC");

  timeStr = time.formatTimezone("%Y-%m-%d_%H-%M-%S %Z", "America/New_York");
  EXPECT_EQ(timeStr, "1970-01-02_01-51-51 EST");

  timeStr = time.formatTimezone("%Y-%m-%d_%H-%M-%S %Z", "Europe/Zurich");
  EXPECT_EQ(timeStr, "1970-01-02_07-51-51 CET");

  timeStr = time.formatTimezone("%Y-%m-%d_%H-%M-%S %Z", "Africa/Nairobi");
  EXPECT_EQ(timeStr, "1970-01-02_09-51-51 EAT");

  timeStr = time.formatTimezone("%Y-%m-%d_%H-%M-%S %Z", "Antarctica/McMurdo");
  EXPECT_EQ(timeStr, "1970-01-02_18-51-51 NZST");

  timeStr = time.formatTimezone("%Y-%m-%d_%H-%M-%S %Z", "Asia/Tokyo");
  EXPECT_EQ(timeStr, "1970-01-02_15-51-51 JST");
}
