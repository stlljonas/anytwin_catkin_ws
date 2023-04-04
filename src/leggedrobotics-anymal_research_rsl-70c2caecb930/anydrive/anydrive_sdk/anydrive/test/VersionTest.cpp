#include <gtest/gtest.h>

#include "anydrive/common/Version.hpp"

using anydrive::common::Version;

TEST(VersionTest, stringConversions) {  // NOLINT
  Version version1(1, 2, 3);
  EXPECT_EQ(version1.major_, 1);
  EXPECT_EQ(version1.minor_, 2);
  EXPECT_EQ(version1.patch_, 3);

  Version version2("1.2.3");
  EXPECT_EQ(version2.major_, 1);
  EXPECT_EQ(version2.minor_, 2);
  EXPECT_EQ(version2.patch_, 3);

  Version version3(-1, -2, -3);
  EXPECT_EQ(version3.major_, -1);
  EXPECT_EQ(version3.minor_, -2);
  EXPECT_EQ(version3.patch_, -3);

  Version version4("-1.-2.-3");
  EXPECT_EQ(version4.major_, -1);
  EXPECT_EQ(version4.minor_, -2);
  EXPECT_EQ(version4.patch_, -3);

  Version version5;
  version5.fromString("1.2.3");
  EXPECT_EQ(version5.major_, 1);
  EXPECT_EQ(version5.minor_, 2);
  EXPECT_EQ(version5.patch_, 3);
  Version version6(version5.toString());
  EXPECT_EQ(version6.major_, version5.major_);
  EXPECT_EQ(version6.minor_, version5.minor_);
  EXPECT_EQ(version6.patch_, version5.patch_);
}

TEST(VersionTest, comparisons) {  // NOLINT
  EXPECT_TRUE(Version("1.2.3") < Version("2.0.0"));
  EXPECT_TRUE(Version("1.2.3") < Version("1.3.0"));
  EXPECT_TRUE(Version("1.2.3") < Version("1.2.4"));
  EXPECT_FALSE(Version("1.2.3") < Version("1.2.3"));

  EXPECT_TRUE(Version("1.2.3") <= Version("2.0.0"));
  EXPECT_TRUE(Version("1.2.3") <= Version("1.3.0"));
  EXPECT_TRUE(Version("1.2.3") <= Version("1.2.4"));
  EXPECT_TRUE(Version("1.2.3") <= Version("1.2.3"));

  EXPECT_FALSE(Version("1.2.3") > Version("2.0.0"));
  EXPECT_FALSE(Version("1.2.3") > Version("1.3.0"));
  EXPECT_FALSE(Version("1.2.3") > Version("1.2.4"));

  EXPECT_FALSE(Version("1.2.3") >= Version("2.0.0"));
  EXPECT_FALSE(Version("1.2.3") >= Version("1.3.0"));
  EXPECT_FALSE(Version("1.2.3") >= Version("1.2.4"));
  EXPECT_TRUE(Version("1.2.3") >= Version("1.2.3"));

  EXPECT_TRUE(Version("1.2.3") == Version("1.2.3"));
  EXPECT_TRUE(Version("1.2.3") != Version("1.2.4"));
}
