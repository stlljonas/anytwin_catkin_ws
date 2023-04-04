#include <gtest/gtest.h>

#include "point_cloud_processor/PointCloudProcessorCore.hpp"

using namespace point_cloud_processor;

TEST(PointCloudProcessor, exampleTest)
{  
  EXPECT_TRUE(true);
}

int main(int argc, char **argv)
{ 
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
