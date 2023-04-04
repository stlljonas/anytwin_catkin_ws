/*!
* @file    test_main.cpp
* @author  Dario Bellicoso
* @date    Nov 23, 2017
*/

#include <gtest/gtest.h>

/// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
