/** @file amr_tests.cpp
 * @brief Executes all tests of the AMR project.
 */

#include <gtest/gtest.h>

#include "unit_tests.hpp"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
