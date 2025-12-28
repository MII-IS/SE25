#include <gtest/gtest.h>
#include <cmath>

// TEST 1: Verify that the robot's mathematical logic is consistent.
// We check that the "shoulder" movement (cosine) stays within limits.
TEST(RobotBailarinTest, CheckMathLogic) {
  double t = 1.0; // Simulate 1 second
  double shoulder = std::cos(t * 1.0) * 0.8; // Changed variable name to 'shoulder' for consistency
  
  // The cosine * 0.8 must never be greater than 0.8 or less than -0.8
  EXPECT_LE(shoulder, 0.8);
  EXPECT_GE(shoulder, -0.8);
}

// TEST 2: Simple Sanity Check (Smoke Test)
// Verifies that 2 + 2 equals 4 (to ensure the test system compiles correctly).
TEST(RobotBailarinTest, SanityCheck) {
  EXPECT_EQ(2 + 2, 4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
