#include <gtest/gtest.h>
#include "../src/path_manager.hpp" 

// TEST 1: Valid Path (Happy Path)
TEST(PathManagerTest, TCTRAJ01_ValidSequence) {
    PathManager manager;
    std::vector<Point> valid_path = { {0.5, 0.0, 0.5}, {0.6, 0.1, 0.5} };

    bool accepted = manager.setPath(valid_path);
    bool executed = manager.executePath();

    EXPECT_TRUE(accepted) << "The system should accept valid paths.";
    EXPECT_TRUE(executed) << "The system should execute valid paths.";
}

// TEST 2: Single Point Path (Boundary)
TEST(PathManagerTest, TCTRAJ02_SinglePointSequence) {
    PathManager manager;
    std::vector<Point> single_point = { {0.5, 0.0, 0.5} };
    bool accepted = manager.setPath(single_point);
    EXPECT_TRUE(accepted) << "The system should accept a single point.";
}

// TEST 3: Empty Path (Defect/Safety)
TEST(PathManagerTest, TCTRAJ04_EmptySequence) {
    PathManager manager;
    std::vector<Point> empty_path; 
    bool accepted = manager.setPath(empty_path);
    EXPECT_FALSE(accepted) << "The system MUST reject empty lists.";
}

// TEST 4: Unreachable Target (Defect)
TEST(PathManagerTest, TCTRAJ05_UnreachableTarget) {
    PathManager manager;
    std::vector<Point> far_path = { {5.0, 0.0, 0.0} };
    bool accepted = manager.setPath(far_path);
    EXPECT_FALSE(accepted) << "The system MUST reject distant points.";
}
