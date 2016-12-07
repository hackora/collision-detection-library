// gtest
#include <gtest/gtest.h> // googletest header file

// collision library
#include <collision_library.h>
using namespace collision;

// gmlib
#include <gmParametricsModule>
using namespace GMlib;

// stl
#include <memory>
#include <chrono>
#include <iostream>
using namespace std::chrono_literals;


TEST(MyUniqueTestCategory, MyCategory_UniqueTestName_WhichPasses) {

  EXPECT_TRUE(true);
}

TEST(MyUniqueTestCategory, MyCategory_UniqueTestName_WhichFails) {

  EXPECT_FALSE(true);
}
