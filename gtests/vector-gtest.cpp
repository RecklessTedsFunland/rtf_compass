#include <gtest/gtest.h> // googletest header file
#include <ahrs.h>
#include <string>
#include <iostream>


using namespace std;


TEST(vector, init) {
    Vec3 v(1,2,3);
    EXPECT_TRUE(v[0] == 1.0);
}
