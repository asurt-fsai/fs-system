#include <gtest/gtest.h>
#include "../include/cone.h"

using namespace std;

TEST(TestInstantiation, TestBlueCone){
    Cone blue_cone(0, 0, 0);
    EXPECT_FLOAT_EQ(blue_cone.color_probs[0], 0.8);
    EXPECT_FLOAT_EQ(blue_cone.color_probs[1], 0.1);
    EXPECT_FLOAT_EQ(blue_cone.color_probs[2], 0.1);
}

TEST(TestInstantiation, TestYellowCone){
    Cone yellow_cone(0, 0, 1);
    EXPECT_FLOAT_EQ(yellow_cone.color_probs[0], 0.1);
    EXPECT_FLOAT_EQ(yellow_cone.color_probs[1], 0.8);
    EXPECT_FLOAT_EQ(yellow_cone.color_probs[2], 0.1);
}

TEST(TestInstantiation, TestOrangeCone){
    Cone orange_cone(0, 0, 2);
    EXPECT_FLOAT_EQ(orange_cone.color_probs[0], 0.1);
    EXPECT_FLOAT_EQ(orange_cone.color_probs[1], 0.1);
    EXPECT_FLOAT_EQ(orange_cone.color_probs[2], 0.8);
}

TEST(TestInstantiation, TestOtherCone){
    EXPECT_THROW(
        Cone other_cone(0, 0, 3);
        , std::invalid_argument);
}

TEST(TestDistance, TestDistance){
    Cone cone(0, 0, 0);
    EXPECT_FLOAT_EQ(cone.getDistance(3, 4), 5);
}

TEST(TestColorCost, TestBlue){
    Cone cone(0, 0, 0);
    EXPECT_FLOAT_EQ(cone.getCost(true), -log(0.1));
    EXPECT_FLOAT_EQ(cone.getCost(false), -log(0.8));
}

TEST(TestColorCost, TestYellow){
    Cone cone(0, 0, 1);
    EXPECT_FLOAT_EQ(cone.getCost(true), -log(0.8));
    EXPECT_FLOAT_EQ(cone.getCost(false), -log(0.1));
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
