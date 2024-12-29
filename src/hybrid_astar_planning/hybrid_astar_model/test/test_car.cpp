#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

#include "hybrid_astar_model/car.hpp"

using namespace Eigen;

TEST(MyNode, testcar)
{
    Matrix2d result = rot_mat_2d(PI / 4);
    Matrix2d expected;
    expected << sqrt(2) / 2, -sqrt(2) / 2, sqrt(2) / 2, sqrt(2) / 2;
    EXPECT_NEAR((result - expected).norm(), 0, 1e-6);

    result = rot_mat_2d(PI / 2);
    expected << 0, -1, 1, 0;
    EXPECT_NEAR((result - expected).norm(), 0, 1e-6);

    double x = 16.959577971656262;
    double y = 20.423099152541;
    double yaw = 0.907906629342417;
    vector<double> ox = { 20.0, 20.0 };
    vector<double> oy = { 21.0, 22.0 };

    bool res = car::rectangle_check(x, y, yaw, ox, oy);
    EXPECT_TRUE(res);

    x = 17.39035596088274;
    y = 20.974851201923684;
    yaw = 0.907906629342417;
    ox = { 20.0, 20.0, 20.0 };
    oy = { 23.0, 21.0, 22.0 };

    res = car::rectangle_check(x, y, yaw, ox, oy);
    EXPECT_FALSE(res);

    vector<Vector2d> points;
    points.push_back(Vector2d(0.0, 0.0));
    points.push_back(Vector2d(1.0, 1.0));
    points.push_back(Vector2d(2.0, 2.0));
    points.push_back(Vector2d(3.0, 3.0));

    KdTree kdtree(points);
    Vector2d cxy = { 1, 1 };
    double radius = 3.0;
    vector<int> idx1 = kdtree.querry_ball_point(cxy, radius);
    vector<int> kd1_tree_expected = { 0, 1, 2, 3 };
    EXPECT_EQ(idx1, kd1_tree_expected);

    radius = 0.0;
    vector<int> idx2 = kdtree.querry_ball_point(cxy, radius);
    vector<int> kd2_tree_expected = { 1 };
    EXPECT_EQ(idx2, kd2_tree_expected);

    cxy = { 10, 10 };
    radius = 2;
    vector<int> idx3 = kdtree.querry_ball_point(cxy, radius);
    vector<int> kd3_tree_expected = {};
    EXPECT_EQ(idx3, kd3_tree_expected);

    CarPose curr_pose;
    curr_pose.x = 10.01367562190605;
    curr_pose.y = 10.399636076839563;
    curr_pose.yaw = 1.4795780856826717;
    double distance = 0.1;
    double steer = -0.6;
    CarPose new_pose = car::update_pose(curr_pose, distance, steer, WB);
    VectorXd new_pose_vec(3);
    new_pose_vec << new_pose.x, new_pose.y, new_pose.yaw;

    VectorXd expected_pose_vec(3);
    expected_pose_vec << 10.022784801182869, 10.499220326863865, 1.4567735254046155;
    EXPECT_NEAR((expected_pose_vec - new_pose_vec).norm(), 0, 1e-6);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}