#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

#include "hybrid_astar_model/hybrid_astar_model.hpp"

using namespace Eigen;

std::string colorMessage(const std::string& message, const std::string& color)
{
    std::string color_code;
    if (color == "r")
    {
        color_code = "1;31";
    }
    else if (color == "g")
    {
        color_code = "1;32";
    }
    else if (color == "b")
    {
        color_code = "1;34";
    }
    else
    {
        color_code = "0";  // Default color
    }
    return "\033[" + color_code + "m" + message + "\033[0m";
}

TEST(MyNode, testHybridAstar)
{
    vector<double> x_list = { 1.0, 2.0, 3.0 };
    vector<double> y_list = { 4.0, 5.0, 6.0 };
    vector<double> yaw_list = { 0.1, 0.2, 0.3 };
    vector<bool> directions = { true, false, true };

    // Initialize the PNode object
    PNode node(1, 2, 3, true, x_list, y_list, yaw_list, directions, 0, 10.0, 0.0);
    Path path(x_list, y_list, yaw_list, directions, 10.0);
    Config config(x_list, y_list, 0.1, 0.1);

    vector<Vector2d> motion_inputs = calc_motion_inputs(-MAX_STEER, MAX_STEER, hybrid_astar_model::N_STEER);
    EXPECT_EQ(motion_inputs.size(), (hybrid_astar_model::N_STEER + 1) * 2);
    Vector2d first_item = Vector2d(-MAX_STEER, 1);
    Vector2d second_item = Vector2d(-MAX_STEER, -1);
    Vector2d third_item = Vector2d(-0.5368, 1);
    Vector2d fourth_item = Vector2d(-0.5368, -1);

    EXPECT_EQ(motion_inputs[0], first_item);
    EXPECT_EQ(motion_inputs[1], second_item);
    EXPECT_NEAR(motion_inputs[2][0], third_item[0], 1e-4);
    EXPECT_NEAR(motion_inputs[2][1], third_item[1], 1e-4);
    EXPECT_NEAR(motion_inputs[3][0], fourth_item[0], 1e-4);
    EXPECT_NEAR(motion_inputs[3][1], fourth_item[1], 1e-4);

    vector<Vector2d> points;
    points.push_back(Vector2d(0.0, 0.0));
    points.push_back(Vector2d(1.0, 1.0));
    points.push_back(Vector2d(2.0, 2.0));
    points.push_back(Vector2d(3.0, 3.0));

    KdTree kd_tree(points);
    PNode next_node = calc_next_node(node, 0.1, 1, config, x_list, y_list, kd_tree);

    // Example usage of the colorMessage function
    std::string colored_message = colorMessage("PNode object initialized successfully!", "g");
    std::cout << colored_message << std::endl;
    EXPECT_EQ(1, 1);
}
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}