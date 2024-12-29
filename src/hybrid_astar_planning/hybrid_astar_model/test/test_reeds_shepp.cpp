#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

#include "hybrid_astar_model/reeds_shepp.hpp"

using namespace Eigen;

void checkCarPose(const CarPose& res, const CarPose& expected)
{
    EXPECT_NEAR(res.x, expected.x, 1e-6);
    EXPECT_NEAR(res.y, expected.y, 1e-6);
    EXPECT_NEAR(res.yaw, expected.yaw, 1e-6);
    EXPECT_EQ(res.direction, expected.direction);
}

TEST(MyNode, testReedShepp)
{
    VectorXd lengths(3);
    lengths << 3.0, -5.0, -7.0;
    double step_size = 0.25;
    vector<VectorXd> inter_res_list = reeds_shepp::calc_interpolate_dists_list(lengths, step_size);
    VectorXd v1 = VectorXd::LinSpaced(lengths[0] / step_size + 1, 0, lengths[0]);
    VectorXd v2 = -VectorXd::LinSpaced(-lengths[1] / step_size + 1, 0, -lengths[1]);
    VectorXd v3 = -VectorXd::LinSpaced(-lengths[2] / step_size + 1, 0, -lengths[2]);
    vector<VectorXd> inter_exp_list{ v1, v2, v3 };
    EXPECT_EQ(inter_res_list, inter_exp_list);

    double dist, length, max_curvature;

    dist = -0.045609120556112825;
    length = -1.5707963267948966;
    max_curvature = 0.2280456027805641;
    char mode = 'R';
    CarPose origin(3.1007253486354567, 1.2843624925988495, 0.7853981633974483, 0);
    CarPose inter_res = reeds_shepp::interpolate(dist, length, mode, max_curvature, origin);
    CarPose inter_expected(2.9625775107126433, 1.1397656690337805, 0.8310072839535612, -1);
    checkCarPose(inter_res, inter_expected);

    dist = 0.0;
    length = 11.572728882475056;
    max_curvature = 0.2280456027805641;
    mode = 'S';
    origin = CarPose(2.698570461183419, -0.928686122485564, -0.6628896974524796, 0);
    inter_res = reeds_shepp::interpolate(dist, length, mode, max_curvature, origin);
    inter_expected = CarPose(2.698570461183419, -0.928686122485564, -0.6628896974524796, 1);
    checkCarPose(inter_res, inter_expected);

    dist = 0.0;
    length = -1.5707963267948966;
    max_curvature = 0.2280456027805641;
    mode = 'L';
    origin = CarPose(2.9380274767378136, -7.640389658466402, -2.40737814529585, 0);
    inter_res = reeds_shepp::interpolate(dist, length, mode, max_curvature, origin);
    inter_expected = CarPose(2.9380274767378136, -7.640389658466402, -2.40737814529585, -1);
    checkCarPose(inter_res, inter_expected);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}