#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include <random>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <filesystem>
#include <vector>
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "visualization_msgs/msg/marker.hpp"

struct LidarData
{
    std::vector<double> theta;
    std::vector<double> dis;
};

struct Point
{
    int x;
    int y;

    Point(int xhat, int yhat)
    {
        x = xhat;
        y = yhat;
    }
};

struct GridMap
{
    int min_x, min_y, max_x, max_y, xw, yw, center_x, center_y;
    float resolution;


    Eigen::MatrixXd prob_map;

    GridMap(int min_x, int min_y, int max_x, int max_y, float resolution)
        : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y), resolution(resolution)
    {
        xw = int(std::round(max_x - min_x) / resolution);
        yw = int(std::round(max_y - min_y) / resolution);

        // set all cells in grid map to probablity of 0.5 -> unknow
        prob_map = Eigen::MatrixXd::Ones(xw, yw) / 2.0;

        center_x = int(std::round(xw / 2));
        center_y = int(std::round(yw / 2));

        std::cout << "Grid map size is: " << xw << " x " << yw << std::endl; 
        // std::cout << prob_map << std::endl;
    }
};

class Lidar2Grid : public rclcpp::Node
{
private:
    float EXTEND_AREA = 1.0;
    float RESOLUTION = 0.02;
    double pi = atan(1) * 4;
    LidarData lidar_data;


    // initilaize laser scan publisher
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;

    // iinitialize occupancy grid map publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;

public:

    Lidar2Grid(std::string &file_path, float extend_area, float resolution) : Node("lidar2grid_pub")
    {
        lidar_data = read_file(file_path);
        EXTEND_AREA = extend_area;
        RESOLUTION = resolution;
        laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
        grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
    }

    LidarData read_file(std::string json_path)
    {

        std::ifstream jsonFile(json_path);

        if (!jsonFile.is_open())
        {
            std::cout <<"Failed to open the JSON file." << std::endl;
        }

        Json::Value jsonData;
        jsonFile >> jsonData;
        jsonFile.close();

        for (const auto& item : jsonData)
        {
            lidar_data.theta.push_back(item[0].asDouble());
            lidar_data.dis.push_back(item[1].asDouble());
        }

        return lidar_data;
    }

    // Bresenham's line drawing algorithm
    std::vector<Point> bresenham(Point &start, Point &end)
    {
        // setup initial conditions
        int x1, y1, x2, y2;
        x1 = start.x;
        y1 = start.y;
        x2 = end.x;
        y2 = end.y;

        int dx = x2 - x1;
        int dy = y2 - y1;

        // determin how steep the line is 
        bool is_steep = (std::abs(dy) > std::abs(dx));
        if (is_steep)
        {

            std::swap(x1, y1);
            std::swap(x2, y2);
        }

        
        bool swapped = false;
        if (x1 > x2)
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
            swapped = true;
        }

        dx = x2 - x1;
        dy = y2 - y1;
        int error = int(dx / 2.0);
        int y_step;
        (y1 < y2) ? (y_step = 1) : (y_step = -1);

        int y = y1;
        std::vector<Point> points;

        for (int x = x1; x <= x2; x++)
        {
            Point point(0, 0);
            if (is_steep)
            {
                point.x = y;
                point.y = x;
            }
            else
            {
                point.x = x;
                point.y = y;
            }

            points.push_back(point);
            error -= std::abs(dy);

            if (error < 0)
            {
                y += y_step;
                error += dx;
            }
        }

        if (swapped)
        {
        std::reverse(points.begin(), points.end());
        }

        return points;

    }    

    // Calculates the size, and the maximum distance acording to the mesurement center
    GridMap cal_grid_map_config(
        std::vector<float> &ox, 
        std::vector<float> &oy, 
        float xy_resolusion)
    {   
        int min_x = std::round(*std::min_element(ox.begin(), ox.end()) - EXTEND_AREA / 2.0);
        int min_y = std::round(*std::min_element(oy.begin(), oy.end()) - EXTEND_AREA / 2.0);
        int max_x = std::round(*std::max_element(ox.begin(), ox.end()) + EXTEND_AREA / 2.0);
        int max_y = std::round(*std::max_element(oy.begin(), oy.end()) + EXTEND_AREA / 2.0);

        GridMap gridmap(min_x, min_y, max_x, max_y, xy_resolusion);

        return gridmap;
    }

    // compute grid map with bresenham ray casting
    GridMap generate_ray_casting_grid_map(std::vector<float> &ox, std::vector<float> &oy, float resolution)
    {
        GridMap gridmap = cal_grid_map_config(ox, oy, resolution);

        for (size_t i = 0; i < ox.size(); i++)
        {
            float x = ox[i];
            float y = oy[i];

            // x coord of the occupided area
            // detected by laser beam
            int ix = (int)std::round((x - gridmap.min_x) / resolution);
            int iy = (int)std::round((y - gridmap.min_y) / resolution);

            // std::cout << ix << " : " << iy << std::endl;

            Point center(gridmap.center_x, gridmap.center_y);
            Point occupided_cell(ix, iy);

            auto laser_beams = bresenham(center, occupided_cell);

            for (Point cell : laser_beams)
            {
                // assign all cells along laser beam
                // to free area
                gridmap.prob_map(cell.x, cell.y) = 0.0;
            }

            gridmap.prob_map(ix, iy) = 1.0;
            gridmap.prob_map(ix, iy + 1) = 1.0;
            gridmap.prob_map(ix + 1, iy) = 1.0;
            gridmap.prob_map(ix + 1, iy + 1) = 1.0;
        }    

        // gridmap.prob_map.transposeInPlace();

        return gridmap;
    }

    void run()
    {   

        rclcpp::Rate rate(20);

        auto laser_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        auto timestamp = this->get_clock()->now();
        laser_msg->header.frame_id = "map";
        laser_msg->header.stamp = timestamp;

        float angle_min = *std::min_element(lidar_data.theta.begin(), lidar_data.theta.end());
        float angle_max = *std::max_element(lidar_data.theta.begin(), lidar_data.theta.end());
        float range_min = *std::min_element(lidar_data.dis.begin(), lidar_data.dis.end());
        float range_max = *std::max_element(lidar_data.dis.begin(), lidar_data.dis.end());

        laser_msg->angle_min = angle_min;
        laser_msg->angle_max = angle_max;
        laser_msg->angle_increment = lidar_data.theta[1] - lidar_data.theta[0];
        laser_msg->range_min = range_min;
        laser_msg->range_max = range_max;

        std::vector<float> ox, oy;
        for (size_t i = 0; i < lidar_data.theta.size(); i++)
        {
            float theta = lidar_data.theta[i];
            float dist = lidar_data.dis[i];
            ox.push_back(std::sin(theta) * dist);
            oy.push_back(std::cos(theta) * dist);

            laser_msg->ranges.push_back(dist);
            laser_msg->intensities.push_back(1);
        }   

        
        GridMap gmap = generate_ray_casting_grid_map(ox, oy, RESOLUTION);
        auto gmap_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        gmap_msg->header.frame_id = "map";
        gmap_msg->info.height = gmap.xw;
        gmap_msg->info.width = gmap.yw;
        gmap_msg->info.resolution = RESOLUTION;
        gmap_msg->info.origin.position.x = -(gmap.center_x * RESOLUTION) + 0.3;
        gmap_msg->info.origin.position.y = -(gmap.center_y * RESOLUTION) + 0.25;
        gmap_msg->info.origin.orientation.z = -0.1088669;
        gmap_msg->info.origin.orientation.w = 0.9940563;
    
        std::vector<int> gmap_data;
        for (int i = 0; i < gmap.prob_map.rows(); i++)
        {
            for (int j = 0; j < gmap.prob_map.cols(); j++)
            {   
                double prob = gmap.prob_map(i, j);
                if (prob == 1.0)
                    gmap_msg->data.push_back(100);
                else if (prob == 0.0)
                    gmap_msg->data.push_back(0);
                else
                   gmap_msg->data.push_back(-1);            
            }
        }   
        
        // size_t counter = 0;
        // while (rclcpp::ok() && counter < lidar_data.theta.size())
        while (rclcpp::ok())
        {
            // laser_msg->ranges.push_back(lidar_data.dis[counter]);
            // laser_msg->intensities.push_back(1.0);
            // counter++;
            laser_publisher_->publish(*laser_msg);
            grid_publisher_->publish(*gmap_msg);
            rate.sleep();
        }
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // std::filesystem::path currentPath = std::filesystem::current_path();
    // std::cout <<  currentPath << std::endl;

    std::string file_path = ("./src/lidar2gridmap/data/lidar_me.json");
    float extend_area = 1.0;
    float resolution = 0.02;
    auto node = std::make_shared<Lidar2Grid>(file_path, extend_area, resolution);
    node->run();

    rclcpp::shutdown();
    return 0;
    
}


