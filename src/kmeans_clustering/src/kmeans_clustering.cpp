#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <random>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"


struct Points
{
    std::vector<float> x;
    std::vector<float> y;
};

struct Objs
{
    std::vector<float> cx;
    std::vector<float> cy;
};

class Clusters
{
private:
    std::vector<float> x, y, center_x, center_y;
    std::vector<int> labels;

public:
    int n_data, n_label;


    Clusters(std::vector<float> &x, std::vector<float> &y, int n_label) : n_data(x.size())
    {
        this->x = x;
        this->y = y;
        this->n_label = n_label;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution dist(0, n_label - 1);   
        std::uniform_real_distribution<float> dist_color(0, 1);     

        for (int i = 0; i < n_data; i++)
        {
            labels.push_back(dist(gen));
        }

        center_x  = std::vector<float>(n_label, 0);
        center_y  = std::vector<float>(n_label, 0);
    }

    Points _get_labeled_x_y(int target_label)
    {
        Points ret_xy;

        
        for (int i = 0; i < (int)labels.size(); i++)
        {
            // std::cout << "labels[i]: " << labels[i] << std::endl;
            if (labels[i] == target_label)
            {
                ret_xy.x.push_back(x[i]);
                ret_xy.y.push_back(y[i]);
            }
        }
        
        return ret_xy;
    }

    std::vector<float> getX()
    {
        return x;
    }

    std::vector<float> getY()
    {
        return y;
    }    

    void calc_centroid()
    {
        for (int label = 0; label < n_label; label++)
        {
            Points ret_xy = _get_labeled_x_y(label);
            // std::cout << n_label << std::endl;
            center_x[label] = std::accumulate(ret_xy.x.begin(), ret_xy.x.end(), 0) / float(ret_xy.x.size());
            center_y[label] = std::accumulate(ret_xy.y.begin(), ret_xy.y.end(), 0) / float(ret_xy.y.size());
        }
    }

    float update_cluster()
    {
        float cost = 0.0;

        for (int i = 0; i < n_data; i++)
        {
            std::vector<float> center_dist;
            float px = x[i];
            float py = y[i];

            for (int ic = 0; ic < (int)center_x.size(); ic++)
            {
                float dx = px - center_x[ic];
                float dy = py - center_y[ic];

                center_dist.push_back(sqrt(pow(dx, 2) + pow(dy, 2)));
            }

            // find closest center and assign new value
            auto min_dist = std::min_element(center_dist.begin(), center_dist.end());
            int min_id = std::distance(center_dist.begin(), min_dist);
            labels[i] = min_id;
            // std::cout << "min_id: " << min_id << std::endl;
            
            cost += *min_dist;
        }
        return cost;
    }

    void printClusterCenX()
    {        
        std::cout << "[ ";
        for (float val : center_x)
        {
            std::cout << val << ", ";
        }
        std::cout << "]" << std::endl;
    }
};

class KmeansClustering : public rclcpp::Node
{
private:
    float DCOST_TH = 0.1;
    int MAX_LOOP = 10;  

    Objs objs;
    int n_points;
    float rand_d;
    int n_cluster;

    std::unordered_map<int, std::array<float, 3>> color_map;    

    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obj_pose_publisher_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr kmeans_publisher_;

public: 

    KmeansClustering(float dcost, Objs objs, int n_points, int n_cluster, float rand_d) : Node("kmeans_publisher"), DCOST_TH(dcost), objs(objs), n_points(n_points)
    {

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist_color(0, 1);     

        // obj_pose_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>
        // ("obj_pose", 10);
        // cluster_pose_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_pose", 1);

        kmeans_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_pose", 10);

        this->n_cluster = n_cluster;
        this->rand_d = rand_d;

        for (int i = 0; i < n_cluster; i++)
        {
            color_map[i] = {dist_color(gen), dist_color(gen), dist_color(gen)};
        }        
    }

    Points cal_raw_data(Objs &objs, int &n_points, float &rand_d)
    {

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0, 1.0);

        Points raw_data;
        for (size_t i = 0; i < objs.cx.size(); i++)
        {
            for (int j = 0; j < n_points; j++)
            {   
                float rand_val = dist(gen);
                raw_data.x.push_back(objs.cx[i] + rand_d * (rand_val - 0.5));

                rand_val = dist(gen);
                raw_data.y.push_back(objs.cy[i] + rand_d * (rand_val - 0.5));
            }
        }

        return raw_data;
    }    


    void update_position(Objs &objs)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        for (size_t i = 0; i < objs.cx.size(); i++)
        {   
            float rand_val = dist(gen);
            objs.cx[i] += rand_val;

            rand_val = dist(gen);
            objs.cy[i] += rand_val;
        }
}    

    Clusters klmeans_clustering(Points data, int n_cluster)
    {
        Clusters clusters(data.x, data.y, n_cluster);
        clusters.calc_centroid();

        float pre_cost = 10000;

        for (int iter = 0; iter < MAX_LOOP; iter++)
        {
            std::cout << " loop: " << iter << std::endl;
            float cost = clusters.update_cluster();
            clusters.calc_centroid();

            float d_cost = std::abs(pre_cost - cost);
            if (d_cost < DCOST_TH)
            {
                break;
            }

            pre_cost = cost;
        }

    return clusters;
    }    

    // std::unique_ptr<visualization_msgs::msg::MarkerArray>setup_obj_pose_msg(
    //     Objs & objs,
    //     float scale
    // )
    // {

    //     auto pose_objs = std::make_unique<visualization_msgs::msg::MarkerArray>();

    //     for (int i=0; i<(int)objs.cx.size(); i++)
    //     {
    //         auto obj_pose_msg = std::make_unique<visualization_msgs::msg::Marker>();
    //         auto timestamp = this->get_clock()->now();
    //         obj_pose_msg->header.frame_id = "map";
    //         obj_pose_msg->header.stamp = timestamp;
    //         obj_pose_msg->ns = "poseObj";
    //         obj_pose_msg->id = i;
    //         obj_pose_msg->type = visualization_msgs::msg::Marker::SPHERE;
    //         obj_pose_msg->action = visualization_msgs::msg::Marker::ADD;
    //         obj_pose_msg->scale.x = scale;
    //         obj_pose_msg->scale.y = scale;
    //         obj_pose_msg->scale.z = scale;
    //         obj_pose_msg->color.a = 1;
    //         obj_pose_msg->color.r = 1;
    //         obj_pose_msg->color.g = 0;
    //         obj_pose_msg->color.b = 0;
    //         obj_pose_msg->pose.position.x = objs.cx[i];
    //         obj_pose_msg->pose.position.y = objs.cy[i];
    //         obj_pose_msg->pose.orientation.z = 0.0;    

    //         pose_objs->markers.push_back(*obj_pose_msg);                
    //     }

    //     return pose_objs;
    // }


    // std::shared_ptr<visualization_msgs::msg::MarkerArray> setup_cluster_msg(
    //     Clusters &clusters,
    //     float scale
    // )
    // {   
    //     auto pose_cluster_arr = std::make_shared<visualization_msgs::msg::MarkerArray>();
    //     auto point_pose = std::make_unique<visualization_msgs::msg::Marker>();

    //     for (int i = 0; i < n_cluster; i++)
    //     {
    //         Points c_points = clusters._get_labeled_x_y(i);
    //         std::cout << "c_points.x.size() " << c_points.x.size() << std::endl;

    //         for (int j = 0; j < (int)c_points.x.size(); j++)
    //         {
    //             auto timestamp = this->get_clock()->now();
    //             point_pose->header.frame_id = "map";
    //             point_pose->header.stamp = timestamp;
    //             point_pose->ns = "posePoints" + std::to_string(i);
    //             point_pose->id = j;
    //             point_pose->type = visualization_msgs::msg::Marker::SPHERE;
    //             point_pose->action = visualization_msgs::msg::Marker::ADD;
    //             point_pose->scale.x = scale;
    //             point_pose->scale.y = scale;
    //             point_pose->scale.z = scale;
    //             point_pose->color.a = 1;

    //             point_pose->color.r = color_map[i][0];
    //             point_pose->color.g = color_map[i][1];
    //             point_pose->color.b = color_map[i][2];

    //             point_pose->pose.position.x = c_points.x[j];
    //             point_pose->pose.position.y = c_points.y[j];
    //             point_pose->pose.orientation.z = 0.0; 

    //             pose_cluster_arr->markers.push_back(std::move(*point_pose));  
    //         }
    //     }
    //     return pose_cluster_arr;
    // }

    std::shared_ptr<visualization_msgs::msg::MarkerArray> setup_kmeans_msg(
        Objs &objs,
        Clusters &clusters,
        float scale_objs,
        float scale_cluster
    )
    {   
        auto pose_cluster_arr = std::make_shared<visualization_msgs::msg::MarkerArray>();
        auto obj_pose_msg = std::make_unique<visualization_msgs::msg::Marker>();
        auto point_pose = std::make_unique<visualization_msgs::msg::Marker>();

        for (int i=0; i<(int)objs.cx.size(); i++)
        {
            auto timestamp = this->get_clock()->now();
            obj_pose_msg->header.frame_id = "map";
            obj_pose_msg->header.stamp = timestamp;
            obj_pose_msg->ns = "poseObj";
            obj_pose_msg->id = i;
            obj_pose_msg->type = visualization_msgs::msg::Marker::SPHERE;
            obj_pose_msg->action = visualization_msgs::msg::Marker::ADD;
            obj_pose_msg->scale.x = scale_objs;
            obj_pose_msg->scale.y = scale_objs;
            obj_pose_msg->scale.z = scale_objs;
            obj_pose_msg->color.a = 1;
            obj_pose_msg->color.r = 1;
            obj_pose_msg->color.g = 0;
            obj_pose_msg->color.b = 0;
            obj_pose_msg->pose.position.x = objs.cx[i];
            obj_pose_msg->pose.position.y = objs.cy[i];
            obj_pose_msg->pose.orientation.z = 0.0;    

            pose_cluster_arr->markers.push_back(*obj_pose_msg);                
        }

        for (int i = 0; i < n_cluster; i++)
        {
            Points c_points = clusters._get_labeled_x_y(i);
            std::cout << "c_points.x.size() " << c_points.x.size() << std::endl;

            for (int j = 0; j < (int)c_points.x.size(); j++)
            {
                auto timestamp = this->get_clock()->now();
                point_pose->header.frame_id = "map";
                point_pose->header.stamp = timestamp;
                point_pose->ns = "posePoints" + std::to_string(i);
                point_pose->id = j;
                point_pose->type = visualization_msgs::msg::Marker::SPHERE;
                point_pose->action = visualization_msgs::msg::Marker::ADD;
                point_pose->scale.x = scale_cluster;
                point_pose->scale.y = scale_cluster;
                point_pose->scale.z = scale_cluster;
                point_pose->color.a = 1;

                point_pose->color.r = color_map[i][0];
                point_pose->color.g = color_map[i][1];
                point_pose->color.b = color_map[i][2];

                point_pose->pose.position.x = c_points.x[j];
                point_pose->pose.position.y = c_points.y[j];
                point_pose->pose.orientation.z = 0.0; 

                pose_cluster_arr->markers.push_back(std::move(*point_pose));  
            }
        }
        return pose_cluster_arr;
    }

    void run()
    {   

        rclcpp::Rate rate(5);

        while (rclcpp::ok())
        {

            update_position(objs);
            Points points = cal_raw_data(objs, n_points, rand_d);
            auto clusters = klmeans_clustering(points, n_cluster);   
            // clusters.printClusterCenX();

            // std::cout << objs.cx[0] << std::endl;
            // std::cout << "clusters.getX().size(): " << clusters.getX().size() << std::endl;

            float scale_objs = 1.5;
            // auto pose_objs_arr = setup_obj_pose_msg(objs, scale_objs);
            // obj_pose_publisher_->publish(std::move(*pose_objs_arr));

            float scale_point_cluster = 0.5;
            // auto pose_cluster_arr = setup_cluster_msg(clusters, scale_point_cluster);
            auto kmeans_cluster_arr = setup_kmeans_msg(objs, clusters, scale_objs, scale_point_cluster);

            // cluster_pose_publisher_->publish(std::move(*pose_cluster_arr));

            kmeans_publisher_->publish(std::move(*kmeans_cluster_arr));
            kmeans_cluster_arr->markers.clear();
            kmeans_publisher_->publish(std::move(*kmeans_cluster_arr));

            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);

    float dcost = 0.1;
    Objs objs;
    objs.cx = {0.0, 3.0, 5.0};
    objs.cy = {0.0, 1.0, 2.5};
    int n_points = 20;
    float rand_d = 3.0;
    int n_cluster = 3;    

    auto node = std::make_shared<KmeansClustering>(dcost, objs, n_points, n_cluster, rand_d);
    node->run();

    rclcpp::shutdown();
    return 0;
}