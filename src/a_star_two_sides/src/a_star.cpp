#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random> 
#include <algorithm>
#include <thread> 

using namespace Eigen;
using namespace std;


struct Env
{
    MatrixXi bound_obs;
    MatrixXi obstacles;
};

class ANode
{
public:
    double G, H, F; // cost from start / heuristic cost / total cost
    VectorXi coordinate;
    ANode *parent = new ANode();
    ANode() : G(0), H(0), coordinate(VectorXi::Zero(2)), parent(nullptr) {}

    ANode(
        const double G, 
        const double H, 
        const VectorXi &coordinate,
        ANode* parent
        ) 
        {
            this->G = G;
            this->H = H;
            this->F = G + H;
            this->coordinate = coordinate;
            this->parent = parent;
        }

    void reset_f()
    {
        this->F = this->G + this->H;
    }

    static double hcost(const VectorXi& node_coordinate, const VectorXi& goal)
    {
        double dx = abs(node_coordinate(0) - goal(0));
        double dy = abs(node_coordinate(1) - goal(1));
        double h_cost = dx + dy;

        return h_cost;
    }

    static double gcost(ANode* fixed_node, const VectorXi& update_node_coord)
    {
        if (fixed_node == nullptr)
        {
            // terminate the program and show error message
            std::cerr <<"Error: fixed_node is nullptr\n" << std::endl;
            std::exit(EXIT_FAILURE);
            
        }
        double dx = abs(fixed_node->coordinate(0) - update_node_coord(0));
        double dy = abs(fixed_node->coordinate(1) - update_node_coord(1));
        double gc = sqrt(dx*dx + dy*dy);
        double g_cost = fixed_node->G + gc;

        return g_cost;
    }
};

VectorXi random_coordinate(const VectorXi& bottom_vertex, const VectorXi& top_vertex, int seed)
{
    // generate random coordinate insise maze

    // Create a random number generator and seed it
    std::default_random_engine generator(seed);

    // Create uniform distribution for x and y coordinates
    std::uniform_int_distribution<int> distribution_x(bottom_vertex(0), top_vertex(0));
    std::uniform_int_distribution<int> distribution_y(bottom_vertex(1), top_vertex(1));

    distribution_x(generator);
    distribution_y(generator);
    int x = distribution_x(generator);
    int y = distribution_y(generator);

    VectorXi coordinate(2);
    coordinate << x, y;

    // cout << "Random coordinate: \n" << coordinate.transpose() << endl;
    return coordinate;
}

MatrixXi gen_random_obstacles(
    const VectorXi& bottom_vertex, const VectorXi& top_vertex, 
    const VectorXi& start, const VectorXi& goal,
    int obs_size, int seed)
{
    // generate random obstacles
    // Create a random number generator and seed it
    std::default_random_engine generator(seed);
    // std::default_random_engine generator_two(seed);

    // Create uniform distribution for x and y coordinates
    // std::uniform_int_distribution<int> distribution_x(bottom_vertex(0) + 1, top_vertex(0) - 1);
    std::uniform_int_distribution<int> distribution_x(bottom_vertex(0) + 1, top_vertex(0) - 1);
    std::uniform_int_distribution<int> distribution_y(bottom_vertex(1) + 1, top_vertex(1) - 1);

    /// initialize
    distribution_x(generator);
    distribution_y(generator);

    MatrixXi obstacles(obs_size, 2);
    
    int counter =  0;
    while (counter < obs_size)
    {
        int x = distribution_x(generator);
        int y = distribution_y(generator);

        if ((x == start(0) && y == start(1)) || (x == goal(0) && y == goal(1)))
        {   
            continue;
        }

        obstacles.row(counter) << x, y;
        counter++;
    }

    return obstacles;
}
    

Env boundary_and_obstacles(
    const VectorXi& start, const VectorXi goal, 
    const VectorXi& top_vertex, const VectorXi& bottom_vertex, 
    const int& obs_size, int seed
)
{
    Env env;
    int height = top_vertex(1) - bottom_vertex(1);
    int width = top_vertex(0) - bottom_vertex(0);
    // left wall
    VectorXi ax = VectorXi::Zero(height);
    VectorXi ay = VectorXi::LinSpaced(height, bottom_vertex(1), top_vertex(1));

    // right wall
    VectorXi cx = VectorXi::Ones(height) * top_vertex(0);
    VectorXi cy = ay;

    // top wall
    VectorXi bx = VectorXi::LinSpaced(width, bottom_vertex(0), top_vertex(0));
    VectorXi by = VectorXi::Zero(width);

    // bottom wall
    VectorXi dx = bx;
    VectorXi dy = VectorXi::Ones(width) * top_vertex(1);

    // combine all walls
    MatrixXi bound(ax.size() + bx.size() + cx.size() + dx.size(), 2);
    bound.col(0) << ax, bx, cx, dx;
    bound.col(1) << ay, by, cy, dy;

    // generate random obstacles
    MatrixXi obstacles = gen_random_obstacles(bottom_vertex, top_vertex, start, goal, obs_size, seed);

    MatrixXi bound_obs(bound.rows() + obstacles.rows(), 2);
    bound_obs << bound, obstacles;

    env.bound_obs = bound_obs;
    env.obstacles = obstacles;

    // cout << bound_obs.rows() << endl;
    return env;
}   

class AStarTwoSide : public rclcpp::Node{
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr targets_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr env_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr end2goal_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal2end_publisher;

public:
    // rgb color vector
    vector<double> black = {0, 0, 0};
    vector<double> green = {0, 1, 0};
    vector<double> blue = {0, 0, 1};
    vector<double> red = {1, 0, 0};
    vector<double> yellow = {1, 1, 0};

    AStarTwoSide() : Node("a_star_node")
    {
        // Initialize publisher
        targets_publisher = this->create_publisher<visualization_msgs::msg::Marker>("targets", 10);
        env_publisher = this->create_publisher<visualization_msgs::msg::Marker>("env", 10);
        path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("path", 10);
        end2goal_publisher = this->create_publisher<visualization_msgs::msg::Marker>("end2goal", 10);
        goal2end_publisher = this->create_publisher<visualization_msgs::msg::Marker>("goal2end", 10);
    }

    int find_node_index(const VectorXi& coordinate, const vector<ANode*>& node_list)
    {       
        for (size_t i = 0; i < node_list.size(); i++)
        {
            if (node_list[i]->coordinate == coordinate)
            {
                return i;
            }
        }
        return 0;
    }
    
    vector<VectorXi> node_to_coordinate(const vector<ANode*>& node_list)
    {
        vector<VectorXi> coordinate_list;
        for (size_t i = 0; i < node_list.size(); i++)
        {
            coordinate_list.push_back(node_list[i]->coordinate);
        }
        return coordinate_list;
    }

    vector<VectorXi> check_node_coincide(const vector<ANode*>& close_ls1, const vector<ANode*>& closed_ls2)
    {
        // check if node in close_ls1 intersect with node in closed_ls2
        auto vec_cl1 = node_to_coordinate(close_ls1);
        auto vec_cl2 = node_to_coordinate(closed_ls2);

        vector<VectorXi> vec_intersect;

        for (const auto& coord : vec_cl1)
        {
            if (std::find(vec_cl2.begin(), vec_cl2.end(), coord) != vec_cl2.end())
            {
                vec_intersect.push_back(coord);
            }
        }
        return vec_intersect;
    }

    bool is_coor_in_list(const VectorXi& vec, const vector<VectorXi>& vec_list)
    {
        for (const auto& val : vec_list)
        {
            if (val == vec)
            {
                return true;
            }
        }
        return false;
    }

    bool is_coor_in_mat(const VectorXi& vec, const MatrixXi& mat)
    {
        for (int i = 0; i < mat.rows(); i++)
        {
            if (mat.row(i) == vec.transpose())
            {
                return true;
            }
        }
        return false;
    }

    vector<VectorXi> find_neighbor(ANode* node, const MatrixXi& obstacles, vector<VectorXi>& closed)
    {
        vector<VectorXi> neighbor_list;
        bool occup_map[3][3];

        for (int x = node->coordinate(0) - 1; x < node->coordinate(0)+2; x++)
        {
            for (int y = node->coordinate(1) - 1; y < node->coordinate(1)+2; y++)
            {
                VectorXi temp(2);
                temp << x, y;
                
                bool is_valid = true;
                for (int i = 0; i < obstacles.rows(); i++)
                {   
                    // if a node is in obstacles or equal to current node
                    // the skip it
                    if (temp.transpose()==obstacles.row(i) || temp==node->coordinate)
                    {
                        is_valid = false;
                        break;
                    }
                }

                if (is_valid)
                {
                    neighbor_list.push_back(temp);
                }
            }
        }

        // remove neighbor nodes who cross through two diagonal obstacles
        // initialize top, bottom, left, right neighbor
        VectorXi top_nei(2), bottom_nei(2), left_nei(2), right_nei(2);
        top_nei << node->coordinate(0), node->coordinate(1) + 1;
        bottom_nei << node->coordinate(0), node->coordinate(1) - 1;
        left_nei << node->coordinate(0) - 1, node->coordinate(1);
        right_nei << node->coordinate(0) + 1, node->coordinate(1);


        // check if neighbor are obstacles
        if (is_coor_in_mat(top_nei, obstacles))
        {
            occup_map[0][1] = true;
        }
        
        if (is_coor_in_mat(bottom_nei, obstacles))
        {
            occup_map[2][1] = true;
        }

        if (is_coor_in_mat(left_nei, obstacles))
        {
            occup_map[1][0] = true;
        }

        if (is_coor_in_mat(right_nei, obstacles))
        {
            occup_map[1][2] = true;
        }

        // Initilize lelf-top, right-top, left-bottom, right-bottom neighbor
        VectorXi lt_nei(2), rt_nei(2), lb_nei(2), rb_nei(2);
        lt_nei << node->coordinate(0) - 1, node->coordinate(1) + 1;
        rt_nei << node->coordinate(0) + 1, node->coordinate(1) + 1;
        lb_nei << node->coordinate(0) - 1, node->coordinate(1) - 1;
        rb_nei << node->coordinate(0) + 1, node->coordinate(1) - 1;


        // Remove neighbor nodes who cross through two diagonal obstacles
        if (is_coor_in_list(lt_nei, neighbor_list) && occup_map[0][1] && occup_map[1][0])
        {
            // cout << "erase" << endl;
            neighbor_list.erase(std::remove(neighbor_list.begin(), neighbor_list.end(), lt_nei), neighbor_list.end());
        }

        if (is_coor_in_list(rt_nei, neighbor_list) && occup_map[0][1] && occup_map[1][2])
        {
            // cout << "erase" << endl;
            neighbor_list.erase(std::remove(neighbor_list.begin(), neighbor_list.end(), rt_nei), neighbor_list.end());
        }

        if (is_coor_in_list(lb_nei, neighbor_list) && occup_map[2][1] && occup_map[1][0])
        {
            // cout << "erase" << endl;
            neighbor_list.erase(std::remove(neighbor_list.begin(), neighbor_list.end(), lb_nei), neighbor_list.end());
        }

        if (is_coor_in_list(rb_nei, neighbor_list) && occup_map[2][1] && occup_map[1][2])
        {
            // cout << "erase" << endl;
            neighbor_list.erase(std::remove(neighbor_list.begin(), neighbor_list.end(), rb_nei), neighbor_list.end());
        }
        
        // for (auto val : neighbor_list)
        // {
        //     cout << val.transpose() << endl;
        // }

        vector<VectorXi> filter_neighbor_lst;
        for (VectorXi& element : neighbor_list)
        {
            if (!is_coor_in_list(element, closed))
            {
                filter_neighbor_lst.push_back(element);
            }
        }

        return filter_neighbor_lst;
    }

    void find_path(vector<ANode*>& open_list, vector<ANode*>& closed_list, const VectorXi& goal, const MatrixXi& obstacles)
    {
        int num_nodes = open_list.size();

        for (int i = 0; i < num_nodes; i++)
        {
            // Get node with smaller total cost F
            ANode* node = open_list.front();
            vector<VectorXi> open_coordinate_list = node_to_coordinate(open_list);
            vector<VectorXi> closed_coordinate_list = node_to_coordinate(closed_list);
            vector<VectorXi> temp = find_neighbor(node, obstacles, closed_coordinate_list);
            
            for (const VectorXi& element : temp)
            {
                if (is_coor_in_list(element, closed_coordinate_list)) continue;
                else if (is_coor_in_list(element, open_coordinate_list))
                {
                    // if node is already in the open list, this mean that a path
                    // to this node has been found before. We need to check if the
                    // current path is better than the previous one and update the
                    // node accordingly

                    int ind = find_node_index(element, open_list);
                    double new_g = ANode::gcost(node, element);

                    if (new_g < open_list[ind]->G)
                    {
                        open_list[ind]->G = new_g;
                        open_list[ind]->reset_f();
                        open_list[ind]->parent = node;
                    }
                }
                else{
                    // if node is not in the open list, add it to the open list
                    double new_G = ANode::gcost(node, element);
                    double new_H = ANode::hcost(element, goal);
                    ANode* new_node = new ANode(new_G, new_H, element, node);
                    open_list.push_back(new_node);
                }
            }

            // delete node from open list and add it to closed list
            open_list.erase(std::remove(open_list.begin(), open_list.end(), node), open_list.end());
            closed_list.push_back(node);
            
            // sort open_list by total cost F
            sort(open_list.begin(), open_list.end(), [](ANode* a, ANode*b) {return a->F < b->F;});
        }
    }

        vector<VectorXi> get_path(
            const vector<ANode*>& org_closed_list, 
            const vector<ANode*>& goal_closed_list, 
            const VectorXi& coordinate)
        {
            vector<VectorXi> final_path, path_from_org, path_from_goal;
            int ind = find_node_index(coordinate, org_closed_list);
            ANode* node = org_closed_list[ind];
            
            // org_closed_list contained travelled nodes from start to intersected node
            while (node != org_closed_list.front())
            {
                path_from_org.push_back(node->coordinate);
                node = node->parent;
            }
            path_from_org.push_back(org_closed_list.front()->coordinate);
            std::reverse(path_from_org.begin(), path_from_org.end());
            int ind2 = find_node_index(coordinate, goal_closed_list);
            ANode* node2 = goal_closed_list[ind2];

            // goal_closed_list contained travelled nodes from goal to intersected node
            while (node2 != goal_closed_list.front())
            {
                path_from_goal.push_back(node2->coordinate);
                node2 = node2->parent;
            }
            path_from_org.push_back(goal_closed_list.front()->coordinate);
            
            // concatenate two path to get the final path
            final_path = path_from_org;
            final_path.insert(final_path.end(), path_from_goal.begin(), path_from_goal.end());

            return final_path;
        }


        void setup_env_msg(
            const Env& env,
            shared_ptr<visualization_msgs::msg::Marker> env_marker,
            const vector<double>& color,
            double scale
        )
        {
            env_marker->header.frame_id = "map";
            env_marker->header.stamp = this->now();
            env_marker->ns = "env";
            env_marker->id = 0;
            env_marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
            env_marker->action = visualization_msgs::msg::Marker::ADD;
            env_marker->pose.orientation.w = 1.0;
            env_marker->scale.x = scale;
            env_marker->scale.y = scale;
            env_marker->scale.z = scale;
            env_marker->color.r = color[0];
            env_marker->color.g = color[1];
            env_marker->color.b = color[2];
            env_marker->color.a = 1.0;

            for (int i = 0; i < env.bound_obs.rows(); i++)
            {
                geometry_msgs::msg::Point p;
                p.x = env.bound_obs(i, 0);
                p.y = env.bound_obs(i, 1);
                p.z = 0;
                env_marker->points.push_back(p);
            }

        }

        void setup_targets_msg(
            const VectorXi& start, 
            const VectorXi& end,
            shared_ptr<visualization_msgs::msg::Marker> target_marker,
            const vector<double>& color,
            double scale
        )
        {
            target_marker->header.frame_id = "map";
            target_marker->header.stamp = this->now();
            target_marker->ns = "targets";
            target_marker->id = 0;
            target_marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
            target_marker->action = visualization_msgs::msg::Marker::ADD;
            target_marker->pose.orientation.w = 1.0;
            target_marker->scale.x = scale;
            target_marker->scale.y = scale;
            target_marker->scale.z = scale;
            target_marker->color.r = color[0];
            target_marker->color.g = color[1];
            target_marker->color.b = color[2];
            target_marker->color.a = 1.0;

            geometry_msgs::msg::Point p1;
            p1.x = start(0);
            p1.y = start(1);
            p1.z = 0;
            target_marker->points.push_back(p1);

            geometry_msgs::msg::Point p2;
            p2.x = end(0);
            p2.y = end(1);
            p2.z = 0;
            target_marker->points.push_back(p2);
        }

        void setup_g2e_msg(
            const vector<ANode*>& closed_list,
            shared_ptr<visualization_msgs::msg::Marker>& g2e_marker,
            double scale,
            const vector<double>& color
        )
        {
            g2e_marker->header.frame_id = "map";
            g2e_marker->header.stamp = this->now();
            g2e_marker->ns = "g2e";
            g2e_marker->id = 0;
            g2e_marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
            g2e_marker->action = visualization_msgs::msg::Marker::ADD;
            g2e_marker->pose.orientation.w = 1.0;
            g2e_marker->scale.x = scale;
            g2e_marker->scale.y = scale;
            g2e_marker->scale.z = scale;
            g2e_marker->color.r = color[0];
            g2e_marker->color.g = color[1];
            g2e_marker->color.b = color[2];
            g2e_marker->color.a = 1.0;

            for (const auto& node : closed_list)
            {
                geometry_msgs::msg::Point p;
                p.x = node->coordinate(0);
                p.y = node->coordinate(1);
                p.z = 0;
                g2e_marker->points.push_back(p);
            }
        }

        void setup_e2g_msg(
            const vector<ANode*>& closed_list,
            shared_ptr<visualization_msgs::msg::Marker>& e2g_marker,
            double scale, 
            const vector<double>& color
        )
        {
            e2g_marker->header.frame_id = "map";
            e2g_marker->header.stamp = this->now();
            e2g_marker->ns = "e2g";
            e2g_marker->id = 0;
            e2g_marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
            e2g_marker->action = visualization_msgs::msg::Marker::ADD;
            e2g_marker->pose.orientation.w = 1.0;
            e2g_marker->scale.x = scale;
            e2g_marker->scale.y = scale;
            e2g_marker->scale.z = scale;
            e2g_marker->color.r = color[0];
            e2g_marker->color.g = color[1];
            e2g_marker->color.b = color[2];
            e2g_marker->color.a = 1.0;

            for (const auto& node : closed_list)
            {
                geometry_msgs::msg::Point p;
                p.x = node->coordinate(0);
                p.y = node->coordinate(1);
                p.z = 0;
                e2g_marker->points.push_back(p);
            }
        }

        void setup_path_msg(
            const vector<VectorXi>& path,
            shared_ptr<visualization_msgs::msg::Marker>& path_marker,
            double scale,
            const vector<double>& color
        )
        {
            path_marker->header.frame_id = "map";
            path_marker->header.stamp = this->now();
            path_marker->ns = "path";
            path_marker->id = 0;
            path_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker->action = visualization_msgs::msg::Marker::ADD;
            path_marker->lifetime = rclcpp::Duration::from_seconds(0);
            path_marker->pose.orientation.w = 1.0;
            path_marker->scale.x = scale;
            path_marker->scale.y = scale;
            path_marker->scale.z = scale;
            path_marker->color.r = color[0];
            path_marker->color.g = color[1];
            path_marker->color.b = color[2];
            path_marker->color.a = 1.0;

            for (const auto& node : path)
            {
                geometry_msgs::msg::Point p;
                p.x = node(0);
                p.y = node(1);
                p.z = 0;
                path_marker->points.push_back(p);
            }
        }
        

        void run(const VectorXi& start, const VectorXi& end, const Env& env)
        {
            rclcpp::Rate rate(1);
            
            double env_scale = 2;
            auto env_marker = std::make_shared<visualization_msgs::msg::Marker>();
            setup_env_msg(env, env_marker, black, env_scale);

            double target_scale = 7;
            auto target_marker = std::make_shared<visualization_msgs::msg::Marker>();
            setup_targets_msg(start, end, target_marker, blue, target_scale);

            MatrixXi bound_obs = env.bound_obs;
            MatrixXi obstacles = env.obstacles;

            ANode* origin = new ANode(0, ANode::hcost(start, end), start, nullptr);
            ANode* goal = new ANode(0, ANode::hcost(end, start), end, nullptr);


            // Initialize open and closed list from origin to goal
            vector<ANode*> origin_open = {origin};
            vector<ANode*> origin_closed = {};

            // Initialize open and closed list from goal to origin
            vector<ANode*> goal_open = {goal};
            vector<ANode*> goal_closed = {};

            // Initialize target
            VectorXi target_goal = end;
            vector<VectorXi> final_path;

            double g2e_scale = 1;
            double e2g_scale = 1;
            double path_scale = 1;
            auto g2e_marker = std::make_shared<visualization_msgs::msg::Marker>();
            auto e2g_marker = std::make_shared<visualization_msgs::msg::Marker>();
            auto path_marker = std::make_shared<visualization_msgs::msg::Marker>();
            int counter = 0;

            while (rclcpp::ok())
            {
                counter++;
                cout << "[INFO] Iteration #" << counter << "....." << endl;
                // Searching from start to end
                find_path(origin_open, origin_closed, target_goal, bound_obs);

                // Update new origin target
                VectorXi target_origin = origin_open.front()->coordinate;

                // Searching from end to updated origin target
                find_path(goal_open, goal_closed, target_origin, bound_obs);

                // Update new goal target
                VectorXi target_goal = goal_open.front()->coordinate;             

                // check if there is intersected node
                // between tww closed paths

                vector<VectorXi> intersected_node = check_node_coincide(origin_closed, goal_closed);
                setup_g2e_msg(origin_closed, g2e_marker, g2e_scale, yellow);
                setup_e2g_msg(goal_closed, e2g_marker, e2g_scale, green);

                if (intersected_node.size())
                {
                    // if there is intersected node, then we have found the path
                    // any intersected node can be used to look for final path
                    // we use the first intersected node in this case

                    final_path = get_path(origin_closed, goal_closed, intersected_node.front());
                    cout << "[INFO] Path is found! " << endl;
                    // for (const auto& val : final_path)
                    // {
                    //     cout << val.transpose() << endl;
                    // }
                }

                env_publisher->publish(std::move(*env_marker));
                targets_publisher->publish(std::move(*target_marker));
                end2goal_publisher->publish(std::move(*e2g_marker));
                goal2end_publisher->publish(std::move(*g2e_marker));

                // setup publishing rate
                if (final_path.size())
                {
                    setup_path_msg(final_path, path_marker, path_scale, red);
                    g2e_marker->points.clear();
                    e2g_marker->points.clear();

                    end2goal_publisher->publish(std::move(*e2g_marker));
                    goal2end_publisher->publish(std::move(*g2e_marker));                    
                    path_publisher->publish(std::move(*path_marker));
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    break;
                }

                rate.sleep();
            }
        }
};


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    VectorXi bottom_vertex(2);
    bottom_vertex << 0, 0;

    VectorXi top_vertex(2);
    top_vertex << 150, 150;

    // Set up seed number for static random number generator
    int seed = 2014;
    auto start = random_coordinate(bottom_vertex, top_vertex, seed-1);
    auto goal = random_coordinate(bottom_vertex, top_vertex, seed+1);

    AStarTwoSide astar;

    int num_obstacles = 250;
    Env env = boundary_and_obstacles(start, goal, top_vertex, bottom_vertex, num_obstacles, seed);

    astar.run(start, goal, env);

    rclcpp::shutdown();
    return 0;
}