#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random> 
#include <algorithm>

using namespace Eigen;
using namespace std;


struct Env
{
    MatrixXi bound_obs;
    MatrixXi obstacles;
};

class Node
{
public:
    double G, H, F; // cost from start / heuristic cost / total cost
    VectorXi coordinate;
    Node *parent = new Node();
    Node() : G(0), H(0), coordinate(VectorXi::Zero(2)), parent(nullptr) {}

    Node(
        const double G, 
        const double H, 
        const VectorXi &coordinate,
        Node* parent
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

    static double gcost(Node* fixed_node, const VectorXi& update_node_coord)
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

        return gc;
    }
};

VectorXi random_coordinate(const VectorXi& bottom_vertex, const VectorXi& top_vertex, int seed)
{
    // generate random coordinate insise maze

    // Create a random number generator and seed it
    std::default_random_engine generator(seed);
    // std::default_random_engine generator_two(seed);

    // Create uniform distribution for x and y coordinates
    // std::uniform_int_distribution<int> distribution_x(bottom_vertex(0) + 1, top_vertex(0) - 1);
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

class AStarTwoSide{
public:
    int find_node_index(const VectorXi& coordinate, const vector<Node*>& node_list)
    {       
        for (int i = 0; i < node_list.size(); i++)
        {
            if (node_list[i]->coordinate == coordinate)
            {
                return i;
            }
        }
        return 0;
    }
    
    vector<VectorXi> node_to_coordinate(const vector<Node*>& node_list)
    {
        vector<VectorXi> coordinate_list;
        for (int i = 0; i < node_list.size(); i++)
        {
            coordinate_list.push_back(node_list[i]->coordinate);
        }
        return coordinate_list;
    }

    vector<VectorXi> check_node_coincide(const vector<Node*>& close_ls1, const vector<Node*>& closed_ls2)
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

    vector<VectorXi> find_neighbor(Node* node, const MatrixXi& obstacles, vector<VectorXi>& closed)
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

        return neighbor_list;
    }

    void find_path(vector<Node*>& open_list, vector<Node*>& closed_list, const VectorXi& goal, const MatrixXi& obstacles)
    {
        int num_nodes = open_list.size();

        for (int i = 0; i < num_nodes; i++)
        {
            // Get node with smaller total cost F
            Node* node = open_list.front();
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
                    double new_g = Node::gcost(node, element);

                    if (new_g < open_list[ind]->G)
                    {
                        open_list[ind]->G = new_g;
                        open_list[ind]->reset_f();
                        open_list[ind]->parent = node;
                    }
                }
                else{
                    // if node is not in the open list, add it to the open list
                    double new_G = Node::gcost(node, element);
                    double new_H = Node::hcost(element, goal);
                    Node* new_node = new Node(new_G, new_H, element, node);
                    open_list.push_back(new_node);
                }
            }

            // delete node from open list and add it to closed list
            // cout << "Before erase" << open_list.size() << endl;
            open_list.erase(std::remove(open_list.begin(), open_list.end(), node), open_list.end());
            // cout << "After erase" << open_list.size() << endl;
            closed_list.push_back(node);
            
            // sort open_list by total cost F
            sort(open_list.begin(), open_list.end(), [](Node* a, Node*b) {return a->F < b->F;});
        }
    }

        vector<VectorXi> get_path(
            const vector<Node*>& org_closed_list, 
            const vector<Node*>& goal_closed_list, 
            const VectorXi& coordinate)
        {
            vector<VectorXi> final_path, path_from_org, path_from_goal;
            int ind = find_node_index(coordinate, org_closed_list);
            Node* node = org_closed_list[ind];
            
            // org_closed_list contained travelled nodes from start to intersected node
            while (node != org_closed_list.front())
            {
                path_from_org.push_back(node->coordinate);
                node = node->parent;
            }
            path_from_org.push_back(org_closed_list.front()->coordinate);
            std::reverse(path_from_org.begin(), path_from_org.end());
            int ind2 = find_node_index(coordinate, goal_closed_list);
            Node* node2 = goal_closed_list[ind2];

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

        void run(const VectorXi& start, const VectorXi& end, const Env& env)
        {
            MatrixXi bound_obs = env.bound_obs;
            MatrixXi obstacles = env.obstacles;

            Node* origin = new Node(0, Node::hcost(start, end), start, nullptr);
            Node* goal = new Node(0, Node::hcost(end, start), end, nullptr);

            // Initialize open and closed list from origin to goal
            vector<Node*> origin_open = {origin};
            vector<Node*> origin_closed = {};

            // Initialize open and closed list from goal to origin
            vector<Node*> goal_open = {goal};
            vector<Node*> goal_closed = {};

            // Initialize target
            VectorXi target_goal = end;
            vector<VectorXi> final_path;
            while (true)
            {

                cout << "Update" << endl;
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

                if (intersected_node.size())
                {
                    // if there is intersected node, then we have found the path
                    // any intersected node can be used to look for final path
                    // we use the first intersected node in this case

                    final_path = get_path(origin_closed, goal_closed, intersected_node.front());
                    cout << "Path found " << endl;
                    break;
                }
            }
        }
};




int main()
{

    // double G = 0.0;
    // double H = 30.0;
    // VectorXi coord(2);
    // coord << 37, 20;
    // VectorXi node_coord(2);
    // node_coord << 31, 51;
    // VectorXi update_coord(2);
    // update_coord << 38, 19;

    // Node* node1 = new Node(G, H, coord, nullptr);
    // Node* fixed_node = new Node(G,H, coord, nullptr);
    // // node1->hcost(node_coord, goal);
    // node1->gcost(fixed_node, update_coord);


    VectorXi bottom_vertex(2);
    bottom_vertex << 0, 0;

    VectorXi top_vertex(2);
    top_vertex << 60, 60;

    int seed = 99;
    auto start = random_coordinate(bottom_vertex, top_vertex, seed-1);
    auto goal = random_coordinate(bottom_vertex, top_vertex, seed+1);

    AStarTwoSide astar;

    int num_obstacles = 1000;
    Env env = boundary_and_obstacles(start, goal, top_vertex, bottom_vertex, num_obstacles, seed);

    astar.run(start, goal, env);
    
    // fixed_node->coordinate = coords;
    // vector<VectorXi> closed = {coords};
    // auto neighbor = astar.find_neighbor(node1, obstacles, closed);
    // cout << "Num neighbor: " << neighbor.size() << endl;

    return 0;
}