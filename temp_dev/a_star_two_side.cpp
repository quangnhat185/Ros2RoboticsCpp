#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random> 

using namespace Eigen;
using namespace std;


struct BoundObs
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
            this->coordinate = coordinate;
            this->parent = parent;

        }

    void reset_f()
    {
        this->F = this->G + this->H;
    }

    double hcost(const VectorXi& node_coordinate, const VectorXi& goal)
    {
        double dx = abs(node_coordinate(0) - goal(0));
        double dy = abs(node_coordinate(1) - goal(1));
        double h_cost = dx + dy;

        return h_cost;
    }

    double gcost(Node* fixed_node, const VectorXi& update_node_coord)
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

MatrixXi gen_random_obstacles(const VectorXi& bottom_vertex, const VectorXi& top_vertex, int obs_size, int seed)
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
    
    for (int i = 0; i < obs_size; i++)
    {
        int x = distribution_x(generator);
        int y = distribution_y(generator);
        obstacles.row(i) << x, y;
    }

    // cout << "Random obstacles: \n" << obstacles << endl;
    return obstacles;
}
    

BoundObs boundary_and_obstacles(
    const VectorXi& start, const VectorXi goal, 
    const VectorXi& top_vertex, const VectorXi& bottom_vertex, 
    const int& obs_size, int seed
)
{
    BoundObs res;
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
    MatrixXi obstacles = gen_random_obstacles(bottom_vertex, top_vertex, obs_size, seed);

    MatrixXi bound_obs(bound.rows() + obstacles.rows(), 2);
    bound_obs << bound, obstacles;

    res.bound_obs = bound_obs;
    res.obstacles = obstacles;

    // cout << bound_obs.rows() << endl;
    return res;
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
};




int main()
{

    double G = 0.0;
    double H = 30.0;
    VectorXi coord(2);
    coord << 37, 20;
    VectorXi node_coord(2);
    node_coord << 31, 51;
    VectorXi update_coord(2);
    update_coord << 38, 19;

    Node* node1 = new Node(G, H, coord, nullptr);
    Node* fixed_node = new Node(G,H, coord, nullptr);
    // node1->hcost(node_coord, goal);
    node1->gcost(fixed_node, update_coord);


    VectorXi bottom_vertex(2);
    bottom_vertex << 0, 0;

    VectorXi top_vertex(2);
    top_vertex << 60, 60;

    int seed = 99;
    auto start = random_coordinate(bottom_vertex, top_vertex, seed-1);
    auto goal = random_coordinate(bottom_vertex, top_vertex, seed+1);

    int a = 0, b = 0;
    vector<Node*> node_list_one;
    vector<Node*> node_list_two;

    for (int i = 0; i < 10; i++)
    {
        VectorXi coord(2);
        coord << a, b;
        Node* node = new Node(G, H, coord, nullptr);
        if (i < 5)
        {
            node_list_one.push_back(node);
            node_list_two.push_back(node);
        }
        else
        {
            node_list_two.push_back(node);
        }
        a++;
        b++;
    }

    AStarTwoSide astar;

    VectorXi coords(2);
    coords << 3,3;
    // auto res =  astar.check_node_coincide(node_list_one, node_list_two);
    // for (auto val : res)
    // {
    //     cout << val.transpose() << endl;
    // }

    int num_obstacles = 1000;
    boundary_and_obstacles(start, goal, top_vertex, bottom_vertex, num_obstacles, seed);

    return 0;
}