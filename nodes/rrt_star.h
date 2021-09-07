#ifndef DYNAMICRRT_STAR_RRT_STAR_H
#define DYNAMICRRT_STAR_RRT_STAR_H
#include "node.h"
#include "helper_functions.h"
#include <chrono>
#include "obstacle.h"
#include <stack>
#include "GMRF.h"

using namespace __gnu_cxx;
typedef vector<node*> VectNode;
using namespace std;

class rrt_star {
private:
    // Q_s passcode
    int global_pass = 0;

    global_params parameters;

    // RRT grid
    float cell_size;
    int n_x;
    int n_y;
    vector<vector<list<node*>>> Grid;

    // Obstacle grid
    vector<vector<list<obstacle*>>> ObstclGrid;
    vector<vector<list<obstacle*>>> RealObstacles;
    vector<obstacle*> mapped_obstacles;

    // goal position
    Point goal_pos;

    // rewiring radius
    float r;

    // rewiring queues
    stack<node*> Qr_prior;
    queue<node *> Qr;
    vector<node*> Qs_toBePushed;
    queue<node*> Q_s;

    Range x_range{};
    Range y_range{};

    float rewire_time;
    node* root_node;
    node* goal_node;

    vector<node*> last_push;
    // GMRF
    int nx_gmrf;
    int ny_gmrf;
    double cs_gmrf;
    int gmrf_update_counter = 0;

    // autonomous exploration
    node* max_utility_node = nullptr;
    node* dummy_node;

    node* find_nearest(Point x_rand);
    vector<node*> near_nodes(node* c_node);
    void rewire_single_node(node* node_parent, node* node_child, bool root);
    bool change_node_parent(node* child_node, node* parent_node);

    void iterate_nodes(node *c_node, bool root = true);
    void add_to_tree(node* new_node);
    bool checkcollision(MyPose pose1, MyPose pose2);
    void sort_in(node* to_sort);

public:

    void rewire();
    void rewire_f_r();

    rrt_star(global_params global_P, MyPose init_pose_, list<obstacle*> obstacles);
    float utility;
    MyPose current_pose;

    node* closest_to_goal;
    GMRF* gmrf_belief;

    VectNode all_nodes;

    VectAllP all_points;
    // obstacles
    void add_obstcl(obstacle* obstcl);

    // tree expansion
    void expand(bool advance=true);
    // closest node to goal
    bool closest_goal();
    void generate_path();
    bool update_goal(Point goal_);
    void rotate_root(float dtheta){
        root_node->set_angle(dtheta);
        root_node->propagate_costs(true, false, true);
        Qr_prior.push(root_node);};
    node* return_root(){return root_node;};
    void update_information_utility_of_near_nodes();
    vector<vector<list<obstacle*>>>* return_obstacle_grid(){return &ObstclGrid;};
    node* return_max_utility_node(){return max_utility_node;};
    vector<MyPose> path;
    void update_pose(MyPose new_pose_);
    void set_root_node(MyPose new_pose);
    void set_goal_node(node* goal){goal_node = goal;};
    node* return_goal_node(){return goal_node;};
    void condition_GMRF(Point location, float meas){
        gmrf_belief->condition(location, meas);
        gmrf_belief->update_mean_uncertainty();
        update_information_utility_of_near_nodes();};

};


#endif //DYNAMICRRT_STAR_RRT_STAR_H
