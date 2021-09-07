#ifndef DYNAMICRRT_STAR_NODE_H
#define DYNAMICRRT_STAR_NODE_H
#include<array>
#include "helper_functions.h"
#include "GMRF.h"
using namespace std;

class node {
private:
    float cost;
    float utility;
    float reward;
    float cost_to_parent;
    float cost_steering;
    float weight_steering;
    float weight_distance;
    float weight_info;
    list<node*> children;
    MyPose pose;
    node* parent;
    float collision_penalty = 0;
    int parent_edge_id;
    bool dynamic_collision = false;
    int path_length = 1;
    vector<node*> path_nodes; // needed to prevent cycles
    // GMRF
    vector<int> shape_inds;

    vector<float> shape_weights;
    float information_utility = 0;
    float path_info_utility = 0;
    float path_cost_steering = 0;
    float dist_travelled = 0;
    // Backpropagation
    tuple<vector<int>, vector<float>> phi_i;

    list<tuple<vector<int>, vector<float>>*> path_reward;
    bool recalculate_path_reward = true;
    float current_path_reward = 0;
    node* most_informative_child;
    global_params parameters;
    float discount;
    // GMRF
    GMRF* gmrf_belief;

    // pointer to max utility node
    node** max_utility_node;

    // Q_s passcode
    int code = 0;

public:

    // obstacles
    float obstacle_discount = 0;
    bool close_to_border = false;
    float border_discount = 0;
    array<array<float, 2>, 2> border_penalization_sector;


    bool has_child;
    bool has_parent;
    node(global_params parameters_, MyPose pose_, GMRF* gmrf_belief, vector<int> shape_inds_,
             vector<float> shape_weights_, tuple<vector<int>, vector<float>> phi_, node** best_node, node* parent_ = nullptr);
    void add_child(node* child_node);
    void remove_child(node* child_node);
    void propagate_costs(bool update_costs=true, bool cost_steer = false, bool first = false);
    void calc_costs();
    float return_costs(){return cost;}; // NOLINT(readability-make-member-function-const)
    float return_costs_wo_info(){return cost - parameters.weight_information * exp(-path_info_utility/path_length);};///(float)path_length);};
    float return_info_utility(){return information_utility;};
    float return_path_info_utility(){return path_info_utility;};
    float return_path_cost_steering(){return path_cost_steering;};
    float return_dist_travelled(){return dist_travelled;};
    float return_steering_cost(){return cost_steering;};
    float calc_costs_to_node(node* to_node);
    void change_parent(node* new_parent);
    MyPose return_pose(){return pose;};
    void adjust_cost(float delta_);
    node* return_parent(){return parent;};
    MyPose* return_pos_pointer(){return &pose;};
    void set_parent_id(int id){ parent_edge_id = id;};
    int get_parent_id(){return parent_edge_id;}
    void update_collision(bool coll);
    void set_pose(MyPose pose_){pose = pose_;};
    list<node*> return_children(){return children;};
    // TODO: cap angle
    void set_angle(float dtheta){pose.at(2) += dtheta;
    if(pose.at(2)>=2*M_PI){
        pose.at(2) -= 2*M_PI;
    }
    else if(pose.at(2)<=-2*M_PI){
        pose.at(2) += 2*M_PI;
    }};

    void set_abs_angle(float theta_new){
        pose.at(2) = theta_new;
        while (abs(pose.at(2)) > M_PI) {
            if (pose.at(2) > 0) {
                pose.at(2) -= 2 * M_PI;
            }
            else{
                pose.at(2) += 2 * M_PI;
            }
        };
    }

    void update_information_utility();
    int return_path_length(){return path_length;};
    vector<node*> return_path_nodes(){return path_nodes;};

    // Backpropagation
    void backpropagate_information();
//        void push_path_rewards(list<tuple<vector<int>, vector<float>>*> reward_list){path_reward.insert(path_reward.end(),
//                                                                                                        reward_list.begin(),
//                                                                                                        reward_list.end());};
    void push_path_rewards(list<tuple<vector<int>, vector<float>>*> reward_list){path_reward = reward_list;
                                                                                 path_reward.push_front(&phi_i);};
    list<tuple<vector<int>, vector<float>>*> return_path_reward_inds(){return path_reward;};
    void remove_path_reward_inds();
    node* return_most_informative_child(){return most_informative_child;};
    float calc_new_costs(node* potential_parent);
    float calc_new_rewards(node* potential_parent);
    float calc_new_utility(node* potential_parent);
    float return_reward(){return reward;};
    float return_utility(){return utility;};
    float compare_utility(node* potential_parent);
    bool feasibility_check(node* potential_parent);
    float calc_costs_between_nodes(node* potential_parent);

    // Q_s passcode set/get
    int return_pass(){return code;};
    void set_pass(int p){code = p;};
};


#endif //DYNAMICRRT_STAR_NODE_H
