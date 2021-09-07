#include "node.h"
#include<iostream>
#include <utility>

node::node(global_params parameters_, MyPose pose_, GMRF* gmrf_belief_, vector<int> shape_inds_,
           vector<float> shape_weights_, tuple<vector<int>, vector<float>> phi_, node** best_node, node* parent_){
    parameters = parameters_;

    // weights
    weight_steering = parameters.weight_steering;
    weight_distance = parameters.weight_distance;
    weight_info = parameters.weight_information;
    discount = parameters.discount;

    obstacle_discount = 0;
    border_discount = 0;
    close_to_border = false;

    parent = parent_;
    gmrf_belief = gmrf_belief_;

    // gmrf inds and weights
    shape_inds = std::move(shape_inds_);
    shape_weights = std::move(shape_weights_);

    phi_i = phi_;
    path_info_utility = 0;
    max_utility_node = best_node;
    utility = 0;
    reward = 0;
    path_reward.push_front(&phi_i);

    pose = pose_;
    if(parent == nullptr) {
        has_parent = false;
    }
    else{
        has_parent = true;
        parent->add_child(this);
        pose.at(2) = atan2_approximation1(pose.at(1) - parent->return_pose().at(1), pose.at(0) - parent->return_pose().at(0));
    }

    update_information_utility();

    if (has_parent){
        path_length = parent->return_path_length() + 1;
        path_nodes = parent->return_path_nodes();
    }

    cost = 0;
    calc_costs();
    if (has_parent){
        utility = calc_new_utility(parent);
    }

    path_nodes.push_back(this);
}

void node::calc_costs() {
    if(has_parent) {
        // agnle difference between node and parent
        float angle_difference = abs(parent->return_pose().at(2) - pose.at(2));

        // change angle to range -pi to pi
        if(angle_difference > M_PI){
            while(angle_difference > M_PI) {
                angle_difference = abs(angle_difference -  2*M_PI);
            }
        }
        reward = calc_new_rewards(parent);
        utility = calc_new_utility(parent);
        // costs
        cost_steering = pow(angle_difference, 2) * weight_steering;
        path_cost_steering = parent->return_path_cost_steering() + cost_steering;

        path_info_utility = reward / cost;
        if (has_child){
            border_discount = 0;
        }


        // OLD cost function
        cost_to_parent = weight_distance * euclidean_dist(parent->return_pose(), pose);
        cost = calc_new_costs(parent);
        if (*max_utility_node != nullptr && (path_info_utility) > (*max_utility_node)->return_path_info_utility() && path_length > 10){
            (*max_utility_node) = this;
        }
    }
    else{
        cost = 0;
    }
}

float node::calc_costs_to_node(node *to_node) {
    return abs(parent->return_pose().at(2) - pose.at(2))*weight_steering + euclidean_dist(pose, to_node->return_pose());
}

void node::change_parent(node *new_parent) {
    if (has_parent){
        parent->remove_child(this);
        if(parent->children.empty()){
            parent->has_child = false;
        }
    }

    // old parent
    parent->remove_path_reward_inds();

    collision_penalty = 0;
    has_parent = true;
    parent = new_parent;

    path_nodes = parent->return_path_nodes();
    path_nodes.push_back(this);
    path_length = parent->return_path_length() + 1;
    dist_travelled = parent->return_dist_travelled() + euclidean_dist(parent->return_pose(), pose);
    reward = calc_new_rewards(parent);

    pose.at(2) = atan2_approximation1(pose.at(1) - parent->return_pose().at(1), pose.at(0) - parent->return_pose().at(0));

    /*if (close_to_border && !has_child){
        // check if heading lies in penalized sector (dot product)
        float hx = cos(pose.at(2));
        float hy = sin(pose.at(2));
        if ((hx * border_penalization_sector.at(0).at(0) + hy * border_penalization_sector.at(0).at(1)) > 0 &&
                (hx * border_penalization_sector.at(1).at(0) + hy * border_penalization_sector.at(1).at(1)) > 0){
            border_discount = 20;
        }
        else{
            border_discount = 0;
        }
    }
    if (has_child){
        border_discount = 0;
    }*/

    parent->add_child(this);
    calc_costs();
    propagate_costs(true, true, true);
//    backpropagate_information();
}

void node::add_child(node *child_node) {
    has_child = true;
    children.push_back(child_node);
}

void node::remove_child(node *child_node) {
    children.remove(child_node);
    if(children.empty()){
        has_child = false;
    }
}

void node::propagate_costs(bool update_costs, bool cost_steer, bool first) {
    if (update_costs) {
        if (cost_steer) {
            float angle_difference = abs(parent->return_pose().at(2) - pose.at(2));
            if (angle_difference > M_PI) {
                while (angle_difference > M_PI) {
                    angle_difference = abs(angle_difference - 2 * M_PI);
                }
            }

            cost_steering = pow(angle_difference, 2) * weight_steering;
            path_cost_steering = parent->return_path_cost_steering() + cost_steering;
        }
        if (has_parent) {
            path_nodes = parent->return_path_nodes();
            path_nodes.push_back(this);
            path_length = parent->return_path_length() + 1;
            cost = calc_new_costs(parent);
            reward = calc_new_rewards(parent);
            utility = calc_new_utility(parent);

            path_info_utility = reward / cost;
        }
        if (*max_utility_node != nullptr && (path_info_utility) > (*max_utility_node)->return_path_info_utility() && path_length > 10){
            (*max_utility_node) = this;
        }
    }
    if(has_child){
        for(node* child:children){
            if(first) {
                child->propagate_costs(true, true, false);
            }
            else{
                child->propagate_costs(true, false, false);
            }
        }
    }
}

void node::adjust_cost(float delta_) {
    obstacle_discount = 10;
    if (has_parent){
        parent->obstacle_discount = 10;
    }
    cost = cost + delta_;
    utility -= 10 * delta_;
    collision_penalty = delta_;
    propagate_costs(false);
}

void node::update_information_utility() {
    auto covariance = *(gmrf_belief->return_covariance());
    auto cov = (float)(shape_weights.at(0) * (covariance)[shape_inds.at(0)] +
                        shape_weights.at(1) * (covariance)[shape_inds.at(1)] +
                        shape_weights.at(2) * (covariance)[shape_inds.at(2)] +
                        shape_weights.at(3) * (covariance)[shape_inds.at(3)]);


    double mean = 0;
    if (parameters.weight_mean > 0){
        auto mean_ = *(gmrf_belief->return_mean());
        mean = (float)(shape_weights.at(0) * (mean_)[shape_inds.at(0)] +
                           shape_weights.at(1) * (mean_)[shape_inds.at(1)] +
                           shape_weights.at(2) * (mean_)[shape_inds.at(2)] +
                           shape_weights.at(3) * (mean_)[shape_inds.at(3)]);
        mean /= mean_.maxCoeff();
    }
    information_utility = parameters.weight_cov * (cov / gmrf_belief->return_mean_field_cov()) +
                            parameters.weight_mean * mean * cov;

//    double max_variance = covariance.head(covariance.size() - 2).maxCoeff();

//    information_utility = parameters.weight_information * (cov / gmrf_belief->return_mean_field_cov());
//    information_utility = weight_info * (cov / max_variance);

    if (has_parent) {
        reward = calc_new_rewards(parent);
        utility = calc_new_utility(parent);
        path_info_utility = reward / cost;
    }
    else{
        path_info_utility = 0;
    }

    if (*max_utility_node != nullptr && (path_info_utility) > (*max_utility_node)->return_path_info_utility() && path_length > 10){
        (*max_utility_node) = this;
    }

}

float node::calc_new_costs(node* potential_parent) {
    float c_new = potential_parent->return_costs() + calc_costs_between_nodes(potential_parent)
            + obstacle_discount + border_discount;
    return c_new;
}

void node::remove_path_reward_inds() {
    path_reward.clear();
    path_reward.push_front(&phi_i);
}


float node::calc_new_rewards(node *potential_parent) {
//    return (float)(potential_parent->return_reward() + parameters.weight_information * information_utility * (euclidean_dist(potential_parent->return_pose(), this->return_pose()) / 0.3));
//
    return (float)(potential_parent->return_reward() + parameters.weight_information * pow(discount, path_length) * ((potential_parent->return_info_utility() + information_utility) / 2) * (euclidean_dist(potential_parent->return_pose(), this->return_pose()) / parameters.nominal_step));
}

float node::calc_new_utility(node *potential_parent) {
    float util = (calc_new_rewards(potential_parent)) / (pow((calc_new_costs(potential_parent)), 1.4)) - obstacle_discount;
    if (euclidean_dist(potential_parent->return_pose(), this->return_pose()) > 0.4){
        util -= 100;
    }
    return util;
}

float node::calc_costs_between_nodes(node* potential_parent){
    float dist = euclidean_dist(potential_parent->return_pose(), this->return_pose()) / parameters.nominal_step;
    float diff = abs(potential_parent->return_pose().at(2) - atan2_approximation1(this->return_pose().at(1)-potential_parent->return_pose().at(1), this->return_pose().at(0) - potential_parent->return_pose().at(0)));
    if(diff>M_PI){
        while(diff>M_PI) {
            diff = abs(diff -  2*M_PI);
        }
    }
//    return (float)(weight_distance * dist + parameters.weight_steering * pow(diff / dist, 2));
    return (float)(weight_distance * dist + parameters.weight_steering * pow(diff, 2));
}

