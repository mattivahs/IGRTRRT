/* TODO:
 - DONE: Rewire from root
 - ROOT-NODE, move if many other nodes around, create new if not
 - intelligent sampling (insbesondere wenn expand(advance = FALSE)?
 - closest goal -> remove insert
*/
#include<iostream>
#include "rrt_star.h"
using namespace __gnu_cxx;

rrt_star::rrt_star(global_params params, MyPose init_pose_, list<obstacle*> obstacles) {
    parameters = params;

    x_range = parameters.x_range;
    y_range = parameters.y_range;
    cell_size = parameters.cell_size;
    current_pose = init_pose_;

    // random sead
    srand(time(NULL));

    // GMRF
    gmrf_belief = new GMRF(parameters);
    Point init_point = {current_pose.at(0), current_pose.at(1)};


    auto sizes = gmrf_belief->return_size();
    nx_gmrf = std::get<0>(sizes);
    ny_gmrf = std::get<1>(sizes);
    cs_gmrf = std::get<2>(sizes);


    // rewiring radius
    r = (float)0.8 * cell_size;

    // RRT grid size
    n_x = ceil((x_range.at(1) - x_range.at(0)) / cell_size);
    n_y = ceil((y_range.at(1) - y_range.at(0)) / cell_size);

    // initialize obstacle grid
    for(int i = 0; i<n_x; i++){
        vector<list<node*>> b;
        vector<list<obstacle*>> bb;
        Grid.push_back(b);
        ObstclGrid.push_back(bb);
        RealObstacles.push_back(bb);
        for(int j = 0; j<n_y; j++){
            list<node*> a;
            list<obstacle*> aa;
            Grid.at(i).push_back(a);
            ObstclGrid.at(i).push_back(aa);
            RealObstacles.at(i).push_back(aa);
        }
    }

    // add intial obstacles
    for(obstacle* obstcl:obstacles){
        add_obstcl(obstcl);
    }

    rewire_time = parameters.rewire_time;

    // get GMRF indices and weights for intial node
    auto t = initializePHI(current_pose, nx_gmrf, ny_gmrf, cs_gmrf, x_range, y_range);

    // dummy node
    dummy_node = new node(parameters, current_pose, gmrf_belief, std::get<0>(t), std::get<1>(t), t, &max_utility_node);
    dummy_node->adjust_cost(2000);
    max_utility_node = dummy_node;

    // initialize tree
    node *init_node = new node(parameters, current_pose, gmrf_belief, std::get<0>(t), std::get<1>(t), t, &max_utility_node);
    root_node = init_node;
    add_to_tree(init_node);

    closest_to_goal = init_node;
    goal_node = init_node;
}

void rrt_star::expand(bool advance) {

    // if new nodes should be added to the tree
    if(advance) {

        // sample random point in area
        Point sampled_p = random_point(parameters);

        // find nearest node
        node *nearest_node = find_nearest(sampled_p);

        // find new pose
        MyPose new_pose = steer(nearest_node->return_pose(), sampled_p, r, parameters.x_range, parameters.y_range);

        if (!checkcollision(new_pose, nearest_node->return_pose())) {

            //check if new node is valid
            float c = calc_costs_between_poses(nearest_node->return_pose(), new_pose, parameters.weight_steering, parameters.weight_distance);
            if (c < 100) {
                // intialize GMRF mapping vector since position does not change
                std::tuple<vector<int>, vector<float>> t = initializePHI(new_pose, nx_gmrf, ny_gmrf, cs_gmrf, x_range, y_range);

                // generate new node
                node *node_to_add = new node(parameters, new_pose, gmrf_belief, std::get<0>(t), std::get<1>(t), t, &max_utility_node, nearest_node);


                // push node to rewiring queue
                Qr_prior.push(node_to_add);

                // add node to tree
                add_to_tree(node_to_add);


                // rewire
                rewire();

                // rewire from root
                rewire_f_r();
            }
        }
    }
    // no new nodes; just rewiring
    else{
        // rewire from root
        rewire_f_r();

        // random rewiring
        int id = (int)(rand() % (all_nodes.size()-1));
        node* to_rewire = all_nodes[id];
        Qr_prior.push(to_rewire);
        rewire();
    }

}

void rrt_star::add_to_tree(node *new_node) {
    VectP pos = {new_node->return_pose().at(0), new_node->return_pose().at(1)};
    all_nodes.push_back(new_node);
    all_points.push_back(pos);

    // sort new node into grid (for faster execution of find nearest)
    sort_in(new_node);

    // check if one of the near nodes is better in terms of utility
    vector<node*> near_n = near_nodes(new_node);

    for(node* near_node:near_n) {
        if (near_node != new_node) {
            if (!checkcollision(near_node->return_pose(), new_node->return_pose())) {
                // new utility with potential parent
                float u_new = new_node->calc_new_utility(near_node);

                // if higher utility -> change parent
                if (u_new > new_node->return_utility()) {
                    change_node_parent(new_node, near_node);
                }
            }
        }
    }
}

void rrt_star::rewire() {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = start_time;

    node* current_n;

    // rewire while rewiring queue is not empty or rewiring time is not exceeded
    while(!(Qr.empty() && Qr_prior.empty()) && (current_time-start_time) / std::chrono::milliseconds(1) < rewire_time) {

        if (!Qr_prior.empty()){
            current_n = Qr_prior.top();
            Qr_prior.pop();
        }
        else {
            current_n = Qr.front();
            Qr.pop();
        }
        // iterate nodes calls rewiring step
        iterate_nodes(current_n, false);
        current_time = std::chrono::high_resolution_clock::now();
    }

}

node *rrt_star::find_nearest(Point x_rand) {

    // function to find nearest node given a sampled point
    bool not_found = true;

    // additional grid cells to check
    int addx = 0;
    list<node*> near_nodes;
    node* nearest_node;

    // get grid index
    int id_x = floor((x_rand.at(0)-x_range.at(0))/cell_size);
    int id_y = floor((x_rand.at(1)-y_range.at(0))/cell_size);

    float d_min = 2000;
    float d;
    MyPose c = {x_rand.at(0), x_rand.at(1), 0};

    // first check only in closest grid cell and the extend until node is found
    while(not_found){
        // iterate over x_cells: start with current x_cell
        for(int xx = id_x - addx; xx <= (id_x + addx); xx++){
            // iterate over y_cells
            for(int yy = id_y - addx; yy <= (id_y + addx); yy++) {
                if((abs(xx - id_x) == addx || abs(yy-id_y)==addx) && (xx < n_x && yy < n_y && xx >= 0 && yy >=0)) {
                    if (!Grid.at(xx).at(yy).empty()) {
                        not_found = false;
                        for(node* aa:Grid.at(xx).at(yy)) {
                            // use euclidean dist squared for efficiency
                            d = euclidean_dist_squared(aa->return_pose(), c);
                            if(d < d_min){
                                d_min = d;
                                nearest_node = aa;
                            }
                        }
                    }
                }
            }
        }
    addx++;
    }

    // check next cells
    for(int xx = id_x-addx; xx<(id_x+addx); xx++){
        for(int yy = id_y-addx; yy<(id_y+addx); yy++) {
            if((abs(xx-id_x)==addx || abs(yy-id_y)==addx) && (xx < n_x && yy < n_y && xx >=0 && yy >=0)) {
                if (!Grid.at(xx).at(yy).empty()) {
                    for(node* aa:Grid.at(xx).at(yy)) {
                        d = euclidean_dist_squared(aa->return_pose(), c);
                        if(d < d_min){
                            d_min = d;
                            nearest_node = aa;
                        }
                    }
                }
            }
        }
    }

    return nearest_node;
}

void rrt_star::rewire_f_r() {
    // function to rewire root
    auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = start_time;

    if(Q_s.empty()) {
        Q_s.push(root_node);
        for(auto i = last_push.begin(); i != last_push.end(); i++){
            if(*i != root_node){
                Q_s.push(*i);

            }
        }
        last_push.clear();
    }
    node* current_n;
    vector<node*> near_ns;
    int iter_rewire_fr = 0;

    // queue not empty or time limit not exceeded
    while(!Q_s.empty() && ((current_time-start_time)/std::chrono::milliseconds(1) < rewire_time)){
        iter_rewire_fr++;
        current_n = Q_s.front();
        Q_s.pop();
        iterate_nodes(current_n);
        current_time = std::chrono::high_resolution_clock::now();
    }

}

bool rrt_star::checkcollision(MyPose pose1, MyPose pose2) {
    // function to check for collision given two poses
    float x1 = pose1.at(0);
    float y1 = pose1.at(1);
    float x2 = pose2.at(0);
    float y2 = pose2.at(1);
    Point p1 = {x1, y1};
    Point p2 = {x2, y2};

    // change order such that line starts left
    if(x1 > x2){
        float x1_b = x1;
        x1 = x2;
        x2 = x1_b;
    }
    if(y1> y2){
        float y1_b = y1;
        y1 = y2;
        y2 = y1_b;
    }

    int id_x_min = floor((x1 - x_range.at(0)) / cell_size);
    int id_y_min = floor((y1 - y_range.at(0)) / cell_size);
    int id_x_max = floor((x2 - x_range.at(0)) / cell_size);
    int id_y_max = floor((y2 - y_range.at(0)) / cell_size);

    // call collision check for every obstacle in obstacle grid
    for(int i=max(0, id_x_min); i<=min(id_x_max, n_x-1); i++){
        for(int j=max(0,id_y_min); j<=min(id_y_max, n_y-1); j++){
            for(obstacle* obstcl:ObstclGrid.at(i).at(j)){
                if(obstcl->check_collision(p1, p2)){
                    return true;
                }
            }
        }
    }
    return false;
}
vector<node *> rrt_star::near_nodes(node *c_node) {
    // function that returns a list of near nodes given a center node

    vector<node*> return_nodes;
    float x = c_node->return_pose().at(0);
    float y = c_node->return_pose().at(1);

    // get grid index
    int id_x = floor((x - x_range.at(0)) / cell_size);
    int id_y = floor((y - y_range.at(0)) / cell_size);

    // return all nodes in the current cell
    return_nodes.insert(return_nodes.end(), Grid.at(id_x).at(id_y).begin(), Grid.at(id_x).at(id_y).end());

    // relative x position in cell
    float diff = x - (cell_size * (float)id_x + cell_size / 2);

    int sgnx=0;
    int sgny=0;
    bool dont_add = false;

    // also return nodes of neighbouring cells, if center node is "at border" of cell
    if(abs(diff) > cell_size / 2 - r){
        if(diff>0){
            sgnx=1;
        }
        else{
            sgnx=-1;
        }
        if(id_x + sgnx < n_x && id_x + sgnx >= 0) {
            return_nodes.reserve(return_nodes.size() + Grid.at(id_x + sgnx).at(id_y).size());
            return_nodes.insert(return_nodes.end(), Grid.at(id_x + sgnx).at(id_y).begin(),
                                Grid.at(id_x + sgnx).at(id_y).end());
        }
        else{
            dont_add = true;
        }
    }

    diff = y - (cell_size * (float)id_y + cell_size / 2);
    if(abs(diff) > cell_size / 2 - r){
        if(diff>0){
            sgny=1;
        }
        else{
            sgny=-1;
        }
        if(id_y + sgny < n_y && id_y + sgny >= 0) {

            return_nodes.insert(return_nodes.end(), Grid.at(id_x).at(id_y + sgny).begin(),
                                Grid.at(id_x).at(id_y + sgny).end());

        }
        else{
            dont_add = true;
        }
    }
    if(sgnx != 0 && sgny!=0 && !dont_add){
        return_nodes.insert(return_nodes.end(), Grid.at(id_x+sgnx).at(id_y+sgny).begin(), Grid.at(id_x+sgnx).at(id_y+sgny).end());
    }
    return return_nodes;
}
void rrt_star::rewire_single_node(node* node_parent, node* node_child, bool root){
    // function that checks if parent should be rewired

    bool parent_changed = false;

    // old cost
    float u_old = node_child->return_utility();

    // new_cost with potential parent
    float u_new = node_child->calc_new_utility(node_parent);

    // if parent changed, push to rewiring queue
    if(u_new > u_old){
        if(!checkcollision(node_parent->return_pose(), node_child->return_pose())){
            parent_changed = change_node_parent(node_child, node_parent);
            if(!root && parent_changed){
                Qr.push(node_child);
            }
        }
    }

    // if rewire from root
    if(root) {
        if(node_child->return_pass() == global_pass) {
            // check if node to be rewired is already in queue
        }
        else{
            Q_s.push(node_child);
            node_child->set_pass(global_pass);
            //Qs_toBePushed.push_back(node_child);
        }
    }
}

void rrt_star::iterate_nodes(node *c_node, bool root) {
    // function to call rewiring step
    float x = c_node->return_pose().at(0);
    float y = c_node->return_pose().at(1);

    // get grid index
    int id_x = floor((x-x_range.at(0))/cell_size);
    int id_y = floor((y-y_range.at(0))/cell_size);

    // call rewiring procedure for near nodes
    for(node* near_node:Grid.at(id_x).at(id_y)){
        if (c_node != near_node) {
            rewire_single_node(c_node, near_node, root);
        }
    }

    // relative position in grid cell
    float diff = x - (cell_size * (float)id_x + cell_size / 2);

    int sgnx=0;
    int sgny=0;
    bool dont_add = false;

    // check which additional cells to check
    if(abs(diff) > cell_size/2 -r){
        if(diff>0){
            sgnx=1;
        }
        else{
            sgnx=-1;
        }
        if(id_x+sgnx < n_x && id_x+sgnx>=0) {
            for(node* near_node:Grid.at(id_x + sgnx).at(id_y)){
                if (c_node != near_node) {
                    rewire_single_node(c_node, near_node, root);
                }
            }
        }
        else{
            dont_add = true;
        }
    }

    // relative y position in grid cell
    diff = y - (cell_size * (float)id_y + cell_size / 2);

    if(abs(diff) > cell_size/2 - r){
        if(diff>0){
            sgny=1;
        }
        else{
            sgny=-1;
        }
        if(id_y+sgny < n_y && id_y+sgny>=0) {
            for(node* near_node:Grid.at(id_x).at(id_y + sgny)){
                if (c_node != near_node) {
                    rewire_single_node(c_node, near_node, root);
                }
            }
        }
        else{
            dont_add = true;
        }
    }

    // check diagonal cells
    if(sgnx != 0 && sgny!=0 && !dont_add){
        for(node* near_node:Grid.at(id_x + sgnx).at(id_y + sgny)){
            if (c_node != near_node) {
                rewire_single_node(c_node, near_node, root);
            }
        }
    }
}

bool rrt_star::closest_goal() {
    // goal diameter
    float goal_d = 0.2;

    // list of nodes close to goal
    list<node*> return_nodes;
    bool found = false;

    MyPose goal_pose = {goal_pos.at(0), goal_pos.at(1), 0};
    float x = goal_pos.at(0);
    float y = goal_pos.at(1);

    // get grid index
    int id_x = max(min((int)floor((x - x_range.at(0)) / cell_size), n_x - 1), 0);
    int id_y = max(min((int)floor((y - y_range.at(0)) / cell_size), n_y - 1), 0);

    // find node with max utility
    float u_min = 0;
    for(node* node_c:Grid.at(id_x).at(id_y)){
        if(euclidean_dist(node_c->return_pose(), goal_pose)<goal_d && !checkcollision(node_c->return_pose(), goal_pose) && node_c->return_path_info_utility() > u_min){
            u_min = node_c->return_path_info_utility();
            closest_to_goal = node_c;
            found = true;
        }
    }
    float diff = x-(cell_size*id_x+cell_size/2);
    int sgnx=0;
    int sgny=0;
    bool dont_add = false;
    if(abs(diff) > cell_size/2 -r){
        if(diff>0){
            sgnx=1;
        }
        else{
            sgnx=-1;
        }
        if(id_x+sgnx < n_x && id_x+sgnx>=0) {
            for(node* node_c:Grid.at(id_x + sgnx).at(id_y)){
                if(euclidean_dist(node_c->return_pose(), goal_pose)<goal_d &&
                    !checkcollision(node_c->return_pose(), goal_pose) &&
                        node_c->return_path_info_utility() > u_min){
                    u_min = node_c->return_path_info_utility();
                    closest_to_goal = node_c;
                    found = true;
                }
            }
        }
        else{
            dont_add = true;
        }
    }

    // check neighbouring cells
    diff = y - (cell_size * (float)id_y + cell_size/2);
    if(abs(diff) > cell_size / 2 - r){
        if(diff>0){
            sgny=1;
        }
        else{
            sgny=-1;
        }
        if(id_y+sgny < n_y && id_y+sgny>=0) {
            for(node* node_c:Grid.at(id_x).at(id_y+sgny)){
                if(euclidean_dist(node_c->return_pose(), goal_pose)<goal_d &&
                    !checkcollision(node_c->return_pose(), goal_pose) &&
                        node_c->return_path_info_utility() > u_min){
                    u_min = node_c->return_path_info_utility();
                    closest_to_goal = node_c;
                    found = true;
                }
            }

        }
        else{
            dont_add = true;
        }
    }

    if(sgnx != 0 && sgny!=0 && !dont_add){
        for(node* node_c:Grid.at(id_x+sgnx).at(id_y+sgny)){
            if(euclidean_dist(node_c->return_pose(), goal_pose)<goal_d &&
                !checkcollision(node_c->return_pose(), goal_pose) &&
                    node_c->return_path_info_utility() > u_min){
                u_min = node_c->return_path_info_utility();
                closest_to_goal = node_c;
                found = true;
            }
        }
    }
    utility = u_min;
    return found;
}

void rrt_star::sort_in(node *to_sort) {
    // sort new node into grid
    float x = to_sort->return_pose().at(0);
    float y = to_sort->return_pose().at(1);
    int id_x = max(min((int)floor((x-x_range.at(0))/cell_size), n_x), 0);
    int id_y = max(min((int)floor((y-y_range.at(0))/cell_size), n_y), 0);
    Grid.at(id_x).at(id_y).push_back(to_sort);
}

void rrt_star::add_obstcl(obstacle *obstcl) {
    // add obstacle to grid and return occupied cells
    list<array<int, 2>> occ_cells = obstcl->sort_in();
    for(array<int,2> occ_cell:occ_cells){
        ObstclGrid.at(occ_cell.at(0)).at(occ_cell.at(1)).push_back(obstcl);
    }
    list<array<int,2>> cells = obstcl->return_cells();
    list<array<int,2>> additional_cells;
    array<int, 2> lower_left = cells.front();
    array<int, 2> upper_right = cells.back();
    for (int i = lower_left.at(0) - 1; i <= upper_right.at(0) + 1; i++){
        if (i >=0 && i < n_x){
            for (int j = lower_left.at(1) - 1; j <= upper_right.at(1) + 1; j++) {
                if (j >=0 && j < n_y) {
                    additional_cells.push_back((array<int, 2>) {i, j});
                }
            }
        }
    }

    // check for collisions with new obstacle
    // check all nodes in rectangle arround obstacle -> can be improved
    for(array<int,2> i:additional_cells){
        for(node* node_to_be_checked:Grid.at(i.at(0)).at(i.at(1))){
            bool found_new = false;
            if(node_to_be_checked->has_parent) {
                // check collision
                if (obstcl->check_collision((Point) {node_to_be_checked->return_pose().at(0),
                                                     node_to_be_checked->return_pose().at(1)},
                                            (Point) {node_to_be_checked->return_parent()->return_pose().at(0),
                                                     node_to_be_checked->return_parent()->return_pose().at(1)})) {
                    vector<node *> near_n = near_nodes(node_to_be_checked);
                    // penalize colliding node
                    node_to_be_checked->adjust_cost(1000);
                    // check near nodes for better parent
                    for (node *near_node:near_n) {
                        if (near_node != node_to_be_checked) {
                            if (!checkcollision(near_node->return_pose(), node_to_be_checked->return_pose())) {
                                // new utility
                                float u_new = node_to_be_checked->calc_new_utility(near_node);

                                if (u_new > node_to_be_checked->return_utility()) {
                                    found_new = true;
                                    change_node_parent(node_to_be_checked, near_node);
                                }
                            }
                        }
                    }

                }
            }
            if (!found_new){
                Qr_prior.push(node_to_be_checked);
            }
        }
    }
}

bool rrt_star::change_node_parent(node *child_node, node *parent_node) {
    // function to change parent of node

    // return all nodes on parent path
    vector<node*> path_nodes = parent_node->return_path_nodes();

    // check if child node is on parent path to prevent cycles
    if (std::find(path_nodes.begin(), path_nodes.end(), child_node) != path_nodes.end()) {
        return false;
    }
    else {
        child_node->change_parent(parent_node);
        return true;
    }
}

bool rrt_star::update_goal(Point goal_) {
    goal_pos = goal_;
    return closest_goal();
}

void rrt_star::generate_path() {
    // generat path to goal point
    node* current_node = closest_to_goal;
//    node* current_node = goal_node;

    // goal path
    path.clear();
    while(current_node->has_parent){
        path.push_back(current_node->return_pose());
        current_node = current_node->return_parent();
    }
    // reverse goal path s.t. first node is the closest to the robot
    std::reverse(path.begin(), path.end());
}

void rrt_star::update_information_utility_of_near_nodes() {
    // update information utility of nodes after field belief is updated
    int id_x = floor((root_node->return_pose().at(0) - x_range.at(0)) / cell_size);
    int id_y = floor((root_node->return_pose().at(1) - y_range.at(0)) / cell_size);
    for(int xx = max(id_x-100, 0); xx <= min((id_x+100), n_x-1); xx++){
        for(int yy = max(id_y - 100, 0); yy <= min(id_y+100, n_y - 1); yy++) {
            for (node *node_c:Grid.at(xx).at(yy)) {
                node_c->update_information_utility();
            }
        }
    }
}

void rrt_star::update_pose(MyPose new_pose_) {
    // motion model: particle model with PD control
    current_pose = new_pose_;

    // angle difference
    while (abs(current_pose.at(2)) > M_PI) {
        if (current_pose.at(2) > 0) {
            current_pose.at(2) -= 2 * M_PI;
        }
        else{
            current_pose.at(2) += 2 * M_PI;
        }
    }

    set_root_node(current_pose);
}

void rrt_star::set_root_node(MyPose new_pose) {
    // for debug
    if((true || !checkcollision(new_pose, root_node->return_pose())) && (x_range.at(0) < new_pose.at(0) && new_pose.at(0) < x_range.at(1))
       && y_range.at(0) < new_pose.at(1) && new_pose.at(1) < y_range.at(1)) {
        root_node->set_pose(new_pose);
        Point root_pt = {new_pose.at(0), new_pose.at(1)};

        for (node *child:root_node->return_children()) {
            change_node_parent(child, root_node);
        }
        Qr_prior.push(root_node);
        queue<node *> empty;
        Q_s.swap(empty);
        global_pass = rand();
        Qs_toBePushed.clear();
    }
    root_node->set_abs_angle(new_pose.at(2));
    root_node->propagate_costs(true, false, true);
    Qr_prior.push(root_node);
}

