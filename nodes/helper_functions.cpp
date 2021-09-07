#include "helper_functions.h"
// euclidean dist between p1 and p2
float euclidean_dist(MyPose pose1, MyPose pose2) {
    return sqrt((pose2.at(0) - pose1.at(0))*(pose2.at(0) - pose1.at(0) )+ (pose2.at(1) - pose1.at(1))*(pose2.at(1) - pose1.at(1)));
}
float euclidean_dist_squared(MyPose pose1, MyPose pose2) {
    return (pose2.at(0) - pose1.at(0))*(pose2.at(0) - pose1.at(0) )+ (pose2.at(1) - pose1.at(1))*(pose2.at(1) - pose1.at(1));
}
Point random_point(global_params params){
    float border = 0.2;
    float x = params.x_range.at(0) + border + (params.x_range.at(1) - params.x_range.at(0) - 2 * border)*rand()/RAND_MAX;
    float y = params.y_range.at(0) + border + (params.y_range.at(1) - params.y_range.at(0) - 2 * border)*rand()/RAND_MAX;
    array<float, 2> sampled_p = {x, y};
    return sampled_p;
}

Point sample_from_square(Point x_range, Point y_range, MyPose pose){
    float d = 2;
    float x = pose.at(0) + (2*d)*rand()/RAND_MAX - d;
    float y = pose.at(1) + (2*d)*rand()/RAND_MAX - d;
    array<float, 2> sampled_p = {min(max(x, x_range.at(0)), x_range.at(1)), min(max(y, y_range.at(0)), y_range.at(1))};
    return sampled_p;
}

MyPose steer(MyPose pose1, Point point1, float r, Range x_range, Range y_range){
    float dx = point1.at(0) - pose1.at(0);
    float dy = point1.at(1) - pose1.at(1);
    float factor = sqrt(dx * dx + dy * dy)/r;
    MyPose p = {min(max(pose1.at(0) + dx/factor, (float)(x_range.at(0) + 0.01)), (float)(x_range.at(1) - 0.01)),
                min(max(pose1.at(1) + dy/factor, (float)(y_range.at(0) + 0.01)), (float)(y_range.at(1) - 0.01)), 0};
    return p;
}

bool ccw(Point A, Point B, Point C) {
    return (C.at(1) - A.at(1)) * (B.at(0) - A.at(0)) > (B.at(1) - A.at(1)) * (C.at(0) - A.at(0));
}

// Return true if line segments AB and CD intersect
bool intersect(Point A, Point B, Point C, Point D) {
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D);
}

list<array<int, 2>> intersected_cells(Point p1_, Point p2_, global_params params){
    list<array<int, 2>> cells;
    float x1 = p1_.at(0);
    float y1 = p1_.at(1);
    float x2 = p2_.at(0);
    float y2 = p2_.at(1);
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
    int id_x_min = max(min((int)floor((x1 - params.x_range.at(0)) / params.cell_size), params.n_x - 1), 0);
    int id_y_min = max(min((int)floor((y1 - params.y_range.at(0)) / params.cell_size), params.n_y - 1), 0);
    int id_x_max = max(min((int)floor((x2 - params.x_range.at(0)) / params.cell_size), params.n_x - 1), 0);
    int id_y_max = max(min((int)floor((y2 - params.y_range.at(0)) / params.cell_size), params.n_y - 1), 0);
    for(int i=id_x_min; i<=id_x_max; i++){
        for(int j=id_y_min; j<=id_y_max; j++){
            array<int, 2> c = {i, j};
            cells.push_back(c);
        }
    }
    return cells;
}

Point limit_point(Point p1, Range x, Range y){
    float xx = min(max(p1.at(0),x.at(0)), x.at(1));
    float yy = min(max(p1.at(1),y.at(0)), y.at(1));
    return((Point){xx,yy});
}

list<vector<array<float, 2>>> rectangle(Point p1, Point p2, Point p3, Point p4){
    list<vector<array<float,2>>> line_list;
    vector<array<float,2>> line;
    line.push_back(p1);
    line.push_back(p2);
    line_list.push_back(line);
    line.clear();
    line.push_back(p2);
    line.push_back(p3);
    line_list.push_back(line);
    line.clear();
    line.push_back(p3);
    line.push_back(p4);
    line_list.push_back(line);
    line.clear();
    line.push_back(p4);
    line.push_back(p1);
    line_list.push_back(line);
    return line_list;
}


// GMRF
SpMat initialize_prec_mat(int nx, int ny) {
    float kappa = 1e-4;
    SpMat PrecisionMatrix;
    int n_nodes = nx * ny;
    PrecisionMatrix.resize(n_nodes + 1, n_nodes + 1); // single latent variable
    PrecisionMatrix.setIdentity();
    PrecisionMatrix *= (4 + kappa * kappa);
    PrecisionMatrix.coeffRef(n_nodes, n_nodes) = 0;
    SpMat SparseIdentity_NX;
    SparseIdentity_NX.resize(n_nodes + 1, n_nodes + 1);
    SparseIdentity_NX.setZero();
    int y_i = nx;
    for (int x_i = 0; x_i < n_nodes; x_i++){
        SparseIdentity_NX.coeffRef(x_i, y_i) = -1;
        SparseIdentity_NX.coeffRef(y_i, x_i) = -1;
        y_i++;
        if (y_i == n_nodes){
            break;
        }
    }
    y_i = 1;
    for (int x_i = 0; x_i < n_nodes; x_i++){
        SparseIdentity_NX.coeffRef(x_i, y_i) = -1;
        SparseIdentity_NX.coeffRef(y_i, x_i) = -1;
        y_i++;
        if (y_i == n_nodes){
            break;
        }
    }

    PrecisionMatrix += SparseIdentity_NX;
    for (int j = 0; j < n_nodes; j++){
        if (j < nx || j >= n_nodes - nx){
            PrecisionMatrix.coeffRef(j, j) = 3 + kappa * kappa;
        }
        if (j % nx == 0){
            if (j >= nx){
                PrecisionMatrix.coeffRef(j, j - 1) = 0;
                PrecisionMatrix.coeffRef(j, j) = 3 + kappa * kappa;
                PrecisionMatrix.coeffRef(j-1, j) = 0;
                PrecisionMatrix.coeffRef(j-1, j-1) = 3 + kappa * kappa;
            }
        }
    }
    // corner values
    PrecisionMatrix.coeffRef(0, 0) = 2 + kappa * kappa;
    PrecisionMatrix.coeffRef(n_nodes - 1, n_nodes - 1) = 2 + kappa * kappa;
    PrecisionMatrix.coeffRef(nx - 1, nx - 1) = 2 + kappa * kappa;
    PrecisionMatrix.coeffRef(n_nodes - nx, n_nodes - nx) = 2 + kappa * kappa;
    float T = 0.001;
    for (int x_ind = 0; x_ind < n_nodes; x_ind++){
        PrecisionMatrix.coeffRef(x_ind, n_nodes) = - PrecisionMatrix.row(x_ind).sum();
    }
    for (int y_ind = 0; y_ind < n_nodes; y_ind++) {
        PrecisionMatrix.coeffRef(n_nodes, y_ind) = - PrecisionMatrix.col(y_ind).sum();
    }
    PrecisionMatrix.coeffRef(n_nodes, n_nodes) = - PrecisionMatrix.row(n_nodes).sum() + T;
    return PrecisionMatrix;
}

std::tuple<vector<int>, vector<float>> initializePHI(MyPose location, int n_x, int n_y, double cell_size, Range x_, Range y_){
    int x_ind = (int)floor((location.at(0) - x_.at(0)) / cell_size) + 1;
    int y_ind = (int)floor((location.at(1) - y_.at(0)) / cell_size) + 1;

    int cell = (y_ind) * (n_x) + (x_ind);
    float x_center = (float)(x_ind - 1) * cell_size;
    float y_center = (float)(y_ind - 1) * cell_size;
    float xRel = location.at(0) - x_center;
    float yRel = location.at(1) - y_center;

    vector<int> neighbour_nodes;
    vector<float> weights;
    neighbour_nodes.push_back(cell);

    // get index of closest node
    if (location.at(0) >= x_center){
        neighbour_nodes.push_back(cell + 1);
    } else {
        neighbour_nodes.push_back(cell - 1);
    }
    if (location.at(1) >= y_center){
        neighbour_nodes.push_back(neighbour_nodes.at(1) + n_x);
        neighbour_nodes.push_back(neighbour_nodes.at(0) + n_x);
    } else {
        neighbour_nodes.push_back(neighbour_nodes.at(1) - n_x);
        neighbour_nodes.push_back(neighbour_nodes.at(0) - n_x);
    }
    // sort neighbouring nodes
    sort (neighbour_nodes.begin(), neighbour_nodes.end());

    weights.push_back(((xRel - cell_size) * (yRel - cell_size)) / (pow(cell_size, 2)));
    weights.push_back(- ((xRel) * (yRel - cell_size)) / (pow(cell_size, 2)));
    weights.push_back(- ((xRel - cell_size) * (yRel)) / (pow(cell_size, 2)));
    weights.push_back(((xRel) * (yRel)) / (pow(cell_size, 2)));

    return std::make_tuple(neighbour_nodes, weights);
}

float atan2_approximation1(float y, float x)
{
//http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
//Volkan SALMA

    const float ONEQTR_PI = M_PI / 4.0;
    const float THRQTR_PI = 3.0 * M_PI / 4.0;
    float r, angle;
    float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
    if ( x < 0.0f )
    {
        r = (x + abs_y) / (abs_y - x);
        angle = THRQTR_PI;
    }
    else
    {
        r = (x - abs_y) / (x + abs_y);
        angle = ONEQTR_PI;
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    if ( y < 0.0f )
        return( -angle );     // negate if in quad III or IV
    else
        return( angle );
}

list<array<int, 2>> cells_in_sensor_range(MyPose current_pose, global_params parameters){
    Point p0 = {current_pose.at(0), current_pose.at(1)};
    Point p1 = {current_pose.at(0) + (cos(current_pose.at(2) + parameters.sensor_angle) * parameters.sensor_range),
                current_pose.at(1) + (sin(current_pose.at(2) + parameters.sensor_angle) * parameters.sensor_range)};
    Point p2 = {current_pose.at(0) + (cos(current_pose.at(2) - parameters.sensor_angle) * parameters.sensor_range),
                current_pose.at(1) + (sin(current_pose.at(2) - parameters.sensor_angle) * parameters.sensor_range)};


    float x_min = min({p0.at(0), p1.at(0), p2.at(0)});
    float x_max = max({p0.at(0), p1.at(0), p2.at(0)});
    float y_min = min({p0.at(1), p1.at(1), p2.at(1)});
    float y_max = max({p0.at(1), p1.at(1), p2.at(1)});

    // get all cell in rectangle around sensor cone
    int id_x_min = max(min((int)floor((x_min - parameters.x_range.at(0)) / parameters.cell_size), parameters.n_x - 1), 0);
    int id_y_min = max(min((int)floor((y_min - parameters.y_range.at(0)) / parameters.cell_size), parameters.n_y - 1), 0);
    int id_x_max = max(min((int)floor((x_max - parameters.x_range.at(0)) / parameters.cell_size), parameters.n_x - 1), 0);
    int id_y_max = max(min((int)floor((y_max - parameters.y_range.at(0)) / parameters.cell_size), parameters.n_y - 1), 0);

    // normal vectors of lines
    array<float, 2> n1 = {sin(atan2(p1.at(1) - p0.at(1), p1.at(0) - p0.at(0))),
                          -cos(atan2(p1.at(1) - p0.at(1), p1.at(0) - p0.at(0)))};
    array<float, 2> n2 = {-sin(atan2(p2.at(1) - p0.at(1), p2.at(0) - p0.at(0))),
                          cos(atan2(p2.at(1) - p0.at(1), p2.at(0) - p0.at(0)))};

    list<array<int, 2>> cells;
    for (int id_x = id_x_min; id_x <= id_x_max; id_x++){
        float x = (float)(id_x) * parameters.cell_size + parameters.cell_size / 2;
        float x_rel = x - current_pose.at(0);
        for (int id_y = id_y_min; id_y <= id_y_max; id_y++) {
            float y = (float)(id_y) * parameters.cell_size + parameters.cell_size / 2;
            float y_rel = y - current_pose.at(1);
            // check if grid cell is in sensor range
            if (pow(x - current_pose.at(0), 2) + pow(y - current_pose.at(1), 2) <= pow(parameters.sensor_range, 2)){
                if ((x_rel * n1.at(0) + y_rel * n1.at(1)) >= 0 && (x_rel * n2.at(0) + y_rel * n2.at(1)) >= 0){
                    cells.push_back((array<int, 2>){id_x, id_y});
                }
            }
        }
    }
    return cells;
}

bool is_point_on_line_segment(float line_x1, float line_y1, float line_x2, float line_y2, float point_x, float point_y){
    bool intersection = false;
    if (line_x1 == line_x2 || line_y1 == line_y2) {
        if (line_x1 == line_x2) {
            if (line_x1 == point_x) {
                if (point_y >= min(line_y1, line_y2) && point_y <= max(line_y1, line_y2)) {
                    return true;
                }
            }
        }
        if (line_y1 == line_y2) {
            if (line_y1 == point_y) {
                if (point_x >= min(line_x1, line_x2) && point_x <= max(line_x1, line_x2)) {
                    return true;
                }
            }
        }
    }
    else{
        if ((point_x - line_x1) / (line_x2 - line_x1) == (point_y - line_y1) / (line_y2 - line_y1)){
            float alpha = (point_x - line_x1) / (line_x2 - line_x1);
            if (alpha >= 0 && alpha <= 1) {
                return true;
            }
        }
    }
    return false;
}

bool is_point_on_line(float line_x1, float line_y1, float line_x2, float line_y2, float point_x, float point_y){
    bool intersection = false;
    if (line_x1 == line_x2 || line_y1 == line_y2) {
        if (line_x1 == line_x2) {
            if (line_x1 == point_x) {
                return true;
            }
        }
        if (line_y1 == line_y2) {
            if (line_y1 == point_y) {
                return true;
            }
        }
    }
    else{
        if ((point_x - line_x1) / (line_x2 - line_x1) == (point_y - line_y1) / (line_y2 - line_y1)){
            return true;
        }
    }
    return false;
}

float calc_costs_between_poses(MyPose parent, MyPose child, float weight_steering, float weight_dist){
    float diff = abs(parent.at(2) - atan2(child.at(1)-parent.at(1), child.at(0) - parent.at(0)));
    if(diff>M_PI){
        while(diff>M_PI) {
            diff = abs(diff -  2*M_PI);
        }
    }

    return weight_dist * euclidean_dist(parent, child) + weight_steering * pow(diff, 2);
}
