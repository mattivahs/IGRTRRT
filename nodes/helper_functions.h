#ifndef DYNAMICRRT_STAR_HELPER_FUNCTIONS_H
#define DYNAMICRRT_STAR_HELPER_FUNCTIONS_H
#include <array>
#include <math.h>
#include <queue>
#include <list>
#include <unordered_set>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
//#include "Eigen/Sparse"
//#include "Eigen/Dense"
#include <iostream>

using namespace std;
typedef array<float, 3> MyPose;
typedef vector<float> VectP;
typedef vector<VectP> VectAllP;
typedef array<float, 2> Point;
typedef array<int, 2> V_Point;
typedef array<float, 2> Range;
typedef array<int, 2> V_Range;
typedef Eigen::SparseMatrix<double> SpMat;

struct global_params{
    Range x_range;
    Range y_range;
    float vel;
    float dt ;
    float c_mean;
    float weight_steering;
    float weight_distance;
    float weight_information;
    float weight_cov;
    float weight_mean;
    float discount;
    float cell_size;
    float rewire_time;
    int n_x;
    int n_y;
    int gmrf_nx;
    int gmrf_ny;
    float meas_noise;
    float sensor_angle;
    float sensor_range;
    bool mapping;
    float nominal_step;
    bool visualize_rviz;
};

float euclidean_dist(MyPose pose1, MyPose pose2);
float euclidean_dist_squared(MyPose pose1, MyPose pose2);
// random point in range
Point random_point(global_params params);
Point sample_from_square(Point x_range, Point y_range, MyPose pose);
Point limit_point(Point p1, Range x, Range y);
MyPose steer(MyPose pose1, Point point1, float r, Range x_range, Range y_range);
bool ccw(Point A, Point B, Point C);
bool intersect(Point A, Point B, Point C, Point D);
list<array<int, 2>> intersected_cells(Point p1, Point p2, global_params params);
list<vector<array<float, 2>>> rectangle(Point p1, Point p2, Point p3, Point p4);
float calc_costs_between_poses(MyPose parent, MyPose child, float weight_steering, float weight_dist);

// GMRF functions
SpMat initialize_prec_mat(int nx, int ny);
std::tuple<vector<int>, vector<float>> initializePHI(MyPose location, int n_x, int n_y, double cell_size, Range x_, Range y_);
float atan2_approximation1(float y, float x);

list<array<int, 2>> cells_in_sensor_range(MyPose current_pose, global_params parameters);
bool is_point_on_line_segment(float line_x1, float line_y1, float line_x2, float line_y2, float point_x, float point_y);
bool is_point_on_line(float line_x1, float line_y1, float line_x2, float line_y2, float point_x, float point_y);
#endif //DYNAMICRRT_STAR_HELPER_FUNCTIONS_H
