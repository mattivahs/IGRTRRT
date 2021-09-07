#include "helper_functions.h"

#ifndef DYNAMICRRT_STAR_OBSTACLE_H
#define DYNAMICRRT_STAR_OBSTACLE_H


class obstacle {
public:
    virtual tuple<list<array<int, 2>>, list<array<int, 2>>> update_obstacle() = 0;
    virtual bool check_collision(Point p1, Point p2) = 0;
    virtual list<array<int, 2>> sort_in() = 0;
    virtual tuple<Point, Point> return_start_and_end_point() = 0;
    list<array<int, 2>> return_cells(){return occ_cells;};
    float cell_size;
    global_params parameters;
protected:
    Range x_range;
    Range y_range;
    list<array<int, 2>> occ_cells;
    bool static_obstacle;
};

class line_obstacle: public obstacle {
private:
    Point p1;
    Point p2;
public:
    line_obstacle(global_params params, Point p1_, Point p2_);
    tuple<list<array<int, 2>>, list<array<int, 2>>> update_obstacle();
    bool check_collision(Point p1, Point p2);
    list<array<int, 2>> sort_in();
    tuple<Point, Point> return_start_and_end_point(){return make_tuple(p1, p2);};
};

class circle_obstacle: public obstacle {
private:
    Point p_center;
    float r;
public:
    circle_obstacle(Point p_center_, float r_, Range x_range_, Range y_range_, float cell_size_);
    tuple<list<array<int, 2>>, list<array<int, 2>>> update_obstacle();
    bool check_collision(Point p1, Point p2);
    tuple<Point, Point> return_start_and_end_point(){return make_tuple((Point){0, 0}, (Point){0, 0});};
    list<array<int, 2>> sort_in();
};


#endif //DYNAMICRRT_STAR_OBSTACLE_H
