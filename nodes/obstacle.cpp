#include "obstacle.h"
#include <tuple>
#include "helper_functions.h"
#include <iostream>

using namespace std;
line_obstacle::line_obstacle(global_params params, Point p1_, Point p2_) {
    x_range = params.x_range;
    y_range = params.y_range;

    cell_size = params.cell_size;
    p1 = p1_;
    p2 = p2_;

    parameters = params;
}


tuple<list<array<int, 2>>, list<array<int, 2>>> line_obstacle::update_obstacle() {
        list<array<int, 2>> occ_old = occ_cells;
        occ_cells.clear();
        // TODO: Update-Funktion für dynamisch
        occ_cells = this->sort_in();
        return{occ_old, occ_cells};
}

bool line_obstacle::check_collision(Point p1_, Point p2_) {
    return intersect(p1, p2, p1_, p2_);
}

list<array<int, 2>> line_obstacle::sort_in() {
    occ_cells = intersected_cells(p1, p2, parameters);
    return occ_cells;
}

circle_obstacle::circle_obstacle(Point p_center_, float r_, Range x_range_, Range y_range_, float cell_size_) {
    x_range = x_range_;
    y_range = y_range_;
    cell_size = cell_size_;
    p_center = p_center_;
    r = r_;
}

tuple<list<array<int, 2>>, list<array<int, 2>>> circle_obstacle::update_obstacle() {
        // TODO: Update-Funktion für dynamisch
        return{occ_cells, occ_cells};
}

bool circle_obstacle::check_collision(Point p1, Point p2) {
    float dx = p2.at(0) - p1.at(0);
    float dy = p2.at(1) - p1.at(1);

    float fx = p1.at(0) - p_center.at(0);
    float fy = p1.at(1) - p_center.at(1);

    float a = pow(dx, 2) + pow(dy, 2);
    float b = 2 * (fx *dx + fy * dy);
    float c = pow(fx, 2) + pow(fy, 2) - pow(r, 2);

    // TODO: Schnittpunkt Linie Kreis (siehe Python Implementierung)
    return false;
}

list<array<int, 2>> circle_obstacle::sort_in(){
    // TODO: Betroffene Zellen finden
    return occ_cells;
}