#include "GMRF.h"

GMRF::GMRF(global_params parameters) {
    /*
     * x_, y_: range of field to be explored
     * cell_size_: defines number of GMRF nodes
     * meas_noise_; SQUARED (!) measurement noise of sensor
     */
    x_range = parameters.x_range;
    y_range = parameters.y_range;
    meas_noise = parameters.meas_noise;

    // add border
    border = 1;
    cell_size = (x_range.at(1) - x_range.at(0)) / float(parameters.gmrf_nx);
    n_x = parameters.gmrf_nx + 2 * border;
    n_y = parameters.gmrf_ny + 2 * border;

    int n_nodes = n_x * n_y;

    mean_belief = Eigen::VectorXd::Constant(n_nodes + 1,5);
    PrecisionMatrix.resize(n_nodes + 1, n_nodes + 1);
    covariance.resize(n_nodes + 1, n_nodes + 1);
    PHI.resize(n_nodes + 1);

    PrecisionMatrix = initialize_prec_mat(n_x, n_y);
    SimplicialLLT<SparseMatrix<double> > solver;
    solver.compute(PrecisionMatrix);
    SparseMatrix<double> I(n_nodes + 1, n_nodes + 1);
    I.setIdentity();
    covariance = solver.solve(I);
    cov_diagonal = covariance.diagonal();
    canonical_mean = PrecisionMatrix * mean_belief;

    for (float x = x_range.at(0) + 0.1; x <= x_range.at(1) - 0.1; x += (x_range.at(1) - x_range.at(0))/20){
        vector<array<float, 2>> test_y;
        for (float y = y_range.at(0) + 0.1; y <= y_range.at(1) - 0.1; y += (y_range.at(1) - y_range.at(0))/20) {
            test_y.push_back((array<float, 2>){x, y});
        }
        RMSE_TestPoints.push_back(test_y);
    }

}


void GMRF::condition(Point location, double measurement) {
    update_PHI(location);

    SimplicialLLT<SparseMatrix<double> > solver;
    VectorXd h_seq = solver.compute(PrecisionMatrix).solve(PHI);

    // update canonical mean
    canonical_mean += measurement/meas_noise * PHI;

    // update precision matrix
    PrecisionMatrix += (1/ meas_noise) * PHI * PHI.transpose();

    // update mean
    mean_belief = solver.compute(PrecisionMatrix).solve(canonical_mean);

    // update diagonal entries of covariance
    cov_diagonal -= h_seq.cwiseProduct(h_seq) / (meas_noise + PHI.dot(h_seq));
}

void GMRF::update_PHI(Point location) {
    PHI.setZero();
    int x_ind = (int)floor((location.at(0) - x_range.at(0)) / cell_size) + 1;
    int y_ind = (int)floor((location.at(1) - y_range.at(0)) / cell_size) + 1;

    int cell = (y_ind) * (n_x) + (x_ind);
    double x_center = (float)(x_ind - 1) * cell_size;
    double y_center = (float)(y_ind - 1) * cell_size;
    double xRel = location.at(0) - x_center;
    double yRel = location.at(1) - y_center;

    vector<int> neighbour_nodes;
    neighbour_nodes.push_back(cell);
    neighbour_nodes.push_back(cell + 1);
    neighbour_nodes.push_back(cell + n_x);
    neighbour_nodes.push_back(cell + n_x + 1);

    PHI.coeffRef(neighbour_nodes[0]) = ((xRel - cell_size) * (yRel - cell_size)) / (pow(cell_size, 2));
    PHI.coeffRef(neighbour_nodes[1]) = - ((xRel) * (yRel - cell_size)) / (pow(cell_size, 2));
    PHI.coeffRef(neighbour_nodes[2]) = - ((xRel - cell_size) * (yRel)) / (pow(cell_size, 2));
    PHI.coeffRef(neighbour_nodes[3]) = ((xRel) * (yRel)) / (pow(cell_size, 2));
}

double GMRF::return_mean_at_location(Point location_) {
    update_PHI(location_);
    return PHI.dot(mean_belief);
}

double GMRF::return_cov_at_location(Point location_) {
    update_PHI(location_);
    return PHI.dot(cov_diagonal);
}

// empty constructor
GMRF::GMRF() {}

/*void GMRF::plot_belief(sf::RenderWindow* window, global_params global_p, bool mean_or_cov) {
    Point plotting_res = {100, 100};
    sf::Image image;
    sf::Texture texture;
    sf::Sprite sprite;
    image.create(global_p.V_x.at(1), global_p.V_y.at(1));
    texture.create(global_p.V_x.at(1), global_p.V_y.at(1));
    sprite.setScale(1, 1);
    float m;
    float scalex = (float) (x_range.at(1) - x_range.at(0)) / (float) (global_p.V_x.at(1) - global_p.V_x.at(0));
    float scaley = (float) (y_range.at(1) - y_range.at(0)) / (float) (global_p.V_y.at(1) - global_p.V_y.at(0));
    int pixel_array_size_x = (global_p.V_x.at(1) - global_p.V_x.at(0)) / plotting_res.at(0);
    int pixel_array_size_y = (global_p.V_y.at(1) - global_p.V_y.at(0)) / plotting_res.at(1);
    for (int x_ = global_p.V_x.at(0); x_ < global_p.V_x.at(1) - 1; x_ += pixel_array_size_x) {
        for (int y_ = global_p.V_y.at(0); y_ < global_p.V_y.at(1) - 1; y_ += pixel_array_size_y) {
            if (mean_or_cov) {
                m = 120 * return_cov_at_location(
                        (Point) {x_range.at(0) + (float) (x_ - global_p.V_x.at(0)) * scalex, (y_range.at(0) + (float) (y_ - global_p.V_y.at(0)) * scaley)});
            }
            else {
                m = return_mean_at_location(
                        (Point) {x_range.at(0) + (float) (x_ - global_p.V_x.at(0)) * scalex, (y_range.at(0) + (float) (y_ - global_p.V_y.at(0)) * scaley)});
            }
            for (int k = x_; k < x_ + pixel_array_size_x; k++) {
                for (int l = y_; l < y_ + pixel_array_size_x; l++) {
                    if (mean_or_cov) {
                        m = min(max((int)m, 0), 255);
                        image.setPixel(k, l, sf::Color(m, 10, 100));
                    }
                    else{
                        image.setPixel(k, l, sf::Color(round(255 * (m/100)), round((255/99) * (100 - m)), 0));
                    }
                }
            }
        }
    }
    texture.update(image);
    sprite.setTexture(texture);
    window->draw(sprite);
}*/

/*
tuple<float, float> GMRF::calc_RMSE(TrueField* true_f) {
    float RMSE = 0;
    float wRMSE = 0;
    float weight;
    float diff;
    int n = 0;
    for(const auto& row:RMSE_TestPoints){
        for(auto pos:row){
            diff = pow((return_mean_at_location(pos) - true_f->get_value(pos)), 2);
            RMSE += diff;
            // TODO: adapt weight
            weight = 1;
            wRMSE += weight * diff;
            n++;
        }
    }
    return make_tuple(sqrt(RMSE/(float)n), sqrt(wRMSE/(float)n));
}
*/
float GMRF::return_predictive_variance() {
    return cov_diagonal.sum() - cov_diagonal.coeffRef(cov_diagonal.size() - 1);
}

vector<double> GMRF::return_cov_python_format(){
    vector<double> cov_python;
    float dx = (x_range.at(1) - x_range.at(0)) / (float)(n_x - 2);
    float dy = (y_range.at(1) - y_range.at(0)) / (float)(n_y - 2);
    for (float x = x_range.at(0); x <= x_range.at(1); x += dx){
        for (float y = y_range.at(0); y <= y_range.at(1); y += dy) {
            Point location = {max(min(x, (float)(x_range.at(1) - 0.01)), (float)(x_range.at(0) + 0.001)),
                              max(min(y, (float)(y_range.at(1) - 0.01)), (float)(y_range.at(0) + 0.001))};
            cov_python.push_back(return_cov_at_location(location));
        }
    }
    return cov_python;
}
