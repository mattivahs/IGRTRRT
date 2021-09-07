#include "helper_functions.h"
//#include "TrueField.h"
#ifndef GMRF_CPP_GMRF_H
#define GMRF_CPP_GMRF_H

using namespace Eigen;

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::SparseVector<double> SpVec;

class GMRF {
private:
    Range x_range;
    Range y_range;
    float cell_size;
    int n_x;
    int n_y;
    int border;
    float meas_noise;

    VectorXd canonical_mean;
    MatrixXd covariance;
    SpMat PrecisionMatrix;
    SpVec PHI;
    vector<vector<array<float, 2>>> RMSE_TestPoints;
    void update_PHI(Point location);
    double mean_field_covariance = 100;

public:

    GMRF();
    void update_mean_uncertainty(){
        mean_field_covariance = (cov_diagonal.sum() - cov_diagonal[cov_diagonal.size() - 1]) / cov_diagonal.size();
    };
    double return_mean_field_cov(){return mean_field_covariance;};
    GMRF(global_params parameters);
    void condition(Point location, double measurement);
    double return_mean_at_location(Point location_);
    double return_cov_at_location(Point location_);
    VectorXd* return_covariance(){return &cov_diagonal;};
    VectorXd* return_mean(){return &mean_belief;};
    std::tuple<int, int, double> return_size(){return std::make_tuple(n_x, n_y, cell_size);};
//     void plot_belief(sf::RenderWindow* window, global_params global_p, bool mean_or_cov=true);
// //    tuple<float, float> calc_RMSE(TrueField* true_f);
    float return_predictive_variance();
    VectorXd mean_belief;
    VectorXd cov_diagonal;
    vector<double> return_cov_python_format();
};


#endif //GMRF_CPP_GMRF_H
