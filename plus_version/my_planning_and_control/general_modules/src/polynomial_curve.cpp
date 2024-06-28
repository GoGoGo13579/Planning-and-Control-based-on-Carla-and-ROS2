#include "polynomial_curve.h"

void PolynomialCurve::curve_fitting(const double& start_s, const double& start_l, const double& start_l_prime, const double& start_l_prime_prime,
                                    const double& end_s, const double& end_l, const double& end_l_prime, const double& end_l_prime_prime)
{
    Eigen::MatrixXd A;
    Eigen::VectorXd B;
    //double corrected_start_l_prime_prime = std::min(start_l_prime_prime, 10.0);
    double corrected_start_l_prime_prime = start_l_prime_prime;
    A.resize(6,6);
    B.resize(6);
    A << 1, start_s,  std::pow(start_s,2),   std::pow(start_s,3),   std::pow(start_s,4),   std::pow(start_s,5),
         0,   1,    2*std::pow(start_s,1), 3*std::pow(start_s,2), 4*std::pow(start_s,3), 5*std::pow(start_s,4),
         0,   0,        2,                 6*std::pow(start_s,1),12*std::pow(start_s,2),20*std::pow(start_s,3),
         1, end_s,    std::pow(end_s,2),     std::pow(end_s,3),     std::pow(end_s,4),     std::pow(end_s,5),
         0,   1,    2*std::pow(end_s,1),   3*std::pow(end_s,2),   4*std::pow(end_s,3),   5*std::pow(end_s,4),
         0,   0,        2,                 6*std::pow(end_s,1),  12*std::pow(end_s,2),  20*std::pow(end_s,3);

    B << start_l,start_l_prime,corrected_start_l_prime_prime,end_l,end_l_prime,end_l_prime_prime;
    _coefficients =  A.fullPivLu().solve(B);

    // std::cout << A << std::endl << B <<std::endl << _coefficients << std::endl;
}

double PolynomialCurve::value_evaluation(const double& s, const int& order)
{
    Eigen::VectorXd phi_s;
    phi_s.resize(6);

    if (order == 0)//0阶导数，就是原函数求值
    {
        phi_s << 1, s, std::pow(s,2), std::pow(s,3), std::pow(s,4), std::pow(s,5);

    }
    else if(order == 1)
    {
        phi_s << 0, 1, 2*std::pow(s,1), 3*std::pow(s,2), 4*std::pow(s,3), 5*std::pow(s,4);
    }
    else if(order == 2)
    {
        phi_s << 0, 0, 2, 6*std::pow(s,1), 12*std::pow(s,2), 20*std::pow(s,3);
    }
    else if(order == 3)
    {
        phi_s << 0, 0, 0, 6, 24*s, 60*std::pow(s,2);
    }

    return phi_s.dot(_coefficients);
    
}
