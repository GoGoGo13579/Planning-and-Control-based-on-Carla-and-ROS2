#pragma once

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "common.h"



class LaterLQRController
{
public:
    LaterLQRController();

private:
    //车辆参数
    double _cf; //前轮侧偏刚度系数
    double _cr; //后轮侧偏刚度系数
    double _m;  //质量
    double _vx; //沿着车身轴线的速度
    double _Iz; //车身转动惯量
    double _a; //质心到车前轴的距离
    double _b; //质心到车后轴的距离
    double _steer_ratio; //方向盘转角到轮胎转角之间的比值系数

    //lqr参数
    int _matrix_size;
    Eigen::MatrixXd _matrix_A; //状态矩阵
    Eigen::MatrixXd _matrix_A_bar; //离散化的状态矩阵
    Eigen::MatrixXd _matrix_B; //输入矩阵
    Eigen::MatrixXd _matrix_B_bar; //离散化的矩阵
    Eigen::MatrixXd _matrix_K; //反馈矩阵
    Eigen::VectorXd _matrix_err; //误差向量
    Eigen::MatrixXd _matrix_Q; //Q矩阵
    Eigen::MatrixXd _matrix_R; //R矩阵
    int _iter_max; //最大迭代次数
    double _tolerance; //迭代精度


    
public:
    double run_step(const TrajectoryPoint& match_point, const VehicleState& current_ego_state,
                    const double& control_time_step);

    bool SolveLQRFeedack(const Eigen::MatrixXd& A,const Eigen::MatrixXd& B,const Eigen::MatrixXd& Q,
                                    const Eigen::MatrixXd& R,const int& iter_max,const double& tolerance);


    
};