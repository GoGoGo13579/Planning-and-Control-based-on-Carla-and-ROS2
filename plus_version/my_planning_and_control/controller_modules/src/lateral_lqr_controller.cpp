#include "lateral_lqr_controller.h"



LateralLQRController::LateralLQRController()
{
    //车辆静态参数初始化
    _cf = -155494.663;
    _cr = -155494.663;
    _m = 1845.0;
    _a = 2.852/2.0;
    _b = 2.852/2.0;
    _Iz = _a*_a*(_m/2.0) + _b*_b*(_m/2.0);
    _steer_ratio = 16.0;
    
    //lqr静态矩阵初始化
    _matrix_size = 4;
    _matrix_A = Eigen::MatrixXd::Zero(_matrix_size,_matrix_size);
    _matrix_A_bar = Eigen::MatrixXd::Zero(_matrix_size,_matrix_size);
    _matrix_B = Eigen::MatrixXd::Zero(_matrix_size,1);
    _matrix_B_bar = Eigen::MatrixXd::Zero(_matrix_size,1);
    _matrix_K = Eigen::MatrixXd::Zero(1,_matrix_size);
    _matrix_err = Eigen::VectorXd::Zero(_matrix_size);
    _matrix_Q = Eigen::MatrixXd::Identity(_matrix_size,_matrix_size);
    _matrix_R = Eigen::MatrixXd::Identity(1,1);

    _matrix_B(1,0) = -_cf/_m;
    _matrix_B(3,0) = -_a*_cf/_Iz;

    //默认权重
    _matrix_R(0,0) = 1.0;//输入权重
    
    _matrix_Q(0,0) = 0.05;//横向距离误差权重
    _matrix_Q(1,1) = 0.0;//横向距离误差导数权重
    _matrix_Q(2,2) = 1.0;//横向角度误差权重
    _matrix_Q(3,3) = 0.0;//横向角度误差导数权重



    _iter_max = 1500;
    _tolerance = 0.01;

}

bool LateralLQRController::compute_control_cmd(
                        const std::vector<TrajectoryPoint>& trajectory,
                        const std::shared_ptr<VehicleState>& ego_state,
                        const double cur_t, const double dt,
                        ControlCMD & cmd)
{
    auto LOG = rclcpp::get_logger("laterl_lqr_controller");
    //1.创建状态矩阵
    _vx = ego_state->v + 1e-4;//除0保护,这里这个v还不确定对不对，要不要做一个投影
    _matrix_A(0,1) = 1.0;
    _matrix_A(1,1) = (_cf + _cr)/(_m * _vx);
    _matrix_A(1,2) = -(_cf + _cr)/(_m);
    _matrix_A(1,3) = (_a*_cf - _b*_cr)/(_m * _vx);
    _matrix_A(2,3) = 1.0;
    _matrix_A(3,1) = (_a*_cf - _b*_cr)/(_Iz * _vx);
    _matrix_A(3,2) = -(_a*_cf - _b*_cr)/(_Iz);
    _matrix_A(3,3) = (_a*_a*_cf + _b*_b*_cr)/(_Iz * _vx);

    //2.离散化，前向欧拉离散
    auto I = Eigen::MatrixXd::Identity(_matrix_size,_matrix_size);
    _matrix_A_bar = (I + _matrix_A*dt);
    _matrix_B_bar = _matrix_B * dt;

    //3.求解反馈矩阵
    if (! compute_lqr_feedack(_matrix_A_bar,_matrix_B_bar,_matrix_Q,_matrix_R,_iter_max,_tolerance))
    {
        RCLCPP_ERROR(LOG,"反馈矩阵k求解失败");
        _matrix_K = Eigen::MatrixXd::Zero(1,_matrix_size);
    }

    //4.计算误差
    //4.1计算目标点
    double predicted_time = cur_t + _preview_window*dt;
    TrajectoryPoint target_point;
    compute_target_point(trajectory, predicted_time, target_point);

    Eigen::Vector2d tao(std::cos(target_point.heading),std::sin(target_point.heading));
    Eigen::Vector2d nor(-std::sin(target_point.heading),std::cos(target_point.heading));
    Eigen::Vector2d vec_d(ego_state->x - target_point.x, ego_state->y - target_point.y);

    double ed = nor.transpose() * vec_d;//横向距离误差
   // double es = tao.transpose() * vec_d;//纵向距离误差

    //double theta_r = target_point.heading;//Apollo版本
    //double theta_r = target_point.heading + target_point.kappa * es;//老王版本
    double ephi = ego_state->heading - target_point.heading;
    //修正角度误差取值区间
    if (ephi > M_PI)
    {
        ephi = ephi - 2*M_PI;
    }
    else if (ephi < -M_PI)
    {
        ephi = ephi + 2*M_PI;
    }
    //为了避免角度多值问题，可以使用sin近似来避免
    ephi = std::sin(ephi);

    double ed_dot = ego_state->v * std::sin(ephi);
    double ephi_dot = ego_state->omega - target_point.kappa * target_point.v;

    _matrix_err[0] = ed;
    _matrix_err[1] = ed_dot;
    _matrix_err[2] = ephi;
    _matrix_err[3] = ephi_dot;


    //5.计算前馈
    double k3 = _matrix_K(0,2);
    double k_m = target_point.kappa;
    double delta_f = k_m*(_a + _b - _b*k3 - (_m*_vx*_vx/(_a+_b)) * (_b/_cf + _a*k3/_cr - _a/_cr));

    //6.计算前轮转角
    double u = -(_matrix_K*_matrix_err)[0] + delta_f;
    //限制前轮转角
    double max_u = 60.0 * M_PI / 180.0;
    u = std::min(std::max(u,-max_u),max_u);

    //7.计算方向盘转角

    double steer = -u;//前轮转角计算出负值应该是右转，所以有一个负号。
    #ifdef CONTROL_DEBUG
    RCLCPP_INFO(LOG,"横向误差(%.3f,%.3f,%.3f,%.3f),输出(%.3f)",_matrix_err[0],_matrix_err[1],
            _matrix_err[2],_matrix_err[3],std::min(std::max(steer,-1.0),1.0));
    #endif
    cmd.steer =  std::min(std::max(steer,-1.0),1.0);

    return true;
}

//设置q矩阵
void LateralLQRController::set_q_matrix(const std::vector<double>& q_vector)
{
    if (static_cast<int>(q_vector.size()) != static_cast<int>(_matrix_Q.rows()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("laterl_lqr_controller"),"输入维度不争取,设置失败");
    }

    for (size_t i = 0; i < q_vector.size(); i++)
    {
        _matrix_Q(i,i) = q_vector[i]; 
    }
    
}

//设置r矩阵
void LateralLQRController::set_r_matrix(const double r)
{
    _matrix_R(0,0) = r;
}

//计算目标点位置
void LateralLQRController::compute_target_point(
     const std::vector<TrajectoryPoint>& trajectory,
     const double predicted_time, TrajectoryPoint& target_point)
{
    int index_nearest = -1;
    for (int i = 0; i < static_cast<int>(trajectory.size())-1; ++i)
    {
        if (predicted_time >= trajectory[i].time_stamped && predicted_time < trajectory[i+1].time_stamped)
        {
            index_nearest = i;
        }
    }
    if (index_nearest == -1)
    {
        return ;
    }

    double delta_t = (trajectory.at(index_nearest+1).time_stamped - trajectory.at(index_nearest).time_stamped);
    double dt = predicted_time - trajectory.at(index_nearest).time_stamped;

    double k_x = (trajectory.at(index_nearest+1).x - trajectory.at(index_nearest).x)/delta_t;
    target_point.x = trajectory.at(index_nearest).x + k_x*dt;

    double k_y = (trajectory.at(index_nearest+1).y - trajectory.at(index_nearest).y)/delta_t;
    target_point.y = trajectory.at(index_nearest).y + k_y*dt;

    double k_v = (trajectory.at(index_nearest+1).v - trajectory.at(index_nearest).v)/delta_t;
    target_point.v = trajectory.at(index_nearest).v + k_v*dt;

    double k_heading = (trajectory.at(index_nearest+1).heading - trajectory.at(index_nearest).heading)/delta_t;
    target_point.heading = trajectory.at(index_nearest).heading + k_heading*dt;

    double k_a_tau = (trajectory.at(index_nearest+1).a_tau - trajectory.at(index_nearest).a_tau)/delta_t;    
    target_point.a_tau = trajectory.at(index_nearest).a_tau + k_a_tau*dt; 
}


bool LateralLQRController::compute_lqr_feedack(
     const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
     const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
     const int iter_max,const double tolerance)
{
    auto LOG = rclcpp::get_logger("laterl_lqr_controller");
    //判断输入矩阵的维数是否正确
    if(A.rows()!=A.cols() || B.rows()!=A.rows() || Q.rows()!=Q.cols() || Q.rows()!=A.rows() || R.cols()!=B.cols())
    {
        RCLCPP_INFO(LOG,"输入矩阵维数不匹配");
        return false;
    }

    auto matrix_size = A.rows();
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_next;
    Eigen::MatrixXd AT;
    Eigen::MatrixXd BT;
    //初始化
    P = Eigen::MatrixXd::Zero(matrix_size,matrix_size);//P初始化成Q矩阵也可以，因为P初始化为0矩阵的话，第一次迭代P_next就是Q矩阵
    P_next = Eigen::MatrixXd::Zero(matrix_size,matrix_size);
    AT = A.transpose();
    BT = B.transpose();

    //迭代计算黎卡提方程
    for (int i = 0; i < iter_max; i++)
    {   
        P_next = Q + AT*P*A - AT*P*B*(R+BT*P*B).inverse()*BT*P*A;
        if (fabs((P_next-P).maxCoeff()) < tolerance)
        {
            _matrix_K = (R + BT*P*B).inverse()*(BT*P*A);
            return true;
        }
        //第一次写忘记写
        P = P_next;
    }
    return false;
}
