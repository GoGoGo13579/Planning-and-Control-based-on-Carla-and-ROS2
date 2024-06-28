#include "pid_controller.h"

void PIDController::set_controller(const double k_p, const double k_i, 
                                   const double k_d)
{
    _k_p = k_p;
    _k_i = k_i;
    _k_d = k_d;
}

void PIDController::computer_control_cmd(const double error, const double dt,
                                         double& control_cmd)
{
    //1.计算微分项
    double diff = 0.0;//误差的差值
    if (_first_hit)//首次启用
    {
        diff = 0.0;
    }
    diff = error - _previous_error;

    //2.计算积分项
    _integral_part += _k_i*error*dt;
    //大于限幅上界就取上界
    _integral_part = _integral_part <= _integral_saturation_high ?
                     _integral_part : _integral_saturation_high;
    //小于限幅下界就取下界
    _integral_part = _integral_part >= _integral_saturation_low ?
                     _integral_part : _integral_saturation_low;


    //3.计算控制指令输出
    control_cmd = _k_p*error + _integral_part + _k_d*diff;
    
}

void PIDController::set_integral_saturation_boundary(
        const double high_boundary,
        const double low_boundary
)
{
    _integral_saturation_high = high_boundary;
    _integral_saturation_low = low_boundary;
}

void PIDController::reset_integral()
{
    _integral_part = 0.0;
}