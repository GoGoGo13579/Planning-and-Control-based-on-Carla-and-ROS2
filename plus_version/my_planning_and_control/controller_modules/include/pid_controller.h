#pragma once
#include <iostream>
#include <limits>

class PIDController
{
public:
    PIDController() {};
    //设置pid参数
    void set_controller(const double k_p, const double k_i, 
                        const double k_d);
    //计算控制指令
    void computer_control_cmd(const double error, const double dt,
                              double& control_cmd);

    //设置积分饱和上下限
    void set_integral_saturation_boundary(
        const double high_boundary,
        const double low_boundary
    );

    //重置积分参数
    void reset_integral();

private:
    double _k_p = 0.0;
    double _k_i = 0.0;
    double _k_d = 0.0;
    double _integral_part = 0.0;
    double _previous_error = 0.0;
    double _integral_saturation_high = std::numeric_limits<double>::max();//积分限幅上限
    double _integral_saturation_low = std::numeric_limits<double>::lowest();//积分限幅下限
    bool _first_hit = true;

};