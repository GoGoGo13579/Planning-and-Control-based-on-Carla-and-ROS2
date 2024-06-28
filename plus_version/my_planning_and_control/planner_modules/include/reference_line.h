#pragma once

#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "common.h"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"


class ReferenceLine
{
public:
    ReferenceLine();

private:
    //初始化参考线
    double _forward_num;//向前搜索个数
    double _backward_num;//向后搜索个数
    int _previous_match_point_index;
    bool _is_first_run;//是都第一次运行标志

    //参考线平滑
    std::shared_ptr<OsqpEigen::Solver> _smooth_solver;
    double _cost_smooth, _cost_geometry, _cost_compact;//代价
    double _max_x_offset, _max_y_offset;

public:
    bool run_step(std::shared_ptr<VehicleState> current_ego_state, const std::vector<PathPoint>& global_path, std::vector<PathPoint>& reference_line);


};