#include "emplanner.h"

DpPathNode::DpPathNode()
{
    sl_point = FrenetPoint();
    min_cost = -1.0;
    pre_node = nullptr;
}

DpPathNode::DpPathNode(const FrenetPoint& args_sl_point, const double& args_min_cost, const std::shared_ptr<DpPathNode> args_pre_node)
{
    sl_point = args_sl_point;
    min_cost = args_min_cost;
    pre_node = args_pre_node;
}

DpSpeedNode::DpSpeedNode()
{
    st_point = STPoint();
    min_cost = -1.0;
    pre_node = nullptr;
}

DpSpeedNode::DpSpeedNode(const STPoint& args_st_point, const double& args_min_cost, const std::shared_ptr<DpSpeedNode> args_pre_node)
{
    st_point = args_st_point;
    min_cost = args_min_cost;
    pre_node = args_pre_node;
}