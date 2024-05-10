//
// Created by gr-agv-lx91 on 24-5-7.
//

#ifndef DEEP_LEARNING_PLANNER_SIMPLE_MOVE_BASE_H
#define DEEP_LEARNING_PLANNER_SIMPLE_MOVE_BASE_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "costmap_2d/costmap_2d_ros.h"
#include "actionlib/server/simple_action_server.h"
#include "isaac_sim/PlanAction.h"
#include "global_planner/planner_core.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/utils.h"
#include "angles/angles.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_srvs/Empty.h"

typedef actionlib::SimpleActionServer<isaac_sim::PlanAction> Server;

class SimpleMoveBase {
public:
    explicit SimpleMoveBase(tf2_ros::Buffer &tf_buffer);

private:
    void make_plan(const isaac_sim::PlanGoalConstPtr &goal);

    bool clear(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

    ros::NodeHandle node;
    tf2_ros::Buffer &buffer;
    global_planner::GlobalPlanner *planner;
    costmap_2d::Costmap2DROS *planner_costmap_ros;
    Server plan_server;
    ros::ServiceServer clear_server;
};

#endif //DEEP_LEARNING_PLANNER_SIMPLE_MOVE_BASE_H
