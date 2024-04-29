#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "costmap_2d/costmap_2d_ros.h"
#include "actionlib/server/simple_action_server.h"
#include "isaac_sim/PlanAction.h"
#include "global_planner/planner_core.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/utils.h"
#include "angles/angles.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"

typedef actionlib::SimpleActionServer<isaac_sim::PlanAction> Server;

global_planner::GlobalPlanner *planner;

void execute(const isaac_sim::PlanGoalConstPtr &goal, Server *as) {
    std::vector<geometry_msgs::PoseStamped> plan;
    isaac_sim::PlanResult result;
    planner->makePlan(goal->start, goal->target, plan);
    for (const auto &pose: plan) {
        double dx, dy, yaw, x, y;
        dx = pose.pose.position.x - goal->start.pose.position.x;
        dy = pose.pose.position.y - goal->start.pose.position.y;
        yaw = tf2::getYaw(goal->start.pose.orientation);
        double sin_yaw = std::sin(yaw), cos_yaw = std::cos(yaw);
        x = dx * cos_yaw - dy * sin_yaw;
        y = dx * sin_yaw + dy * cos_yaw;
        result.x.push_back(x);
        result.y.push_back(y);
        result.yaw.push_back(angles::normalize_angle(tf2::getYaw(goal->target.pose.orientation) - yaw));
    }
    as->setSucceeded(result);
}

void static_transform() {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "base_link";
    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.0;
    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(static_transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_move_base");
    ros::console::set_logger_level(std::string("ros"), ros::console::levels::Info);
    ros::console::notifyLoggerLevelsChanged();
    ros::NodeHandle node;
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    static_transform();
    auto *planner_costmap_ros = new costmap_2d::Costmap2DROS("costmap", buffer);
    planner = new global_planner::GlobalPlanner();
    planner->initialize("planner", planner_costmap_ros);
    Server server(node, "plan", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}