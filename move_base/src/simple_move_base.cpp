#include "move_base/simple_move_base.h"

SimpleMoveBase::SimpleMoveBase(tf2_ros::Buffer &tf_buffer) :
        buffer(tf_buffer),
        plan_server(node, "plan", boost::bind(&SimpleMoveBase::make_plan, this, _1), false) {
    planner_costmap_ros = new costmap_2d::Costmap2DROS("costmap", buffer);
    planner = new global_planner::GlobalPlanner();
    planner->initialize("planner", planner_costmap_ros);
    clear_server = node.advertiseService("clear", &SimpleMoveBase::clear, this);
    plan_server.start();
}

void SimpleMoveBase::make_plan(const isaac_sim::PlanGoalConstPtr &goal) {
    std::vector<geometry_msgs::PoseStamped> plan;
    isaac_sim::PlanResult result;
    geometry_msgs::PoseStamped start, target;
    if (planner->makePlan(goal->start, goal->target, plan)) {
        for (const auto &pose: plan) {
            double dx, dy, yaw, x, y;
            dx = pose.pose.position.x - goal->start.pose.position.x;
            dy = pose.pose.position.y - goal->start.pose.position.y;
            yaw = tf2::getYaw(goal->start.pose.orientation);
            double sin_yaw = -std::sin(yaw), cos_yaw = std::cos(yaw);
            x = dx * cos_yaw - dy * sin_yaw;
            y = dx * sin_yaw + dy * cos_yaw;
            result.x.push_back(x);
            result.y.push_back(y);
            result.yaw.push_back(angles::normalize_angle(tf2::getYaw(pose.pose.orientation)) - yaw);
        }
        plan_server.setSucceeded(result);
    } else {
        plan_server.setAborted();
    }
}

bool SimpleMoveBase::clear(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res) {
    planner->clear();
    planner_costmap_ros->resetLayers();
    ros::Duration(2.0).sleep();
    return true;
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
    std::shared_ptr<SimpleMoveBase> simple_move_base(new SimpleMoveBase(buffer));
    ros::spin();
    return 0;
}