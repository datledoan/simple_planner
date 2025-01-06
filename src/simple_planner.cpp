#include <angles/angles.h>
#include <simple_planner/simple_planner.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_planner::SimplePlanner, nav_core::BaseGlobalPlanner)

namespace simple_planner {

  SimplePlanner::SimplePlanner()
  : costmap_ros_(NULL), costmap_(NULL), initialized_(false){}

  SimplePlanner::SimplePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  SimplePlanner::~SimplePlanner() {}
  
  void SimplePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      waypoints_sub_ = private_nh.subscribe("/waypoints", 1, &SimplePlanner::waypointsCB, this);
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  void SimplePlanner::waypointsCB(const geometry_msgs::PoseArray::ConstPtr& waypoints){
    ROS_INFO("Got new waypoints");
    waypoints_ = *waypoints;
  }

  void SimplePlanner::findWaypoints(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseArray& waypoints, std::vector<geometry_msgs::PoseStamped>& plan_waypoints){
    double min_dist_start = std::numeric_limits<double>::infinity();
    double min_dist_goal = std::numeric_limits<double>::infinity();
    int start_index = 0;
    int goal_index = 0;
    for (int i = 0; i < waypoints.poses.size(); i++) {
      double dx = waypoints.poses[i].position.x - start.pose.position.x;
      double dy = waypoints.poses[i].position.y - start.pose.position.y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist_start) {
        min_dist_start = dist;
        start_index = i;
      }
      dx = waypoints.poses[i].position.x - goal.pose.position.x;
      dy = waypoints.poses[i].position.y - goal.pose.position.y;
      dist = std::hypot(dx, dy);
      if (dist < min_dist_goal) {
        min_dist_goal = dist;
        goal_index = i;
      }
    }

    plan_waypoints.clear();
    plan_waypoints.push_back(start);
    if(waypoints.poses.size() > 0){
      if (start_index < goal_index) {
        for (int i = start_index; i <= goal_index; i++) {
          geometry_msgs::PoseStamped waypoint;
          waypoint.header.frame_id = start.header.frame_id;
          waypoint.pose = waypoints.poses[i];
          waypoint.pose.orientation = start.pose.orientation;
          plan_waypoints.push_back(waypoint);
        }
      } else {
        for (int i = start_index; i >= goal_index; i--) {
          geometry_msgs::PoseStamped waypoint;
          waypoint.header.frame_id = start.header.frame_id;
          waypoint.pose = waypoints.poses[i];
          waypoint.pose.orientation = start.pose.orientation;
          plan_waypoints.push_back(waypoint);
        }
      }
    }
    plan_waypoints.push_back(goal);
  }

  void SimplePlanner::generatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end, std::vector<geometry_msgs::PoseStamped>& plan)
    {
      double dx = end.pose.position.x - start.pose.position.x;
      double dy = end.pose.position.y - start.pose.position.y;
      double distance = std::hypot(dx, dy);
      int num_waypoints = (int)(distance / step_size_) + 1;

      plan.push_back(start);
      for (int i = 0; i < num_waypoints; i++) {
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.frame_id = start.header.frame_id;
        waypoint.pose.position.x = start.pose.position.x + i * dx / num_waypoints;
        waypoint.pose.position.y = start.pose.position.y + i * dy / num_waypoints;
        waypoint.pose.position.z = start.pose.position.z;
        waypoint.pose.orientation = start.pose.orientation;
        plan.push_back(waypoint);
      }
      plan.push_back(end);
    }

  bool SimplePlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    SimplePlanner::findWaypoints(start, goal, waypoints_, plan_waypoints_);

    for (int i = 0; i < plan_waypoints_.size() - 1; i++) {
      ROS_INFO_STREAM("WP: " << plan_waypoints_[i].pose.position.x << ", " << plan_waypoints_[i].pose.position.y << " -> " << plan_waypoints_[i + 1].pose.position.x << ", " << plan_waypoints_[i + 1].pose.position.y);
      SimplePlanner::generatePath(plan_waypoints_[i], plan_waypoints_[i + 1], plan);
    }

  return true;
  }
};
