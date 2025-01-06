#ifndef SIMPLE_PLANNER_H_
#define SIMPLE_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace simple_planner{
  /**
   * @class SimplePlanner
   * @brief Provides a simple global planner.
   */
  class SimplePlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the SimplePlanner
       */
      SimplePlanner();
      /**
       * @brief  Constructor for the SimplePlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      SimplePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Destructor
       */
      ~SimplePlanner();

      /**
       * @brief  Initialization function for the SimplePlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      
      /**
       * @brief Callback to update the waypoints from waypoint_rviz_plugin
       * @param waypoints The new waypoints
       */
      void waypointsCB(const geometry_msgs::PoseArray::ConstPtr& waypoints);

      /**
       * @brief Given a start and end pose, generate a path
       * @param start The start pose
       * @param end The end pose
       * @param plan The plan... filled by the planner
       */
      void generatePath(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      
      /**
       * @brief Given a start and end pose, find the waypoints
       * @param start The start pose
       * @param goal The end pose
       * @param waypoints The waypoints list
       * @param plan_waypoints The waypoints to follow
       */
      void findWaypoints(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseArray& waypoints ,std::vector<geometry_msgs::PoseStamped>& plan_waypoints);
      


    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      bool initialized_;
      ros::Subscriber waypoints_sub_;
      geometry_msgs::PoseArray waypoints_;
      std::vector<geometry_msgs::PoseStamped> plan_waypoints_;
  };
};  
#endif
