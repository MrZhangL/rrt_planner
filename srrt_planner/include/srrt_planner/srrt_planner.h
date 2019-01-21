/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef SRRT_PLANNER_H_
#define SRRT_PLANNER_H_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** include standard libraries **/
#include <iostream>
#include <limits>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <boost/random.hpp>

#include <srrt_planner/srrt_vertex.h>

namespace srrt_planner{
  /**
   * @class CarrotPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class srrtPlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the CarrotPlanner
       */
      srrtPlanner();
      /**
       * @brief  Constructor for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      srrtPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the CarrotPlanner
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
    * @brief returns the obstacle map
    * @return std::vector<bool> Unsafe cells are false, safe cells are true
    */
    std::vector<bool> getObstacleMap() {
      return obstacle_map_;
    }

    /**
    * @brief returns the rrt vertex tree
    */
    std::vector<srrt_planner::Vertex> getVertexTree() {
      return vertex_list_;
    }

     /**
     * @brief Gets a random point in the map space
     * @return returns an x,y pair
     */
     std::pair<float, float> GetRandomPoint();

     /**
     * @brief Gets the closest vertex to the given point
     * @param random_point A point in the map space
     * @param vertex_list the rrt vertex tree
     * @return the index of the closest vertex to the given point
     */
     int GetClosestVertex(std::pair<float, float> random_point);

    /**
    * @brief adds a new vertex to the rrt vertex tree
    * @param new_vertex the new vertex to be added
    */
    void addVertex(srrt_planner::Vertex new_vertex) {
      vertex_list_.push_back(new_vertex);
    }

     /**
     * @brief Euclidean distance between two points
     * @param start_point starting point
     * @param end_point ending point
     * @return euclidean distance between the points
     */
     float GetDistance(std::pair<float, float> start_point,
                       std::pair<float, float> end_point);

     /**
     * @brief Moves from the closest vertex towards the random point
     * @detail Begins at the closest point and attempts to move step_size_
     * towards the random point. Each step along the way at delta_ intervals
     * is checked for obstacles. If an obstacle is encountered the function
     * returns false. If it makes it from the closest vertex to step_size_
     * towards the random point a new vertex is created and added to
     * vertex_list_ and the function returns true.
     * @return true if a move was made, false if blocked by obstacle
     */
     bool MoveTowardsPoint(int closest_vertex,
                           std::pair<float, float> random_point);

     /**
     * @brief Is vertex within goal_radius_ of the goal
     * @param the vertex to be checked
     * @return true if within goal_radius_
     */
     bool ReachedGoal(int new_vertex);

     /**
     * @brief builds the plan from vertices and returns in PoseStamped
     * @param goal_index the index of the vertex that has reached the goal
     * @param start the starting location of the robot as passed to makePlan
     * @param goal the goal location of the robot as passed to makePlan
     * @return a vector of geometry_msgs:PoseStamped from the start to the goal
     */
     std::vector<geometry_msgs::PoseStamped>
       BuildPlan(int goal_index,
                 const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal);

     /**
     * @brief returns the best path
     * @param start starting point of robot
     * @param goal goal point
     * @return returns the index of the point that reaches the goal
     */
     int FindPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal);

     /**
     * @brief Checks if the path is safe between start_point and end_point
     * @param start_point starting point location
     * @param end_point ending point location
     * @return true if path between points does not intersect obstacles
     */
     bool IsSafe(std::pair<float, float> start_point,
                 std::pair<float, float> end_point);

    private:
      
      double min_dist_from_robot_;
  

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
     ros::NodeHandle node_handle_;

     ros::Publisher pub_;

     visualization_msgs::Marker points_,line_;

     geometry_msgs::Point p_; 
     /**
     * @brief obstacles
     */
     std::vector<bool> obstacle_map_;

     /**
     * @brief The ROS wrapper for the costmap the controller will use
     */
     costmap_2d::Costmap2DROS* costmap_ros_;

     /**
     * @brief The ROS wrapper for the costmap the controller will use
     */
     costmap_2d::Costmap2D* costmap_;

     unsigned char lethal_cost_;

     /**
     * @brief the max number of iterations to try and find a path
     */
     int max_iterations_;

     /**
     * @brief the current number of iterations
     */
     int current_iterations_;

     /**
     * @brief World model associated to the costmap
     */
     base_local_planner::WorldModel* world_model_;

     /**
     * @brief Check if the global planner is initialized
     */
     bool initialized_;

     /**
     * @brief How close to the goal is close enough
     */
     float goal_radius_;

     /**
     * @brief Size of the step the RRT planner takes
     */
     double step_size_;

     /**
     * @brief Size of the sub-step used for collision checking
     */
     float delta_;

     /**
     * @brief x coordinate of robot origin
     */
     float x_origin_;

     /**
     * @brief y coordinate of robot origin
     */
     float y_origin_;

     /**
     * @brief x coordinate of goal
     */
     float x_goal_;

     /**
     * @brief y coordinate of goal
     */
     float y_goal_;

     /**
     * @brief width of 2d map in cells
     */
     unsigned int map_width_cells_;

     /**
     * @brief height of 2d map in cells
     */
     unsigned int map_height_cells_;

     /**
     * @brief List of vertices
     */
     std::vector<srrt_planner::Vertex> vertex_list_;
  };
};  
#endif
