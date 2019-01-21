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
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <srrt_planner/srrt_planner.h>
#include <srrt_planner/srrt_vertex.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>
#include <algorithm>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(srrt_planner::srrtPlanner, nav_core::BaseGlobalPlanner)

namespace srrt_planner {

  srrtPlanner::srrtPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  srrtPlanner::srrtPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void srrtPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle node("~/srrt_planner");
      node_handle_ = node;
      world_model_ = new base_local_planner::CostmapModel(*costmap_);

      pub_ = node.advertise<visualization_msgs::Marker>("visualization_marker",1);
      /*node_handle_.getParam("/move_base/step_size", step_size_);
      node_handle_.getParam("/move_base/delta", delta_);
      node_handle_.getParam("/move_base/goal_radius", goal_radius_);
      node_handle_.getParam("/move_base/max_iterations", max_iterations_);
      ROS_INFO("Step size: %.2f, goal radius: %.2f, delta: %.2f, max "
               "iterations: %d", step_size_, goal_radius_, delta_,
               max_iterations_);*/

     //visualizations  points and lines..
     points_.header.frame_id="/robot_1/map";
     line_.header.frame_id="/robot_1/map";
     points_.header.stamp=ros::Time(0);
     line_.header.stamp=ros::Time(0);
	
     points_.ns=line_.ns = "markers";
     points_.id = 0;
     line_.id =1;

     points_.type = points_.POINTS;
     line_.type=line_.LINE_LIST;

     //Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     points_.action =points_.ADD;
     line_.action = line_.ADD;
     points_.pose.orientation.w =1.0;
     line_.pose.orientation.w = 1.0;
     line_.scale.x =  0.02;
     line_.scale.y= 0.02;
     points_.scale.x=0.1; 
     points_.scale.y=0.1; 

     line_.color.r =9.0/255.0;
     line_.color.g= 236.0/255.0;
     line_.color.b =91.0/255.0;
     points_.color.r = 255.0/255.0;
     points_.color.g = 0.0/255.0;
     points_.color.b = 0.0/255.0;
     points_.color.a=1.0;
     line_.color.a = 1.0;
     points_.lifetime = ros::Duration();
     line_.lifetime = ros::Duration();

     /*****************/

      step_size_ = 10;
      delta_ = 1;
      goal_radius_ = 8;
      max_iterations_ = 5000;
      current_iterations_ = 0;
      lethal_cost_ = 100;

      // Get obstacles in the costmap
      map_width_cells_ = costmap_-> getSizeInCellsX();
      map_height_cells_ = costmap_-> getSizeInCellsY();

      for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
        for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
          /*unsigned char cost = static_cast<int>(costmap_->getCost(ix, iy));*/
          unsigned char cost = costmap_->getCost(ix, iy);
          if (cost >= lethal_cost_)
            obstacle_map_.push_back(false);
          else
            obstacle_map_.push_back(true);
        }
      }
      // Display info message
      ROS_INFO("RRT planner initialized successfully.");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }


  bool srrtPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

      // Check if we've initialized, if not error
    if (!initialized_) {
      ROS_ERROR("RRT planner has not been initialized, please call "
                "initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Start: %.2f, %.2f", start.pose.position.x,
              start.pose.position.y);
    ROS_DEBUG("Goal: %.2f, %.2f", goal.pose.position.x,
              goal.pose.position.y);

    // reset path, iterations, vertex tree
    plan.clear();
    current_iterations_ = 0;
    ROS_DEBUG("Current iterations reset to %d.", current_iterations_);
    vertex_list_.clear();

    // reset origin and goal
    x_origin_ = start.pose.position.x;
    y_origin_ = start.pose.position.y;
    x_goal_ = goal.pose.position.x;
    y_goal_ = goal.pose.position.y;

    // Initialize root node
    unsigned int mx,my;
    if (!costmap_->worldToMap(x_origin_, y_origin_, mx, my)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    
    x_origin_ = float(mx);
    y_origin_ = float(my);

    if (!costmap_->worldToMap(x_goal_, y_goal_, mx, my)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    
    x_goal_ = float(mx);
    y_goal_ = float(my);

    ROS_DEBUG("Start: %.2f, %.2f", x_origin_,
              y_origin_);
    ROS_DEBUG("Goal: %.2f, %.2f", x_goal_,
              y_goal_);

    srrt_planner::Vertex root(x_origin_, y_origin_, 0, -1);
    vertex_list_.push_back(root);

    // Make sure that the goal header frame is correct
    // Goals are set within rviz
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(),
                goal.header.frame_id.c_str());
      return false;
    }

    // Have the srrtPlanner calculate the path. Returns the index of the node
    // that reaches the goal
    ROS_DEBUG("Going into FindPath");
    int goal_index = srrtPlanner::FindPath(start, goal);

    // Rebuild the plan from the goal_index to the start using the
    // desired plan message format
    ROS_DEBUG("Going into BuildPlan");
    plan = srrtPlanner::BuildPlan(goal_index, start, goal);

    if (plan.size() > 1) {
      ROS_INFO("A path was found.");
      return true;
    } else {
      ROS_WARN("No path was found.");
      return false;
    }
  }

  std::pair<float, float> srrtPlanner::GetRandomPoint() {
    // generate random x and y coords within map bounds
    float judge;
    std::pair<float, float> random_point;
    std::random_device rd;
    //std::mt19937 gen(rd());
    std::uniform_real_distribution<> x(0, map_width_cells_);
    std::uniform_real_distribution<> y(0, map_height_cells_);
    std::uniform_real_distribution<> s(0, 100);

    judge = s(rd);
    if(judge > 40)
    {
       random_point.first = x(rd);
       random_point.second = y(rd);
    }
    else
    {
       random_point.first = x_goal_;
       random_point.second = y_goal_;
    }
    return random_point;
  }

  int srrtPlanner::FindPath(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal) {
    bool done = false;
    int goal_index = -1;
    current_iterations_ = 0;

    // Run until we either find the goal or reach the max iterations
    while (!done && current_iterations_ < max_iterations_) {
      ROS_DEBUG("Finding the path.");
      
      // get a random point on the map
      std::pair<float, float> random_point = srrtPlanner::GetRandomPoint();
      ROS_DEBUG("Random point: %.2f, %.2f", random_point.first,
                random_point.second);

      // find the closest known vertex to that point
      int closest_vertex = srrtPlanner::GetClosestVertex(random_point);
      ROS_DEBUG("Closest point %.5f, %.5f, index: %d.",
                vertex_list_.at(closest_vertex).get_location().first,
                vertex_list_.at(closest_vertex).get_location().second,
                vertex_list_.at(closest_vertex).get_index());

      // try to move from the closest known vertex towards the random point
      if (srrtPlanner::MoveTowardsPoint(closest_vertex, random_point)) {
        ROS_DEBUG("Moved, closest vertex: %d", closest_vertex);
        
        // If successful increase our iterations
        current_iterations_++;
        ROS_DEBUG("the current_interations is: %d",current_iterations_) ;
        // check if we've reached our goal
        int new_vertex = vertex_list_.back().get_index();
        done = ReachedGoal(new_vertex);

        if (done) {
          ROS_INFO("Hey, we reached our goal, index: %d", new_vertex);
          goal_index = new_vertex;
          break;
        }
      }

      if (current_iterations_ == max_iterations_)
        ROS_INFO("Max iterations reached, no plan found.");
    }
    return goal_index;
  }

  int srrtPlanner::GetClosestVertex(std::pair<float, float> random_point) {
    int closest = -1;

    // closest_distance will keep track of the closest distance we find
    float closest_distance = std::numeric_limits<float>::infinity();
    // current_distance will keep track of the distance of the current
    float current_distance = std::numeric_limits<float>::infinity();

    // iterate through the vertex list to find the closest
    for (srrt_planner::Vertex v : vertex_list_) {
      current_distance = GetDistance(v.get_location(), random_point);

      // If the current distance is closer than what was previously
      // saved, update
      if (current_distance < closest_distance) {
        ROS_DEBUG("Closest distance: %.5f, vertex: %d.",
                  current_distance, v.get_index());
        closest = v.get_index();
        closest_distance = current_distance;
      }
    }
    return closest;
  }

  float srrtPlanner::GetDistance(std::pair<float, float> start_point,
                                 std::pair<float, float> end_point) {
    // euclidean distance
    float distance = sqrt(pow((start_point.first - end_point.first), 2) + 
                                        pow((start_point.second - end_point.second), 2));

    ROS_DEBUG("Distance: %.5f", distance);
    return distance;
  }

  bool srrtPlanner::MoveTowardsPoint(int closest_vertex,
                                    std::pair<float, float> random_point) {
    
    float x_closest = vertex_list_.at(closest_vertex).get_location().first;
    float y_closest = vertex_list_.at(closest_vertex).get_location().second;

    // get the angle between the random point and our closest point (in rads)
    float theta = atan2(random_point.second - y_closest, random_point.first - x_closest);

    float new_x,new_y;
    double new_wx,new_wy;
    // proposed new point step_size_ from our closest vertex towards
    // the random point
    new_x = round(x_closest + step_size_ * cos(theta));
    new_y = round(y_closest + step_size_ * sin(theta));

    new_x = std::max(0 , (int)new_x); new_x = std::min(map_width_cells_-1 , (unsigned int)new_x);
    new_y = std::max(0 , (int)new_y); new_y = std::min(map_height_cells_-1, (unsigned int)new_y);
    
    costmap_->mapToWorld((unsigned int)new_x , (unsigned int)new_y, new_wx , new_wy);
    /*********/
  	p_.x=new_wx; 
		p_.y=new_wy; 
		p_.z=0.0;
    points_.points.push_back(p_);
    pub_.publish(points_) ;
    /*********/

    std::pair<float, float> proposed_point(new_x, new_y);
    std::pair<float, float> closest_point(x_closest, y_closest);
    
    // Check if the path between closest_vertex and the new point
    // is safe
    if (IsSafe(closest_point, proposed_point)) {
      // If safe, add new Vertex to the back of vertex_list_
      srrt_planner::Vertex new_vertex(new_x, new_y, vertex_list_.size(),
                                       closest_vertex);
      ROS_DEBUG("Added new vertex at: %.5f, %.5f, index: %d",
               new_x, new_y, new_vertex.get_index());
      addVertex(new_vertex);
      /************/
    	p_.x=new_wx; 
	  	p_.y=new_wy; 
		  p_.z=0.0;
	 	  line_.points.push_back(p_);
      double wx_closest,wy_closest;
      costmap_->mapToWorld((unsigned int)x_closest , (unsigned int)y_closest, wx_closest , wy_closest); 
	 	  p_.x=wx_closest; 
	  	p_.y=wy_closest; 
		  p_.z=0.0;
	 	  line_.points.push_back(p_);
      pub_.publish(line_); 
      /************/
      // Return true, that we moved towards the proposed point
      return true;
    }
    // Return false, move not made
    return false;
  }

  bool srrtPlanner::IsSafe(std::pair<float, float> start_point,
                           std::pair<float, float> end_point) {

    if (costmap_->getCost((unsigned int)end_point.first, (unsigned int)end_point.second) > lethal_cost_)
        return false;

    // check the path at intervals of delta for collision
    float theta = atan2(end_point.second - start_point.second,
                        end_point.first - start_point.first);
    float current_x = start_point.first;
    float current_y = start_point.second;

    ROS_DEBUG("Testing proposed point ");

    while (GetDistance(std::pair<float, float>(current_x, current_y),
                       end_point) > delta_) {
      // increment towards end point
      current_x = current_x + delta_ * cos(theta);
      current_y = current_y + delta_ * sin(theta);
      
         ROS_DEBUG("theta is: %.2f",theta);
         ROS_DEBUG("Testing proposed point %.5f, %.5f.", end_point.first,
                                                    end_point.second);
         ROS_DEBUG("current point %.2f, %.2f.", current_x,
                                                   current_y);
      unsigned int ad_x = ceil(current_x);
      unsigned int ad_y = ceil(current_y);

      ad_x = std::max(0 , (int)ad_x); ad_x = std::min(map_width_cells_-1 , ad_x);
      ad_y = std::max(0 , (int)ad_y); ad_y = std::min(map_height_cells_-1 , ad_y);
     
      // check for collision
      if (costmap_->getCost(ad_x, ad_y) > lethal_cost_)
        return false;

      ad_x = floor(current_x);
      ad_y = floor(current_y);

      ad_x = std::max(0 , (int)ad_x); ad_x = std::min(map_width_cells_-1 , ad_x);
      ad_y = std::max(0 , (int)ad_y); ad_y = std::min(map_height_cells_-1 , ad_y);
     
      // check for collision
      if (costmap_->getCost(ad_x, ad_y) > lethal_cost_)
        return false;

      ad_x = floor(current_x);
      ad_y = ceil(current_y);

      ad_x = std::max(0 , (int)ad_x); ad_x = std::min(map_width_cells_-1 , ad_x);
      ad_y = std::max(0 , (int)ad_y); ad_y = std::min(map_height_cells_-1 , ad_y);
     
      // check for collision
      if (costmap_->getCost(ad_x, ad_y) > lethal_cost_)
        return false;

      ad_x = ceil(current_x);
      ad_y = floor(current_y);

      ad_x = std::max(0 , (int)ad_x); ad_x = std::min(map_width_cells_-1 , ad_x);
      ad_y = std::max(0 , (int)ad_y); ad_y = std::min(map_height_cells_-1 , ad_y);
     
      // check for collision
      if (costmap_->getCost(ad_x, ad_y) > lethal_cost_)
        return false;
    }
    return true;
  }

  bool srrtPlanner::ReachedGoal(int new_vertex) {
    ROS_DEBUG("In ReachedGoal, vertex index: %d.", new_vertex);

    // save our goal and current location as pairs
    std::pair<float, float> goal(x_goal_, y_goal_);
    std::pair<float, float> current_location;

    current_location.first =
      vertex_list_.at(new_vertex).get_location().first;
    current_location.second =
      vertex_list_.at(new_vertex).get_location().second;

    ROS_DEBUG("cx: %.5f, cy: %.5f, gx: %.5f, gy: %.5f",
              current_location.first,
              current_location.second,
              goal.first,
              goal.second);

    // Check distance between current point and goal, if distance is less
    // than goal_radius_ return true, otherwise return false
    float distance = GetDistance(current_location, goal);
    ROS_DEBUG("Distance to goal: %.5f", distance);

    if (distance <= goal_radius_)
      return true;
    else
      return false;
  }

  std::vector<geometry_msgs::PoseStamped>
    srrtPlanner::BuildPlan(int goal_index,
                           const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal) {
      ROS_DEBUG("Building the plan.");

      // reset our current iterations
      current_iterations_ = 0;

      // The plan we'll be adding to and returning
      std::vector<geometry_msgs::PoseStamped> plan;

      // no plan found
      if (goal_index == -1)
        return plan;

      // The list of vertex indices we pass through to get to the goal
      std::deque<int> index_path;
      int current_index = goal_index;
      while (current_index > 0) {
        index_path.push_front(current_index);
        current_index = vertex_list_.at(current_index).get_parent();
      }
      index_path.push_front(0);

      // build the plan back up in PoseStamped messages
      for (int i : index_path) {
        if (i == 0) {
          plan.push_back(start);
        } else {
          geometry_msgs::PoseStamped mpos,pos;
          
          mpos.pose.position.x = vertex_list_.at(i).get_location().first;
          mpos.pose.position.y = vertex_list_.at(i).get_location().second;
          mpos.pose.position.z = 0.0;

          costmap_->mapToWorld((unsigned int)mpos.pose.position.x , (unsigned int)mpos.pose.position.y, 
          pos.pose.position.x , pos.pose.position.y); 
          pos.pose.position.z = 0.0;
          pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
          plan.push_back(pos);
        }
      }
      plan.push_back(goal);
      return plan;
  }


};

