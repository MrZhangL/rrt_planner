#include <dstarl_planner/dstarl_planner.h>
#include <pluginlib/class_list_macros.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dstarl_planner::dstarlPlanner, nav_core::BaseGlobalPlanner)

namespace dstarl_planner {

  dstarlPlanner::dstarlPlanner()
  : costmap_ros_(NULL), initialized_(false){
    window_size_ = 100;
    lethal_cost_ = 250;

    flag_ = false;
      //initialize the params

  }

  dstarlPlanner::dstarlPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void dstarlPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle node("~/dstarl_planner");
      node_handle_ = node;

      plan_pub_ = node.advertise<nav_msgs::Path>("plan", 1);

      // Get obstacles in the costmap
      map_width_cells_ = costmap_-> getSizeInCellsX();
      map_height_cells_ = costmap_-> getSizeInCellsY();

      init();

      // Display info message
      ROS_INFO("D* Lite planner initialized successfully.");
      initialized_ = true;
    }
    else
      ROS_WARN("D* Lite planner has already been initialized... doing nothing");

  }

   bool dstarlPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

      // Check if we've initialized, if not error
    if (!initialized_) {
      ROS_ERROR("D* lite planner has not been initialized, please call "
                "initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Start: %.2f, %.2f", start.pose.position.x,
              start.pose.position.y);
    ROS_DEBUG("Goal: %.2f, %.2f", goal.pose.position.x,
              goal.pose.position.y);

    // Make sure that the goal header frame is correct
    // Goals are set within rviz
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(),
                goal.header.frame_id.c_str());
      return false;
    }

    // reset origin and goal
    float wx , wy;

    wx = start.pose.position.x;
    wy = start.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, x_origin_, y_origin_)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, x_goal_, y_goal_)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

    ROS_INFO("Start: %d, %d", x_origin_,
              y_origin_);
    ROS_DEBUG("Goal: %d, %d", x_goal_,
              y_goal_);

    ROS_DEBUG("Going into FindPath");

    if(old_goalx_ != wx || old_goaly_ != wy){
      

      init();
      SetGoal(x_goal_, y_goal_);
      s_start_.x = x_origin_;
      s_start_.y = y_origin_;
      s_last_ = s_start_;

      ROS_INFO("new goal set.");

      plan.clear();
      old_goalx_ = goal.pose.position.x;
      old_goaly_ = goal.pose.position.y;

      computeShortestPath();
      plan = BuildPlan(start, goal);
    }
    else{

      ROS_INFO("update the environment.");

      std::vector<int> Changelist;

      s_start_.x = x_origin_;
      s_start_.y = y_origin_;

      if(SearchChange(s_start_.x, s_start_.y, Changelist)){
        plan.clear();
        k_m_ += heuristic(s_start_, s_last_);
        s_last_ = s_start_;
        std::vector<Cell> s;

        for(int i : Changelist){
          getNeighbor(Mapcell_.at(i),s);
          for(Cell cell : s) updateVertex(cell);
        }

        computeShortestPath();
        plan = BuildPlan(start, goal);
      }
    }
   
    if (plan.size() > 2) {
      publishPlan(plan);
      ROS_INFO("A path was found.");
      return true;
    } 
    else {
      ROS_WARN("No path was found.");
      return false;
    }
  }

  void dstarlPlanner::init() {
  
    Mapcell_.clear();
    queue_.clear();

    old_goalx_ = INFINITY;
    old_goaly_ = INFINITY;

    k_m_ = 0;
    Cell cell;

    //Initialize all the cells.
    for(unsigned int j = 0; j < map_height_cells_; j++){
      for(unsigned int i = 0; i < map_width_cells_; i++){

        cell.x = i;
        cell.y = j;
        cell.index = j*map_width_cells_ + i;
        cell.g = cell.rhs = cell.k.first = cell.k.second = INFINITY;
        cell.Inqueue = false;

        map_value_.push_back(costmap_->getCost(i, j));

        if(costmap_->getCost(i, j) > lethal_cost_ && 
                          !(costmap_->getCost(i, j)==costmap_2d::NO_INFORMATION)){
          //ROS_INFO("set obstacle");
          cell.obstacle = true;
          map_value_.push_back(cell.obstacle);
        }
        else{
          cell.obstacle = false;
          map_value_.push_back(cell.obstacle);
        }

        Mapcell_.push_back(cell);
      }
    }

  }
  /* double dstarlPlanner::heuristic(state a, state b)
   * --------------------------
   * Pretty self explanitory, the heuristic we use is the 8-way distance
   * scaled by a constant C1 (should be set to <= min cost).
   */
  float dstarlPlanner::heuristic(Cell a, Cell b) {
      
      float temp;
      unsigned int temp2;
      unsigned int min = abs(a.x - b.x);
      unsigned int max = abs(a.y - b.y);
      if (min > max) {
         temp2 = min;
         min = max;
         max = temp2;
      }

      temp = (M_SQRT2-1.0)*(float)min + (float)max;

      return temp;

  }
  Cell dstarlPlanner::calculateKey(Cell u) {
  
    float val = fminf(u.rhs,u.g);

    //ROS_INFO("the val rhs is: %f",u.rhs);
  
    u.k.first  = val + heuristic(u,s_start_) + k_m_;
    u.k.second = val;

    return u;

  }
  /* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
 void dstarlPlanner::updateVertex(Cell u) {

    std::vector<Cell> s;
    
 
    if (u.index != s_goal_.index) {
      getNeighbor(u,s);
      //s = getSucc(s);

      //ROS_INFO("the neighbor size is: %ld", s.size());
      float tmp2, tmp = INFINITY;

      for (Cell cell: s) {
        tmp2 = cell.g + cost(u,cell);
        if (tmp2 < tmp) tmp = tmp2;
      }
      Mapcell_.at(u.index).rhs = tmp;
      //if(!flag_){
        //ROS_INFO("change rhs-value to %f",tmp);
        //flag_ = true;
      //}
      
    }

    if(u.Inqueue)
    {
      for(unsigned i = 0; i < queue_.size(); i++){
        if(u.index == queue_.at(i).index){
          queue_.erase(queue_.begin() + i);
          std::make_heap (queue_.begin(),queue_.end(),greater1());
          Mapcell_.at(u.index).Inqueue = false;
          break;
        }
      }
    }

    if(Mapcell_.at(u.index).g != Mapcell_.at(u.index).rhs) {
      Insert(Mapcell_.at(u.index));
      //ROS_INFO("Insert");
    }
  }
  /* void dstarlPlanner::getNeighbor(Cell u,std::vector<Cell> &s)
   * --------------------------
   * Returns a list of Neighbor cell for Cell u, since this is an
   * 8-way graph this list contains all of a cells neighbours. 
   */
  void dstarlPlanner::getNeighbor(Cell u,std::vector<Cell> &s) {
  
    s.clear();
    
    if( u.x < map_width_cells_ - 1 && u.x > 0 && u.y < map_height_cells_ - 1 && u.y > 0){
      u.index += 1;                 s.push_back(Mapcell_.at(u.index));
      u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
      u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
      u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      u.index += 1;                 s.push_back(Mapcell_.at(u.index));
      u.index += 1;;                s.push_back(Mapcell_.at(u.index));
    }

    else if(u.x = map_width_cells_ - 1){
      if(u.y < map_height_cells_ - 1 && u.y > 0){
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index += 1;                 s.push_back(Mapcell_.at(u.index));
      }
      else if(u.y = 0){
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      }
      else{
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      }
    }

    else if(u.x = 0){
      if(u.y < map_height_cells_ - 1 && u.y > 0){
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index += 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
      }
      else if(u.y = 0){
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index += 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      }
      else{
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index += 1;                 s.push_back(Mapcell_.at(u.index));
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      }
    }

    else{
      if(u.y = 0){
        u.index += 1;                 s.push_back(Mapcell_.at(u.index));
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      }
      else{
        u.index += 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= map_width_cells_;  s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index -= 1;                 s.push_back(Mapcell_.at(u.index));
        u.index += map_width_cells_;  s.push_back(Mapcell_.at(u.index));
      }
    }
  }

  std::vector<Cell> dstarlPlanner::getPred(std::vector<Cell> &neighbor){

    std::vector<Cell> s;
    s.clear();

    for(Cell cell : neighbor){
      if(!cell.Inqueue) s.push_back(cell);
    }

    return s;
  }

  std::vector<Cell> dstarlPlanner::getSucc(std::vector<Cell> &neighbor){

    std::vector<Cell> s;
    s.clear();

    for(Cell cell : neighbor){
      if(cell.Inqueue) s.push_back(cell);
    }

    return s;
  }
  /* void dstarlPlanner::SetGoal(unsigned int x, unsigned int y)
   * --------------------------
   * Initialize the goal's rhs-value, add the goal into the priority queue. 
   */
  void dstarlPlanner::SetGoal(unsigned int x, unsigned int y){

    unsigned int index = y*map_width_cells_ + x;
    Mapcell_.at(index).rhs = 0;
    s_goal_ = Mapcell_.at(index);

    Insert(s_goal_);
  }

  void dstarlPlanner::Insert(Cell u){

    if(Mapcell_.at(u.index).obstacle) return;

    Mapcell_.at(u.index) = calculateKey(Mapcell_.at(u.index));
    
    queue_.push_back(Mapcell_.at(u.index));
    std::push_heap (queue_.begin(),queue_.end(),greater1());
    Mapcell_.at(u.index).Inqueue = true;
    
  }

  float dstarlPlanner::cost(Cell a, Cell b) {
    
    if(!a.obstacle && !b.obstacle){
      unsigned int xd = abs(a.x-b.x);
      unsigned int yd = abs(a.y-b.y);
      float scale = 1;

      if (xd+yd>1) scale = M_SQRT2;
      return scale;
    }
    
    else 
      return INFINITY;

  }

  void dstarlPlanner::computeShortestPath(){

    unsigned int start_index = s_start_.y*map_width_cells_ + s_start_.x;
    Mapcell_.at(start_index) = calculateKey(Mapcell_.at(start_index));
    
    while(queue_.front().k.first < Mapcell_.at(start_index).k.first || 
                                  Mapcell_.at(start_index).g != Mapcell_.at(start_index).rhs)
    {
      
      float k_old = queue_.front().k.first;
      //ROS_INFO("the smallest key is: %f",k_old);
      unsigned int index = queue_.front().index;
      std::vector<Cell> s;

      std::pop_heap (queue_.begin(),queue_.end(),greater1()); 
      queue_.pop_back();
      Mapcell_.at(index).Inqueue = false;

      Mapcell_.at(index) = calculateKey(Mapcell_.at(index));

      if(k_old < Mapcell_.at(index).k.first) Insert(Mapcell_.at(index));

      else if(Mapcell_.at(index).g > Mapcell_.at(index).rhs){
        Mapcell_.at(index).g = Mapcell_.at(index).rhs;
        getNeighbor(Mapcell_.at(index),s);
        s = getPred(s);

        for(Cell cell : s){
          updateVertex(cell);
        }

        //ROS_INFO("the queue size is: %ld", queue_.size());
      }

      else{
        Mapcell_.at(index).g = INFINITY;
        getNeighbor(Mapcell_.at(index),s);
        s = getPred(s);

        updateVertex(Mapcell_.at(index));
        for(Cell cell : s){
          updateVertex(cell);
        }
      }
    }
    ROS_INFO("compute the shortest path already.");
  }

   bool dstarlPlanner::SearchChange(unsigned int x, unsigned int y, std::vector<int> Changelist){
     
     Changelist.clear();
     bool flag = false;

     for(unsigned int i = x - window_size_/2; i < x + window_size_/2; i++){
       if(i >= 0 && i < map_width_cells_){
         for(unsigned int j = y - window_size_/2; j < y + window_size_/2; j++){
           if(j >= 0 && j < map_height_cells_){
             unsigned int index = j*map_width_cells_ + i;

             if(costmap_->getCost(i, j) > lethal_cost_ && 
                           !(costmap_->getCost(i, j)==costmap_2d::NO_INFORMATION)){
               Mapcell_.at(index).obstacle = true;
               Mapcell_.at(index).g = INFINITY;
            
             }

             else Mapcell_.at(index).obstacle = false;
                           
             if(map_value_.at(index) != Mapcell_.at(index).obstacle){

                 map_value_.at(index) = Mapcell_.at(index).obstacle;
                 Changelist.push_back(index);
                 flag = true;
                            
             }
           }
         }
       }
     }

     return flag;
   }
  
  std::vector<geometry_msgs::PoseStamped> 
      dstarlPlanner::BuildPlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal){

    ROS_INFO("Building the plan.");

    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::PoseStamped pos;
    unsigned int x, y;

    plan.push_back(start);

    std::pair<unsigned int, unsigned int> ming_coordinate;

    ming_coordinate = getMin_gcell(x_origin_, y_origin_);
    
    x = ming_coordinate.first;
    y = ming_coordinate.second;

    while(x != x_goal_ && y != y_goal_){

      ming_coordinate = getMin_gcell(x, y);
      x = ming_coordinate.first;
      y = ming_coordinate.second;

      costmap_->mapToWorld(x, y, pos.pose.position.x, pos.pose.position.y); 
      pos.pose.position.z = 0.0;
      pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      plan.push_back(pos);

    }

    return plan;
  }

  std::pair<unsigned int, unsigned int> 
             dstarlPlanner::getMin_gcell(unsigned int x, unsigned int y){
    
    Cell u,ming_cell;
    std::vector<Cell> s;
    std::pair<unsigned int, unsigned int> ming_coordinate;
    unsigned int index = y*map_width_cells_ + x;
    float tmp,tmp2 = INFINITY;

    u = Mapcell_.at(index);

    getNeighbor(u,s);

    for(Cell cell : s){
      tmp = cell.g;
      if(tmp < tmp2){
        tmp2 = tmp;
        ming_cell = cell;
      }
    }

    if(tmp2 == INFINITY) ROS_INFO("no valid path!");
    
    ming_coordinate.first = ming_cell.x;
    ming_coordinate.second = ming_cell.y;

    return ming_coordinate;
  }

  void dstarlPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    
    ROS_INFO("building path");
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "/robot_1/map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }
  
};
