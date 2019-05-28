#include <dstarl_planner/dstarl_planner.h>
#include <pluginlib/class_list_macros.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dstarl_planner::dstarlPlanner, nav_core::BaseGlobalPlanner)

namespace dstarl_planner {

  dstarlPlanner::dstarlPlanner()
  : costmap_ros_(NULL), initialized_(false){
    window_size_ = 80;
    lethal_cost_ = 253;
    neutral_cost_ = 50;

    old_goalx_ = -1;
    old_goaly_ = -1;
      //initialize the params

    maxSteps = 80000;  // node expansions before we give up
    C1       = 1;      // cost of an unseen cell

  }

  dstarlPlanner::dstarlPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void dstarlPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      frame_id_ = costmap_ros_->getGlobalFrameID();
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle node("~/dstarl_planner");
      node_handle_ = node;

      plan_pub_ = node.advertise<nav_msgs::Path>("plan", 1);
      obpub_ = node.advertise<visualization_msgs::Marker>("dstar_Ob", 100);

      // Get obstacles in the costmap
      map_width_cells_ = costmap_-> getSizeInCellsX();
      map_height_cells_ = costmap_-> getSizeInCellsY();

      //orientation_filter_ = new global_planner::OrientationFilter();

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

    ROS_INFO("Start: %.2f, %.2f", start.pose.position.x,
              start.pose.position.y);
    ROS_INFO("Goal: %.2f, %.2f", goal.pose.position.x,
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
    ROS_INFO("Goal: %d, %d", x_goal_,
              y_goal_);

    outlineMap(costmap_->getCharMap(), map_width_cells_, map_height_cells_, costmap_2d::LETHAL_OBSTACLE);

    ROS_DEBUG("Going into FindPath");

    if(old_goalx_ != wx || old_goaly_ != wy){
      
      plan.clear();
      ROS_INFO("new goal set.");

      init((int)x_origin_, (int)y_origin_, (int)x_goal_, (int)y_goal_);
      Updatemap();

      old_goalx_ = goal.pose.position.x;
      old_goaly_ = goal.pose.position.y;

      replan();

    }
    else{

      ROS_INFO("update the cells");
      Updatemap();

      if(!CheckPlan()){
        ROS_INFO("replan!");
        updateStart((int)x_origin_, (int)y_origin_);
        replan();
      }

      
    }
   
    if (getPath(plan,start,goal)) {
      //orientation_filter_->processPath(start, plan);
      publishPlan(plan);
      ROS_INFO("A path was found.");
      return true;
    } 
    else {
      ROS_WARN("No path was found.");
      return false;
    }
  }

  void dstarlPlanner::init(int sX, int sY, int gX, int gY) {
  
    cellHash.clear();
    path.clear();
    visited.clear();
    openHash.clear();
    while(!openList.empty()) openList.pop();

    k_m = 0;
  
    s_start.x = sX;
    s_start.y = sY;
    s_goal.x  = gX;
    s_goal.y  = gY;

    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    tmp.cost = C1;

    cellHash[s_goal] = tmp;

    tmp.g = tmp.rhs = heuristic(s_start,s_goal);
    tmp.cost = C1;
    cellHash[s_start] = tmp;
    s_start = calculateKey(s_start);

    s_last = s_start;

  }

  /* float Dstar::keyHashCode(state u) 
   * -------------------------- 
   * Returns the key hash code for the state u, this is used to compare
   * a state that have been updated
   */
  float dstarlPlanner::keyHashCode(state u) {

    return (float)(u.k.first + 1193*u.k.second);

  }

  /* bool Dstar::isValid(state u) 
   * --------------------------
   * Returns true if state u is on the open list or not by checking if
   * it is in the hash table.
   */
  bool dstarlPlanner::isValid(state u) {
  
    ds_oh::iterator cur = openHash.find(u);
    if (cur == openHash.end()) return false;
    if (!close(keyHashCode(u), cur->second)) return false;
    return true;
  
  }

  /* void Dstar::getPath() 
   * --------------------------
   * Returns the path created by replan()
   */
  bool dstarlPlanner::getPath(vector<geometry_msgs::PoseStamped>& plan, const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal) {
    
    list<state>::iterator i;
    geometry_msgs::PoseStamped pose;
    ros::Time plan_time = ros::Time::now();

    pose.header.stamp = plan_time;
    pose.header.frame_id = frame_id_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    //plan.push_back(start);

    for (i = path.begin(); i != path.end(); i++) {

      costmap_->mapToWorld((unsigned int)i->x, (unsigned int)i->y, pose.pose.position.x, pose.pose.position.y);

      plan.push_back(pose);
    }
    plan.push_back(goal);

    return !plan.empty();
  }

  /* bool Dstar::occupied(state u)
   * --------------------------
   * returns true if the cell is occupied (non-traversable), false
   * otherwise. non-traversable are marked with a cost < 0.
   */
  bool dstarlPlanner::occupied(state u) {
  
    ds_ch::iterator cur = cellHash.find(u);
  
    if (cur == cellHash.end()) return false;
    return (cur->second.cost < 0);
  }


  /* void Dstar::makeNewCell(state u)
   * --------------------------
   * Checks if a cell is in the hash table, if not it adds it in.
   */
  void dstarlPlanner::makeNewCell(state u) {
  
    if (cellHash.find(u) != cellHash.end()) return;
    
    cellInfo tmp;
    tmp.g       = tmp.rhs = heuristic(u,s_goal);
    tmp.cost    = C1;
    cellHash[u] = tmp;
  
  }

  /* double Dstar::getG(state u)
   * --------------------------
   * Returns the G value for state u.
   */
  double dstarlPlanner::getG(state u) {

    if (cellHash.find(u) == cellHash.end()) 
      return heuristic(u,s_goal);
    return cellHash[u].g;
  
  }

  /* double Dstar::getRHS(state u)
   * --------------------------
   * Returns the rhs value for state u.
   */
  double dstarlPlanner::getRHS(state u) {

    if (u == s_goal) return 0;  

    if (cellHash.find(u) == cellHash.end()) 
      return heuristic(u,s_goal);
    return cellHash[u].rhs;
    
  }

  /* void Dstar::setG(state u, double g)
   * --------------------------
   * Sets the G value for state u
  */
  void dstarlPlanner::setG(state u, double g) {
  
   makeNewCell(u);  
   cellHash[u].g = g; 
   
  }

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
double dstarlPlanner::setRHS(state u, double rhs) {
  
  makeNewCell(u);
  cellHash[u].rhs = rhs;
  visited.push_back(u);
  //cout<<"add vertex!";

}

/* double Dstar::eightCondist(state a, state b) 
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 */
double dstarlPlanner::eightCondist(state a, state b) {
  double temp;
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return (0.414*min + max);
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles. 
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int dstarlPlanner::computeShortestPath() {
  
  list<state> s;
  list<state>::iterator i;
  ds_oh::iterator iter1;
  double newx, newy;

  vector<int> ob_index = costmap_ros_->getobIndex();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  marker.ns = "dstar";
  marker.id = 0;
  marker.type = marker.POINTS;
  marker.action = marker.ADD;
  marker.lifetime = ros::Duration();
  //cout<<"1";

  if (openList.empty()) return 1;
  //cout<<"13";
  int k=0;
  while ((!openList.empty()) && 
         (openList.top() < (s_start = calculateKey(s_start))) || 
         (getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }
    
    ROS_INFO("1");
    state u;

    marker.points.clear();
    for(iter1=openHash.begin(); iter1 != openHash.end(); iter1++) {
      costmap_->mapToWorld(iter1->first.x, iter1->first.y, newx, newy);
      geometry_msgs::Point point;
      point.x = newx;
      point.y = newy;
      point.z = 0;
      marker.points.push_back(point);
      
    }
  
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.color.a = 0.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    obpub_.publish( marker );

    bool test = (getRHS(s_start) != getG(s_start));
    
    // lazy remove
    while(1) { 
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();
    
      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }
    
    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;
    //ROS_INFO("(%d,%d) Top key is %lf",k_old.x,k_old.y,k_old.k.first);

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u);
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        //cout<<"2";
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool Dstar::close(double x, double y) 
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool dstarlPlanner::close(double x, double y) {
    
  if (isinf(x) && isinf(y)) return true;
  return (fabs(x-y) < 0.00001);
  
}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void dstarlPlanner::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;
 
  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  if (!close(getG(u),getRHS(u))) insert(u);
  
}

/* void Dstar::insert(state u) 
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void dstarlPlanner::insert(state u) {
  
  ds_oh::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  if ((cur != openHash.end()) && (close(csum,cur->second))) return;
  
  openHash[u] = csum;
  openList.push(u);
} 

/* void Dstar::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void dstarlPlanner::remove(state u) {
  
  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double Dstar::trueDist(state a, state b) 
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double dstarlPlanner::trueDist(state a, state b) {
  
  float x = a.x-b.x;
  float y = a.y-b.y;

  return sqrt(x*x + y*y);
  
}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double dstarlPlanner::heuristic(state a, state b) {
  return eightCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state dstarlPlanner::calculateKey(state u) {
  
  double val = fmin(getRHS(u),getG(u));
  
  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
double dstarlPlanner::cost(state a, state b) {

  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = 1.414;

  if (cellHash.count(a) == 0) 
    return scale*C1 ;
  return scale*cellHash[a].cost;

/*
  return (scale*C1 + costmap_->getCost((unsigned int)a.x,(unsigned int)b.y) + neutral_cost_);
  */
}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void dstarlPlanner::updateCell(int x, int y, double val) {
  
   state u;
  
  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;
 
  makeNewCell(u); 
  cellHash[u].cost = val;

  updateVertex(u);
}

/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors. 
 */
void dstarlPlanner::getSucc(state u,list<state> &s) {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  u.x += 1;
  s.push_front(u);
  u.y += 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);

}

/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void dstarlPlanner::getPred(state u,list<state> &s) {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  
}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void dstarlPlanner::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;
  
  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;
  
}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start). 
 */
bool dstarlPlanner::replan() {

  path.clear();
  visited.clear();

  //cout<<"replan!";

  int res = computeShortestPath();
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start; 
  flag = 0;

  if (isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  
  while(cur != s_goal) {

    //ROS_INFO("building path!");
    flag ++;
    path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin;
    double val,val2;
    state smin;
    
    for (i=n.begin(); i!=n.end(); i++) {
      
      
      if (occupied(*i)) continue;
      val  = cost(cur,*i);
      val2 = trueDist(*i,s_goal) + trueDist(s_start,*i) ;
               //+ costmap_->getCost((unsigned int)i->x,(unsigned int)i->y) + neutral_cost_; // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    cur = smin;
    if(flag < 20)
        ROS_INFO("(%d,%d,%lf,%lf)",smin.x,smin.y,val,val2);
  }
  path.push_back(s_goal);
  return true;
}

void dstarlPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++){
      *pc++ = value;
      updateCell(i, 0, -1);
    }
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++){
      *pc++ = value;
      updateCell(i, ny - 1, -1);
    }      
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx){
      *pc = value;
      updateCell(0, i, -1);     
    }    
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx){
      *pc = value;
      updateCell(nx - 1, i, -1);    
    }
        
}

void dstarlPlanner::Updatemap(){
/*
  int x,y;
  double newx, newy;

  vector<int> ob_index = costmap_ros_->getobIndex();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  marker.ns = "dstar";
  marker.id = 0;
  marker.type = marker.POINTS;
  marker.action = marker.ADD;
  marker.lifetime = ros::Duration();

  for(int i = 0; i < ob_index.size(); i++)
  {
    x = ob_index[i]%map_width_cells_;
    y = ob_index[i]/map_height_cells_;
    ROS_INFO("(%d,%d) obstacle!",x,y); 
    updateCell(x,y,-1);
    
    costmap_->mapToWorld((unsigned int)x, (unsigned int)y, newx, newy);
    geometry_msgs::Point point;
    point.x = newx;
    point.y = newy;
    point.z = 0;
    marker.points.push_back(point);
  }
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.color.a = 0.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  obpub_.publish( marker );
  */
  
  for(int j = y_origin_ - window_size_/2; j < y_origin_ + window_size_/2; j++)
    for(int i = x_origin_ - window_size_/2; i < x_origin_ + window_size_/2; i++)
      if(costmap_->getCost((unsigned int)i,(unsigned int)j) > lethal_cost_) {
        updateCell(i,j,-1);
        //ROS_INFO("(%d,%d) obstacle!",i,j); 
      }
}

bool dstarlPlanner::CheckPlan(){
  
  list<state>::iterator i;
  double dist_bound = window_size_*window_size_/4;
  for (i = path.begin(); i != path.end(); i++){
    double diff_x = (int)x_origin_ - i->x;
    double diff_y = (int)y_origin_ - i->y;
    if(diff_x*diff_x + diff_y*diff_y > dist_bound) continue;
    else
      if(costmap_->getCost((unsigned int)i->x,(unsigned int)i->y) >= lethal_cost_) return false;
  }

  return true;
}

void dstarlPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

}
