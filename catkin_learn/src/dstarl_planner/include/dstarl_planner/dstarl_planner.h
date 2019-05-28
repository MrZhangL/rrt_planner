#ifndef DSTARL_PLANNER_H
#define DSTARL_PLANNER_H

#include <cmath>
#include <stack>
#include <queue>
#include <list>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_core/base_global_planner.h>
#include <global_planner/orientation_filter.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace dstarl_planner{

class state {
 public:
  int x;
  int y;
  pair<double,double> k;
  
  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }
  
  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }
  
  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;
  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;
  }
  

  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;
  }
   
};
/*
struct ipoint2 {
  int x,y;
};
*/
struct cellInfo {

  double g;
  double rhs;
  double cost;

};

class state_hash {
 public:
  size_t operator()(const state &s) const {
    return s.x + 34245*s.y;
  }
};


typedef priority_queue<state, vector<state>, greater<state> > ds_pq;
typedef unordered_map<state,cellInfo, state_hash, equal_to<state> > ds_ch;
typedef unordered_map<state, float, state_hash, equal_to<state> > ds_oh;


class dstarlPlanner : public nav_core::BaseGlobalPlanner{

 public:
    
  dstarlPlanner();

  dstarlPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

  void   init(int sX, int sY, int gX, int gY);

  void   updateCell(int x, int y, double val);

  void   updateStart(int x, int y);

  //void   updateGoal(int x, int y);

  bool   replan();

  void   Updatemap();

  bool   getPath(vector<geometry_msgs::PoseStamped>& plan, const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal);

  void   publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
  
 private:
  
  list<state> path;
  list<state> visited;

  double C1;
  double k_m;
  int map_width_cells_, map_height_cells_;
  state s_start, s_goal, s_last;
  int maxSteps,flag;  
  float old_goalx_, old_goaly_;
  std::string frame_id_;
  unsigned int x_origin_, y_origin_, x_goal_, y_goal_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  bool initialized_;
  unsigned char lethal_cost_, neutral_cost_;
  int window_size_;
  ros::NodeHandle node_handle_;
  ros::Publisher plan_pub_,obpub_;

  ds_pq openList;
  ds_ch cellHash;
  ds_oh openHash;

  bool   close(double x, double y);
  void   makeNewCell(state u);
  double getG(state u);
  double getRHS(state u);
  void   setG(state u, double g);
  double setRHS(state u, double rhs);
  double eightCondist(state a, state b);
  int    computeShortestPath();
  void   updateVertex(state u);
  void   insert(state u);
  void   remove(state u);
  double trueDist(state a, state b);
  double heuristic(state a, state b);
  state  calculateKey(state u);
  void   getSucc(state u, list<state> &s);
  void   getPred(state u, list<state> &s);
  double cost(state a, state b); 
  bool   occupied(state u);
  bool   isValid(state u);
  float  keyHashCode(state u);
  void   outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
  bool   CheckPlan();
};
}
#endif