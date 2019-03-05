#ifndef DSTARL_PLANNER_H
#define DSTARL_PLANNER_H

#include <stdlib.h>
#include <cmath> 
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_core/base_global_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>



namespace dstarl_planner{

    struct Cell {

      unsigned int x;
      unsigned int y;
      unsigned int index;
      float g;
      float rhs;
      bool Inqueue;
      bool obstacle;
      std::pair<float,float> k;

    };

    struct greater1 {
        bool operator()(const Cell& a, const Cell& b) const {
            return a.k.first > b.k.first;
        }
    };

    class dstarlPlanner : public nav_core::BaseGlobalPlanner{
       
      public:

        /**
         * @brief Default constructor for the ros wrapper
         */
        dstarlPlanner();

        /**
         * @brief Constructs the ros wrapper
         * @param name The name to give this instance of the dstar planner
         * @param tf A pointer to a transform listener
         * @param costmap The cost map to use for assigning costs to trajectories
         */
        dstarlPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief  Initialization function for the NavFnROS object
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

       void init();

       float heuristic(Cell a, Cell b);

       Cell calculateKey(Cell u);

       void updateVertex(Cell u);

       void getNeighbor(Cell u,std::vector<Cell> &s);

       std::vector<Cell> getPred(std::vector<Cell> &neighbor);

       std::vector<Cell> getSucc(std::vector<Cell> &neighbor);

       void SetGoal(unsigned int x, unsigned int y);

       void Insert(Cell u);

       float cost(Cell a, Cell b);

       void computeShortestPath();

       bool SearchChange(unsigned int x, unsigned int y, std::vector<int> Changelist);

       std::vector<geometry_msgs::PoseStamped> BuildPlan(const geometry_msgs::PoseStamped& start,
                                                         const geometry_msgs::PoseStamped& goal);
       
       std::pair<unsigned int, unsigned int> getMin_gcell(unsigned int x, unsigned int y);
        
        
     protected:

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    private:

       ros::NodeHandle node_handle_;

       /**
        * @brief the publisher to pubish the path
        */
        ros::Publisher plan_pub_;
    
       /**
        * @brief The ROS wrapper for the costmap the controller will use
        */
        costmap_2d::Costmap2DROS* costmap_ros_;

       /**
        * @brief The ROS wrapper for the costmap the controller will use
        */
        costmap_2d::Costmap2D* costmap_;

       /**
        * @brief width and height of 2d map in cells
        */
        unsigned int map_width_cells_, map_height_cells_;

       /**
        * @brief Check if the global planner is initialized
        */
        bool initialized_;
      
       /**
        * @brief x,y coordinate of robot origin and goal
        */
        unsigned int x_origin_, y_origin_, x_goal_, y_goal_;

        std::vector<Cell> Mapcell_;
        std::vector<Cell> queue_;
        std::vector<bool> map_value_;

        unsigned char lethal_cost_;
        unsigned int window_size_;

        float old_goalx_, old_goaly_;
        bool goalchange_;

        Cell s_start_, s_goal_, s_last_;

        bool flag_;

        float k_m_;

    };
};
#endif