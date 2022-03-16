#ifndef JPS_PLANNER_JPS_PLAN_H_
#define JPS_PLANNER_JPS_PLAN_H_

#include <iostream>
// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>

// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>

#include <vector>


namespace jps_planner{
    class Node{
        public:
            Node(int index, float cost, float gcost){
                this->index = index;
                this->cost = cost;
                this->gcost = gcost;
            }
            int index;
            float cost, gcost;
    };
     struct greater1
     {
         bool operator()(const Node& a, const Node& b){
             return a.cost > b.cost;
         }
     };

     class jpsPlanner : public nav_core::BaseGlobalPlanner
     {
     private:
         /* data */
     public:
         jpsPlanner(/* args */);
         jpsPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

         bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

         void getPlan(int start_index, int goal_index, std::vector<int> camefrom, std::vector<geometry_msgs::PoseStamped>& plan);

         void getJumpNode(int index, std::vector<bool>& visted, 
            std::vector<int>& cameFrom, int goal_index, float current_gcost);

         std::vector<unsigned int> getForceNeighbourCell(int current_x, int current_y, std::vector<bool>& visted, int diff_x, int diff_y);

         inline int getOri(int diff){
             if(diff == 0) return 0;
             return diff > 0 ? 1 : -1;
         };

         bool searchJPS(int pre_jps_index, int current_x, int current_y,std::vector<bool>& visted, 
            std::vector<int>& cameFrom, int dx, int dy, int goal_index, float gcost);

         inline bool isCollision(int x, int y);

         float getHCost(int current_index, int goal_index);
         float getGCost(int pre_index, int current_index);

         void publishPlan(std::vector<geometry_msgs::PoseStamped> path);

         bool addJpsNode(float gcost, int current_index, int goal_index, int pre_index, std::vector<bool>& visted, std::vector<int>& cameFrom);


     public:
        int width, height, map_size;
        bool initialized_;
        ros::Publisher plan_pub_;
        std::string frame_id_;
        unsigned char *costs;
        costmap_2d::Costmap2DROS* costmap_ros_;
	    costmap_2d::Costmap2D* costmap_;
        std::vector<std::vector<bool> > judgeMap;
        std::vector<Node> openList;
     };   
     
};
#endif
