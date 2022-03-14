#include "jps_plan.h"
#include <algorithm>
#include <time.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(jps_planner::jpsPlanner, nav_core::BaseGlobalPlanner)

namespace jps_planner{

    jpsPlanner::jpsPlanner(){}
    jpsPlanner::jpsPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }


    void jpsPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            std::cout<<"map_size: " << map_size;
            map_size = width * height;
            std::cout<<" map_size: " << map_size<<std::endl;
            // std::vector<bool> j_map(width, false);
            // judgeMap.resize(height, j_map); //用于是否是否为障碍物节点

            // for(int i = 0; i < height; i++){
            //     for(int j = 0; j < width; j++){
            //         unsigned int cost = static_cast<int>(costmap_->getCost(j,i));
            //         if(cost > 250) judgeMap[i][j] = true;
            //     }
            // }
            frame_id_ = costmap_ros_->getGlobalFrameID();
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            initialized_ = true;
        }
    }

    bool jpsPlanner::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
            
            if(!initialized_){
                ROS_ERROR("The plannner has not been initialized...");
                return false;
            }
            ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                goal.pose.position.x, goal.pose.position.y);
            
            double wx = start.pose.position.x;
            double wy = start.pose.position.y;

            unsigned int start_x, start_y;
            costmap_->worldToMap(wx, wy, start_x, start_y);
            int start_index = costmap_->getIndex(start_x, start_y);

            wx = goal.pose.position.x;
            wy = goal.pose.position.y;

            unsigned int goal_x, goal_y;
            costmap_->worldToMap(wx, wy, goal_x, goal_y);
            int goal_index = costmap_->getIndex(goal_x, goal_y);

            openList.clear();

            std::vector<bool> visted(map_size, false); //记录是否搜寻过节点
            visted[start_index] = true;

            openList.push_back(Node(start_index, 0, 0));//加入开始节点，其cost和gcost都初始为0

            std::vector<int> cameFrom(map_size, -1); //记录当前节点的父节点

            //std::vector<float> gcost(map_size, INT_MAX); //记录当前节点到起始节点的代价，同时用于判断是否已经探寻过该节点

            //Node top = openList[0];//取cost最小的节点
            std::cout<<start_index<<" "<<goal_index<<std::endl;

            while (!openList.empty())
            {
                Node top = openList[0];//取cost最小的节点
                std::pop_heap(openList.begin(), openList.end(), greater1());
                std::cout<<" top.index:"<<top.index<<" openList.size: "<<openList.size()<<std::endl;
                float pre_jps_gcost = top.gcost; // 记录上一个跳点的gcost
                openList.pop_back();

                //删除该节点并重新排序
                //if(openList.size() > 1) std::pop_heap(openList.begin(), openList.end(), greater1());

                int current_index = top.index;

                if(current_index == goal_index){
                    ROS_INFO("getPath!");
                    getPlan(start_index, goal_index, cameFrom, plan);
                    return true;
                }
                getJumpNode(current_index, visted, cameFrom, goal_index, pre_jps_gcost);

            }
            if(cameFrom[goal_index] == -1){
                ROS_INFO("can't get a path");
                return false;
            }
	   return false;
            
        }

        void jpsPlanner::getJumpNode(int index, std::vector<bool>& visted, 
            std::vector<int>& cameFrom, int goal_index, float current_gcost){
                //std::cout<<" into getJumpNode ";
                // int pre_index = cameFrom[index];
                // unsigned pre_x, pre_y, current_x, current_y;
                // if(pre_index == -1){
                //     pre_index = index;
                // }
                // costmap_->indexToCells(pre_index, pre_x, pre_y);
                unsigned current_x, current_y;
                costmap_->indexToCells(index, current_x, current_y);

                std::vector<unsigned int> neighbour = getFreeNeighbourCell(current_x, current_y, visted);
                //std::cout<<"neighbour.size: "<<neighbour.size()<<std::endl;
                for(std::vector<unsigned int>::iterator it = neighbour.begin(); it != neighbour.end(); it++){
                    unsigned int nb_x, nb_y;
                    costmap_->indexToCells(*it, nb_x, nb_y);
                    int diff_x = getOri(current_x - nb_x);
                    int diff_y = getOri(current_y - nb_y);
                    if(searchJPS(index, current_x, current_y, visted, cameFrom, diff_x, diff_y, goal_index, current_gcost)) continue;

                }
                

        }

        std::vector<unsigned int> jpsPlanner::getFreeNeighbourCell(int current_x, int current_y, std::vector<bool>& visted){
            //std::cout<<" into getFreeNeighbourCell ";
            std::vector<unsigned int> nbNode;
            for(int x = -1; x <=1; x++){
                for(int y = -1; y <= 1; y++){
                    int next_x = current_x + x;
                    int next_y = current_y + y;
                    unsigned int next_index = costmap_->getIndex(next_x, next_y);
                    if(next_x >= 0 && next_x <= width && next_y >= 0 && next_y <= height && !visted[next_index]){
                        nbNode.push_back(next_index);
                        //visted[next_index] = true;
                    }
                }
            }
            return nbNode;
        }

         bool jpsPlanner::searchJPS(int pre_jps_index, int current_x, int current_y, std::vector<bool>& visted, 
            std::vector<int>& cameFrom, int dx, int dy, int goal_index, float gcost){
                 std::cout<<pre_jps_index<<"  ";
                int next_x = current_x + dx;
                int next_y = current_y + dy;
                int next_index = costmap_->getIndex(next_x, next_y);

                if(!visted[next_index]){
                    visted[next_index] = true; //标记该节点为已访问

                    if(dx != 0 || dy != 0) gcost += 1;
                    else gcost += 1.4;

                    if(next_index == goal_index){
                        float cost = gcost + getHCost(next_index, goal_index);
                        openList.push_back(Node(next_index, cost, gcost));
                        std::push_heap(openList.begin(), openList.end(), greater1());
                        cameFrom[next_index] = pre_jps_index;
                        return true;
                    }

                    if(next_x < 0 || next_x >= width || next_y < 0 || next_y >= height || isCollision(next_x, next_y)){
                        return false;
                    }
                    //斜线找到跳点时，同时压入该跳点以及具有强迫邻居的节点
                    //为了解决斜线搜索时，从当前跳点开始，直线再搜索跳点（因为直线节点在上一次找到目前跳点时已经标为被访问）
                    if(dx != 0 && dy != 0){
                        //next_index-->需要更新具有强邻居节点的父节点为找到的跳点

                        if(searchJPS(next_index, current_x, current_y, visted, cameFrom, dx, 0, goal_index, gcost) ||
                            searchJPS(next_index, current_x, current_y, visted, cameFrom, 0, dy, goal_index, gcost)){
                                float cost = gcost + 20*getHCost(next_index, goal_index);
                                openList.push_back(Node(next_index, cost, gcost));
                                std::push_heap(openList.begin(), openList.end(), greater1());
                                cameFrom[next_index] = pre_jps_index;
                                return true;
                            }

                    }else{
                        if(dx != 0){
                            if(!isCollision(next_x, next_y) && ((isCollision(current_x, current_y + 1) && !isCollision(next_x, current_y + 1)) || 
                                (isCollision(current_x, current_y - 1) && !isCollision(next_x, current_y - 1)))){
                                    float cost = gcost + 20*getHCost(next_index, goal_index);
                                    openList.push_back(Node(next_index, cost, gcost));
                                    std::push_heap(openList.begin(), openList.end(), greater1());
                                    cameFrom[next_index] = pre_jps_index;
                                    return true;
                                }

                        }else{
                            if(!isCollision(next_x, next_y) && ((isCollision(current_x + 1, current_y) && !isCollision(current_x + 1, next_y)) || 
                                (isCollision(current_x - 1, current_y) && !isCollision(current_x - 1, next_y)))){
                                    float cost = gcost + 20*getHCost(next_index, goal_index);
                                    openList.push_back(Node(next_index, cost, gcost));
                                    std::push_heap(openList.begin(), openList.end(), greater1());
                                    cameFrom[next_index] = pre_jps_index;
                                    return true;
                                }
                        }
                    }
                    
                    return searchJPS(pre_jps_index, next_x, next_y, visted, cameFrom, dx, dy, goal_index, gcost);

                }
            return false;      
        }

        float jpsPlanner::getHCost(int current_index, int goal_index){
            unsigned int map_start_x, map_start_y, map_goal_x, map_goal_y;
            double world_start_x, world_start_y, world_goal_x, world_goal_y;

            costmap_->indexToCells(current_index, map_start_x, map_start_y);
            costmap_->indexToCells(goal_index, map_goal_x, map_goal_y);
            costmap_->mapToWorld(map_start_x, map_start_y, world_start_x, world_start_y);
            costmap_->mapToWorld(map_goal_x, map_goal_y, world_goal_x, world_goal_y);

            return std::sqrt((std::pow(world_goal_x - world_start_x, 2)) + std::pow(world_goal_y - world_start_y, 2));
        }

        inline bool jpsPlanner::isCollision(int x, int y){
            unsigned int cost = static_cast<int>(costmap_->getCost(x, y));
            if(cost >= 240) return true;
            return false;
        }

        void jpsPlanner::getPlan(int start_index, int goal_index, std::vector<int> camefrom, std::vector<geometry_msgs::PoseStamped>& plan){
             std::cout<<" into getPlan ";
            int current_index = goal_index;
            std::vector<int> bestPath;
            while(current_index != start_index){
                bestPath.push_back(current_index);
                current_index = camefrom[current_index];
            }
            bestPath.push_back(current_index);
            std::reverse(bestPath.begin(), bestPath.end());

            ros::Time plan_time = ros::Time::now();
            for(int i = 0; i < bestPath.size(); i++){
                unsigned int map_x, map_y;
                costmap_->indexToCells(bestPath[i], map_x, map_y);
                double world_x, world_y;
                costmap_->mapToWorld(map_x, map_y, world_x, world_y);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();

                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                plan.push_back(pose);
            }
            publishPlan(plan);
        }

        void jpsPlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> path){
            if(!initialized_){
                ROS_ERROR("has not initialized");
                return ;
            }
            nav_msgs::Path gui_path;
            gui_path.poses.resize(path.size());

            gui_path.header.frame_id = frame_id_;
            gui_path.header.stamp = ros::Time::now();

            for(unsigned int i = 0; i < path.size(); i++){
                gui_path.poses[i] = path[i];
            }
            plan_pub_.publish(gui_path);
        }


}
