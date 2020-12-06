#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using namespace std;
using std::string;

#ifndef ZM_POTENTIALFIELD_GLOBAL_PLANNER_CPP
#define ZM_POTENTIALFIELD_GLOBAL_PLANNER_CPP

namespace zm_potentialfield_global_planner
{
    class zmPotentialFieldGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
        public:
          zmPotentialFieldGlobalPlanner(); 
          zmPotentialFieldGlobalPlanner( std::string name, costmap_2d::Costmap2DROS* costmap_ros);
          void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
          bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
          
        private:
          std::vector<int> potentialPlanner(int startCell, int goalCell);
          std::vector<int> findPath(int startCell, int goalCell);
          void calculateGoalPotential(int goalCell) ;
          void calculateObstaclePotential();
          void getCoordinate (float& x, float& y);
          int convertToCellIndex(float x, float y);
          void convertToCoordinate(int index, float& x, float& y);
          bool isCellInsideMap(float x, float y);
          bool isStartAndGoalCellsValid(int startCell,int goalCell);
          bool isFree(int cellID);
          float getDistance(float x1, float y1, float x2, float y2);
          void sumPointPotential( int cellID);
          int findBestCell(int cellID);
          int getCellIndex(int i, int j);
          int getCellRowID(int index);
          int getCellColID(int index);
          bool isGoalAcomplished( int currentCell , int goalCell);
          
          float originX_;
          float originY_;
          float resolution_;
          costmap_2d::Costmap2DROS* costmap_ros_;
          costmap_2d::Costmap2D* costmap_;
          bool initialized_;
          int width_;
          int height_;
          float* posPotMap_; //position potential map;
          float* obsPotMap_; // obstacle potential map;
    };
};
#endif 