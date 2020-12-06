#include <zm_potentialfield_global_planner/zm_potentialfield_global_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(zm_potentialfield_global_planner::zmPotentialFieldGlobalPlanner, nav_core::BaseGlobalPlanner)

//using namespace std;

int value_;
int mapSize_;
bool* OGM_;
const float RAIO = 1 ; // Raio do Potencial de um obstáculo
const float JUMP = 1 ;
const float C = 3;      // Constante da Soma do Potencial de um obstáculo
const float GOALPOTENTIALMULTIPLIER = 100;
const float WALK =  2 ; // quantas cells pular
const float GOALERROR = 0.3 ;// distancia da goal para ser considerado atiginda
const float DELTAERROR = 20 ;

namespace zm_potentialfield_global_planner
{
    zmPotentialFieldGlobalPlanner::zmPotentialFieldGlobalPlanner()
    {

    }
    
    zmPotentialFieldGlobalPlanner::zmPotentialFieldGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }
    
    void zmPotentialFieldGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            originX_ = costmap_->getOriginX();
            originY_ = costmap_->getOriginY();
            width_ = costmap_->getSizeInCellsX();
            height_ = costmap_->getSizeInCellsY();
            resolution_ = costmap_->getResolution();
            mapSize_ = width_ * height_;
            cout << "width = " << width_ << ", height = " << height_ << ", mapsize = " << mapSize_ << endl;
            
            posPotMap_ = new float [mapSize_];
            obsPotMap_ = new float [mapSize_];
            OGM_ = new bool [mapSize_];

            for (unsigned int iy = 0; iy < height_; iy++)
            {
                for (unsigned int ix = 0; ix < width_; ix++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
                    if (cost < DELTAERROR )
                    {
                        OGM_[iy * width_ + ix] = true;
                    }
                    else
                    {
                        OGM_[iy * width_ + ix] = false;
                    }
                }
            }
            
            calculateObstaclePotential();
            ROS_INFO(" Potential planner initialized successfully - Obstacle Potential Already Calculated");
            initialized_ = true;
        }
        else
        {
            ROS_WARN("This planner has already been initialized... doing nothing");
        }
    }
    
    void zmPotentialFieldGlobalPlanner::calculateObstaclePotential()
    {
        for( int i = 0 ; i < mapSize_ ; i++)
        {
            if (!(isFree(i)))
            {
                sumPointPotential(i);
            }
        }
    }
    
    bool zmPotentialFieldGlobalPlanner::isFree(int cellID)
    {
        return OGM_[cellID];
    }
    
    void zmPotentialFieldGlobalPlanner::sumPointPotential(int cellID)
    {
        float x, y;
        int index;
        float cost;
        convertToCoordinate( cellID, x, y);
        getCoordinate(x,y);
        for( register float i = - RAIO; i < RAIO; i+= JUMP * resolution_)
        {
            for( register float j = - (RAIO - fabs(i)) ; j < (RAIO - fabs(i)) ; j += JUMP * resolution_)
            {
                if ( isCellInsideMap(x+i,y+j))
                {
                    index = convertToCellIndex(x+i ,y+j);
                    if( i != 0 || j != 0 )
                    {
                        cost = ( C / getDistance( x+i, y+j, x, y));
                    }   
                    else
                    {
                        cost = 300000;
                    } 
                    
                    if(obsPotMap_[index] < cost)
                    {
                        obsPotMap_[index] = cost ;
                    }
                }
            }
        }
    }
    
    void zmPotentialFieldGlobalPlanner::convertToCoordinate(int index, float& x, float& y)
    {
        x = getCellColID(index) * resolution_;
        y = getCellRowID(index) * resolution_;
        
        x = x + originX_;
        y = y + originY_;
    }
    
    void zmPotentialFieldGlobalPlanner::getCoordinate (float& x, float& y)
    {
        x = x - originX_;
        y = y - originY_;
    }

    int zmPotentialFieldGlobalPlanner::convertToCellIndex(float x, float y)
    {
        float newX = x / resolution_;
        float newY = y / resolution_;
        return getCellIndex(newX, newY);
    }

    int zmPotentialFieldGlobalPlanner::getCellIndex(int i ,int j)
    {
        return i + (j * width_);
    }
    
    int zmPotentialFieldGlobalPlanner::getCellRowID(int index)
    {
        return index / width_;
    }
    
    int zmPotentialFieldGlobalPlanner::getCellColID(int index)
    {
        return index % width_;
    }
    
    bool zmPotentialFieldGlobalPlanner::isCellInsideMap(float x, float y)
    {
        bool valid;
        valid = ! (x >= ( width_ * resolution_) || y >= (height_* resolution_) || (x < 0) || (y< 0));
    }
    
    float zmPotentialFieldGlobalPlanner::getDistance(float x1, float y1, float x2, float y2)
    {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    
    bool zmPotentialFieldGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }
            
        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
        
        plan.clear();

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        // convert the start and goal positions
        
        float startX = start.pose.position.x;
        float startY = start.pose.position.y;
        
        float goalX = goal.pose.position.x;
        float goalY = goal.pose.position.y;

        getCoordinate(startX, startY);
        getCoordinate(goalX, goalY);
        
        int startCell;
        int goalCell;

        if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
        {
            startCell = convertToCellIndex(startX, startY);
            goalCell = convertToCellIndex(goalX, goalY);
        }
        else
        {
            ROS_WARN("the start or goal is out of the map");
            return false;
        }

        if(isStartAndGoalCellsValid(startCell, goalCell))
        {
            vector<int> bestPath;
            bestPath.clear();
            bestPath = potentialPlanner(startCell, goalCell); 

            if(bestPath.size() > 0)
            {
                for(int i = 0; i < bestPath.size(); i++)
                {
                    float x = 0.0;
                    float y = 0.0;
                    int index = bestPath[i];
                    convertToCoordinate(index, x, y);

                    geometry_msgs::PoseStamped pose = goal;
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation = start.pose.orientation;
                    plan.push_back(pose);
                }

                plan.push_back(goal);

                float path_length = 0.0;
                
                std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
                geometry_msgs::PoseStamped last_pose;
                last_pose = *it;
                it++;
                for (; it!=plan.end(); ++it)
                {
                    path_length += hypot((*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y );
                    last_pose = *it;
                }
                cout <<"The global path length: "<< path_length << " meters" <<endl;
                return true;
            }
            else
            {
                ROS_WARN("The planner failed to find a path, choose other goal position");
                return false;
            }
        }
        else
        {
            ROS_WARN("Not valid start or goal");
            return false;
        }
    }

    bool zmPotentialFieldGlobalPlanner::isStartAndGoalCellsValid(int startCell, int goalCell)
    {
        bool isvalid=true;
        bool isFreeStartCell = isFree(startCell);
        bool isFreeGoalCell = isFree(goalCell);
        if (startCell==goalCell)
        {
            ROS_WARN("The Start and the Goal cells are the same...");
            isvalid = false;
        }
        else if(!isFreeStartCell || !isFreeGoalCell)
        {
            ROS_ERROR("The start or the goal cells are obstacle positions...");
            isvalid = false;
        }

        return isvalid;
    }

    std::vector<int> zmPotentialFieldGlobalPlanner::potentialPlanner(int startCell, int goalCell)
    {
        vector<int> bestPath;
        calculateGoalPotential(goalCell);
        bestPath = findPath(startCell, goalCell);
        return bestPath;
    }

    void zmPotentialFieldGlobalPlanner::calculateGoalPotential(int goalCell)
    {
        float goalX,goalY;
        convertToCoordinate(goalCell, goalX, goalY);
        float x, y ;

        for( int i=0; i < mapSize_ ; i++)
        {
            convertToCoordinate(i, x, y);
            posPotMap_[i] = GOALPOTENTIALMULTIPLIER * getDistance(x, y, goalX, goalY);
        }
    }

    std::vector<int> zmPotentialFieldGlobalPlanner::findPath(int startCell, int goalCell)
    {
        std::vector<int> bestPath;
        int currentCell= startCell;
        bestPath.push_back(currentCell);
        while(!isGoalAcomplished(currentCell, goalCell))
        {
            currentCell = findBestCell(currentCell);
            if(currentCell == - 1)
            {
                return bestPath;
            } 
            bestPath.push_back(currentCell);
        }
        return bestPath;
    }

    bool zmPotentialFieldGlobalPlanner::isGoalAcomplished(int currentCell, int goalCell)
    {
        float xG,yG,xC,yC;
        convertToCoordinate(currentCell, xC, yC);
        convertToCoordinate(goalCell, xG, yG);
        if(getDistance(xC, yC, xG, yG) < GOALERROR)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int zmPotentialFieldGlobalPlanner::findBestCell(int cellID)
    {
        float x,y;
        int index, best_index;
        float best_value= 999999;
        
        convertToCoordinate(cellID, x, y);
        getCoordinate(x,y);
        index = convertToCellIndex(x , y ) ;
        cout << " DADO: (" << x << "," << y << ") INDEX: " << index << endl; //<< " COST: " << _obsPotMap[index] + _posPotMap[index] << endl ;

        for(float i = - WALK * resolution_ ; i <=  WALK * resolution_ ; i+= resolution_)
        {
            for(float j = - WALK * resolution_ ; j <= WALK * resolution_ ; j+= resolution_)
            {
                if(isCellInsideMap(x+i, y+j))
                {
                    index = convertToCellIndex(x+i, y+j);
                    if( best_value > obsPotMap_[index] + posPotMap_[index])
                    {
                        best_value = obsPotMap_[index] + posPotMap_[index];
                        best_index = index;
                    }
                }
            }
        }
        
        if( best_index == cellID)
        {
            ROS_ERROR("MINIMO LOCAL");
            return -1;
        }
        return best_index;
    }
}