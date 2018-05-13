#include <super_waypoint_filter/super_waypoint_filter.h>
#define MIN_DISTANCE_FOR_RUN 2.5
#define OBSTACLE_ZONE_START_X 1.5
#define OBSTACLE_ZONE_END_X 4.44
#define MIN_WAYPOINT_DISTANCE .8

#define FIELD_WIDTH_2 1.89

double SuperWaypointFilter::interpolateYFromXAndTwoPoints(double x0, double y0, double x1, double y1, double x)
{
    if (std::abs(x1 - x0) > 0.000001)
    {
        double slope = ((y1 - y0)/(x1 - x0));
        double result = ((x - x1)*slope + y1);
        return result;
    }
    else
    {
        return (y1 + y0)/2.0;
    }
}

bool SuperWaypointFilter::filterWaypoints(std::vector<geometry_msgs::Pose2D> waypointList)
{
    rawPath = waypointList; //copy vector
    if (waypointList.back().x - waypointList.front().x < 0)
    {
        return false; //waypoints did not go forward
    }
    bool noWaypointsInStartZone = true;
    int indexOfLastWaypointInStartZone = 0;
    for (int index=0; index<waypointList.size(); index++)
    {
        //find if a waypoint is in the start zone or not
        if (waypointList.at(index).x < OBSTACLE_ZONE_START_X)
        {
            noWaypointsInStartZone = false;
            indexOfLastWaypointInStartZone = index;
        }
    }
    
    forwardPath.clear();
    backwardPath.clear();
    geometry_msgs::Pose2D point_to_insert;
    
    //grab entrance to obstacle zone
    if (noWaypointsInStartZone)
    {
        forwardPath.push_back(waypointList.front()); //grab first waypoint and extrapolate back
        if (interpolateAndAddPoint(&forwardPath, OBSTACLE_ZONE_START_X) != 1)
        {
            return false;
        }
        forwardPath.erase(forwardPath.begin()+1);
    }
    else
    {
        forwardPath.push_back(waypointList.at(indexOfLastWaypointInStartZone)); //valid to extrapolate this point forward
                                                                                //because it will point at next point (in obstacle field)
                                                                                //if there is no next point, it will still work
        if (interpolateAndAddPoint(&forwardPath, OBSTACLE_ZONE_START_X) != 3)
        {
            return false;
        }
        forwardPath.erase(forwardPath.begin());
    }
    
    //grab obstacle zone points, if any
    //also find the first point in the goal zone
    int indexToStartAddingPointsAt = noWaypointsInStartZone ? 0 : indexOfLastWaypointInStartZone + 1;
    int indexOfFirstGoalZonePoint = -1;
    for (int index =indexToStartAddingPointsAt; index< waypointList.size(); index++)
    {
        if (waypointList.at(index).x < OBSTACLE_ZONE_END_X)
        {
            forwardPath.push_back(waypointList.at(index));
        }
        else
        {
            indexOfFirstGoalZonePoint = index;
            break;
        }
    }
    
    //calculate obstacle zone exit point
    if (indexOfFirstGoalZonePoint == -1) //no real point in goal zone
    {
        if (interpolateAndAddPoint(&forwardPath, OBSTACLE_ZONE_END_X) != 3)
        {
            return false;
        }
    }
    else if (indexOfFirstGoalZonePoint >= 0) //first point in waypointList is goal point
    {
        forwardPath.push_back(waypointList.at(indexOfFirstGoalZonePoint));
        if (interpolateAndAddPoint(&forwardPath, OBSTACLE_ZONE_END_X) != 2) //have entrance point and goal point 
        {
            return false;
        }
        forwardPath.erase(forwardPath.end());
    }
    
    //interpolate intermediate obstacle zone points
    //guaranteed to have at least entrance and exit to obstacle zone in forward path at this point
    interpolateAndAddPoint(&forwardPath, 2.0);  
    interpolateAndAddPoint(&forwardPath, 2.5);  
    interpolateAndAddPoint(&forwardPath, 3.0);  
    interpolateAndAddPoint(&forwardPath, 3.5);  
    interpolateAndAddPoint(&forwardPath, 4.0);  
    //reverse path for backward path
    for(int index = 0; index < forwardPath.size(); index++)
    {
      point_to_insert.x = forwardPath.at(forwardPath.size() - index - 1).x;
      point_to_insert.y = forwardPath.at(forwardPath.size() - index - 1).y;
      if (index < forwardPath.size() - 1)
      {
        point_to_insert.theta = forwardPath.at(forwardPath.size() - index - 2).theta;
      }
      else 
      {
         point_to_insert.theta = 0;
      }
      backwardPath.push_back(point_to_insert);
    }
    //add points to forward path for digging
    point_to_insert.x = 5.4;
    point_to_insert.y = forwardPath.back().y + .015;
    point_to_insert.theta = 0;
    forwardPath.push_back(point_to_insert);
    
    point_to_insert.x = 6.4;
    point_to_insert.y = forwardPath.back().y - .015;
    point_to_insert.theta = 0;
    forwardPath.push_back(point_to_insert);
    
    //add points to reverse path for dumping
    point_to_insert.x = .8;
    point_to_insert.y = 0;
    point_to_insert.theta = 0;
    backwardPath.push_back(point_to_insert);
    
    point_to_insert.x = 1.4;
    point_to_insert.y = 0.015;
    point_to_insert.theta = 0;
    backwardPath.push_back(point_to_insert);
    
    point_to_insert.x = .8;
    point_to_insert.y = 0.015;
    point_to_insert.theta = 0;
    backwardPath.push_back(point_to_insert);
    
    point_to_insert.x = -.8;
    point_to_insert.y = -0.015;
    point_to_insert.theta = 0;
    backwardPath.push_back(point_to_insert);
    
    return true;
}

int SuperWaypointFilter::interpolateAndAddPoint( std::vector<geometry_msgs::Pose2D> *path, double insertX)
{
    geometry_msgs::Pose2D waypoint;
    int ret_val=0;
    int indexToPutMidPointAt;
    for (auto const &wp : *path)
    {
        if (wp.x == insertX)
        {
            return 0; //waypoint is already there
        }
    }
    if (path->at(0).x > insertX)
    {
        //there was no point before the
        //place one was to be inserted
        //so extrapolate back
        waypoint.x = insertX;
        waypoint.y = interpolateYFromXAndTwoPoints(path->front().x, path->front().y,
                            path->front().x - cos(path->front().theta),
                            path->front().y - sin(path->front().theta), 
                            insertX);
        if (std::abs(waypoint.y) > FIELD_WIDTH_2 - .5)
        {
          waypoint.y = path->front().y;
          waypoint.theta = 0;
        }
        else
        {
          waypoint.theta = path->front().theta;
        }
        path->insert(path->begin(), waypoint);
        ret_val = 1;
    }
    else
    {
        for (indexToPutMidPointAt=1; indexToPutMidPointAt < path->size(); indexToPutMidPointAt++)
        {
            if (path->at(indexToPutMidPointAt-1).x < insertX && path->at(indexToPutMidPointAt).x > insertX)
            {
                waypoint.x = insertX;
                waypoint.y = interpolateYFromXAndTwoPoints(path->at(indexToPutMidPointAt).x, path->at(indexToPutMidPointAt).y,
                                                                  path->at(indexToPutMidPointAt-1).x,path->at(indexToPutMidPointAt-1).y,
                                                                  insertX);
                waypoint.theta = path->at(indexToPutMidPointAt-1).theta;
                path->insert(path->begin() + indexToPutMidPointAt, waypoint);
                ret_val = 2;
                break;
            }
        }
        if (indexToPutMidPointAt == path->size())
        {
            //there was no point beyond where
            //the inserted one should be
            //so extrapolate forward
            waypoint.x = insertX;
            waypoint.y = interpolateYFromXAndTwoPoints(path->back().x, path->back().y,
            path->back().x - cos(path->back().theta),
            path->back().y - sin(path->back().theta),
            insertX);
            if (std::abs(waypoint.y) > FIELD_WIDTH_2 - .4)
            {
                waypoint.y = path->back().y;
                waypoint.theta = 0;
            }
            else
            {
                waypoint.theta = path->back().theta;
            }
            path->push_back(waypoint);
            ret_val = 3;
        }
    }    
    return ret_val;
}


std::vector<geometry_msgs::Pose2D> SuperWaypointFilter::getForwardPath()
{
    return forwardPath;
}


std::vector<geometry_msgs::Pose2D> SuperWaypointFilter::getBackwardPath()
{
    return backwardPath;
}

std::vector<geometry_msgs::Pose2D> SuperWaypointFilter::getRawPath()
{
    return rawPath;
}
