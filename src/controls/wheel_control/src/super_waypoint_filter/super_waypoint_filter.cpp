#include <super_waypoint_filter/super_waypoint_filter.h>

#define MIN_DISTANCE_FOR_RUN 2.5
#define OBSTACLE_ZONE_START_X 1.5
#define OBSTACLE_ZONE_END_X 4.44
#define MIN_WAYPOINT_DISTANCE .8

double interpolateYFromXAndTwoPoints(double x0, double y0, double x1, double y1, double x)
{
    return ((y1 - y0)/(x1 - x0)) * (x - x0) + y0;
}

bool filterWaypoints(std::vector<geometry_msgs::Pose2D> waypointList)
{
    rawPath = waypointList; //copy vector
    if (waypointList.back.x - waypointList.front.x < 0)
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
    Pose2D point_to_insert;
    
    //grab entrance to obstacle zone
    if (noWaypointsInStartZone)
    {
        //extrapolate first point back until obstacle zone entrance.
        //Add this point as the first one for going forward
        point_to_insert.x = OBSTACLE_ZONE_START_X;
        point_to_insert.y = interpolateYFromXAndTwoPoints(waypointList.front.x, waypointList.front.y,
                            waypointList.front.x - cos(waypointList.front.theta),
                            waypointList.front.x - sin(waypointList.front.theta), 
                            OBSTACLE_ZONE_START_X);
        point_to_insert.theta = 0.0;
        forwardPath.push_back(point_to_insert);
    }
    else
    {
        //waypoints in start zone, extrapolate obstacle zone entrance point from 
        //last point in start zone, which should point directly towards the next point
        point_to_insert.x = OBSTACLE_ZONE_START_X;
        point_to_insert.y = interpolateYFromXAndTwoPoints(waypointList.at(indexOfLastWaypointInStartZone).x, 
                            waypointList.at(indexOfLastWaypointInStartZone).y,
                            waypointList.at(indexOfLastWaypointInStartZone).x - cos(waypointList.at(indexOfLastWaypointInStartZone).theta),
                            waypointList.at(indexOfLastWaypointInStartZone).y - sin(waypointList.at(indexOfLastWaypointInStartZone).theta),
                            OBSTACLE_ZONE_START_X);
        point_to_insert.theta = waypointList.at(indexOfLastWaypointInStartZone).theta;
        forwardPath.push_back(point_to_insert);
        
    }
    
    //grab obstacle zone points, if any
    int indexToStartAddingPointsAt = noWaypointsInStartZone ? 0 : indexOfLastWaypointInStartZone;
    int indexOfFirstGoalZonePoint = -1;
    for (int index =indexToStartAddingPointsAt; index< waypointsList.size(); index++)
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
    if (indexOfFirstGoalZonePoint == -1) //no point in goal zone, this is probably bad
    {
        point_to_insert.x = OBSTACLE_ZONE_END_X;
        point_to_insert.y = interpolateYFromXAndTwoPoints(waypointList.back.x, waypointList.back.y,
                                                          waypointList.back.x - cos(waypointList.back.theta),
                                                          waypointList.back.y - sin(waypointList.back.theta),
                                                          OBSTACLE_ZONE_END_X);
        point_to_insert.theta = waypointList.back.theta;
        forwardPath.push_back(point_to_insert);
    }
    else
    {
        //interpolate between last point in goal zone and point before it
        //the point before it points directly at it
        //if there is no point before it, just extrapolate it back
        if (indexOfFirstGoalZonePoint > 0)
        {
            point_to_insert.x = OBSTACLE_ZONE_END_X;
            point_to_insert.y = interpolateYFromXAndTwoPoints(waypointList.at(indexOfFirstGoalZonePoint-1).x,
                                waypointList.at(indexOfFirstGoalZonePoint-1).y,
                                waypointList.at(indexOfFirstGoalZonePoint-1).x - cos(waypointList.at(indexOfFirstGoalZonePoint-1)),
                                waypointList.at(indexOfFirstGoalZonePoint-1).y - sin(waypointList.at(indexOfFirstGoalZonePoint-1)),
                                OBSTACLE_ZONE_END_X;
            point_to_insert.theta = waypointList.at(indexOfFirstGoalZonePoint-1).theta;
            forwardPath.push_back(point_to_insert);
        }
        else
        {
            point_to_insert.x = OBSTACLE_ZONE_END_X;
            point_to_insert.y = interpolateYFromXAndTwoPoints(waypointList.at(indexOfFirstGoalZonePoint).x,
                                waypointList.at(indexOfFirstGoalZonePoint).y,
                                waypointList.at(indexOfFirstGoalZonePoint).x - cos(waypointList.at(indexOfFirstGoalZonePoint)),
                                waypointList.at(indexOfFirstGoalZonePoint).y - sin(waypointList.at(indexOfFirstGoalZonePoint)),
                                OBSTACLE_ZONE_END_X;
            point_to_insert.theta = waypointList.at(indexOfFirstGoalZonePoint).theta;
            forwardPath.push_back(point_to_insert);
        }
    }
    
    //insert points at x = 2.0m, x = 2.5m
        
    
    //reverse path for backward path
    
    //add points to forward path for digging
    
    //add points to reverse path for dumping
    
    return true;
}

std::vector<geometry_msgs::Pose2D> getForwardPath()
{
    return forwardPath;
}


std::vector<geometry_msgs::Pose2D> getBackwardPath()
{
    return backwardPath;
}

std::vector<geometry_msgs::Pose2D> getRawPath()
{
    return rawPath;
}
