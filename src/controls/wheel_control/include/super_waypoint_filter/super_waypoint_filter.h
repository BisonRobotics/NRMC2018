#include <geometry_msgs/Pose2D.h>

class SuperWaypointFilter
{
public:
    bool filterWaypoints(std::vector<geometry_msgs::Pose2D> waypointList);
    std::vector<geometry_msgs::Pose2D> getForwardPath();
    std::vector<geometry_msgs::Pose2D> getBackwardPath();
    std::vector<geometry_msgs::Pose2D> getRawPath();
    void insertPointInStartZone(std::vector<geometry_msgs::Pose2D> &waypointList)
private:
    std::vector<geometry_msgs::Pose2D> rawPath;
    std::vector<geometry_msgs::Pose2D> forwardPath;
    std::vector<geometry_msgs::Pose2D> backwardPath;
};