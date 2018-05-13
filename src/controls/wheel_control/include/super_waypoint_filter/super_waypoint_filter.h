#include <geometry_msgs/Pose2D.h>

class SuperWaypointFilter
{
public:
    bool filterWaypoints(std::vector<geometry_msgs::Pose2D> waypointList);
    std::vector<geometry_msgs::Pose2D> getForwardPath();
    std::vector<geometry_msgs::Pose2D> getBackwardPath();
    std::vector<geometry_msgs::Pose2D> getRawPath();
    int interpolateAndAddPoint( std::vector<geometry_msgs::Pose2D> *path, double insertX);
private:
    std::vector<geometry_msgs::Pose2D> rawPath;
    std::vector<geometry_msgs::Pose2D> forwardPath;
    std::vector<geometry_msgs::Pose2D> backwardPath;
};