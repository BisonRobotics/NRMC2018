#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#include <exception>

AprilTagTrackerInterface::AprilTagTrackerInterface()
{
  tfListener = new tf2_ros::TransformListener(tfBuffer);
}

AprilTagTrackerInterface::~AprilTagTrackerInterface()
{
  delete tfListener;
}

double AprilTagTrackerInterface::getX()
{
  return x;
}

double AprilTagTrackerInterface::getY()
{
  return y;
}

double AprilTagTrackerInterface::getTheta()
{
  return theta;
}

ReadableSensors::ReadStatus AprilTagTrackerInterface::receiveData()
{
  try {
    geometry_msgs::TransformStamped msg = tfBuffer.lookupTransform("map", "apriltag_tracker_pose_estimate", ros::Time(0));
    tf2::fromMsg(msg, map_to_position_estimate_tf);
    x = map_to_position_estimate_tf.getOrigin().x();
    y = map_to_position_estimate_tf.getOrigin().y();

    tf2::Matrix3x3 matrix; // TODO way of skipping this step?
    matrix.setRotation(map_to_position_estimate_tf.getRotation());
    double tmp1, tmp2;
    matrix.getRPY(tmp1, tmp2, theta);

    return ReadableSensors::ReadStatus::READ_SUCCESS;
  }
  catch (std::exception e)
  {
    //ROS_WARN("Caught exception: %s", e.what());
    return ReadableSensors::ReadStatus::READ_FAILED;
  }

}