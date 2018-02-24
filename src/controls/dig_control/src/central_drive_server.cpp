#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <central_drive_control/central_driveAction.h>
#include <math.h>
#include <stdio.h>

// backhoe dynamics
double ddQ[2][1] = { { 0.0001 }, { 0.0001 } };
double Q[2][1] = { { 0.0001 }, { 0.0001 } };
double dQ[2][1] = { { 0.0001 }, { 0.0001 } };

// control velocity
double v[2][1] = { { 0 }, { 0 } };

// error
double error_Q[2][1] = { { 0.0001 }, { 0.0001 } };

// gear ratios
double gr[2] = { 500.001, 500.001 };

// set goal
double set_goal[2] = { 0, 0 };

double dt = 0.01;

int kp = 5;  // gain

class central_drive
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<central_drive_control::central_driveAction> as_;  // NodeHandle instance must be created
                                                                                  // before this line. Otherwise strange
                                                                                  // error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  central_drive_control::central_driveFeedback feedback_;
  central_drive_control::central_driveResult result_;

public:
  central_drive(std::string name)
    : as_(nh_, name, boost::bind(&central_drive::executeCB, this, _1), false), action_name_(name)
  {
    as_.start();
  }

  ~central_drive(void)
  {
  }

  // int central_drive_dynamics(int var1,int setpoint){
  // int dif = var1 - setpoint;
  // return dif;
  //};
  void central_drive_dynamics(double inth[2][1], double dinth[2][1])
  {
    double m[3] = { 100, 10, 300 };
    double r[3] = { 1.001, 1.001, 1.001 };
    double k1 = 1.001;

    double s1 = sin(inth[0][0] / gr[0]);
    double c1 = cos(inth[0][0] / gr[0]);
    double s2 = sin(inth[1][0] / gr[1]);
    double c2 = cos(inth[1][0] / gr[1]);
    double s12 = sin(inth[0][0] / gr[0] + inth[1][0] / gr[1]);
    double bhbangled = 3.14159;
    double g = 9.8;

    // double r0s = r(0)*r(0);
    // double r1s = r(1)*r(1);
    // double r2s = r(2)*r(2);

    // ddQ[1][1]  = 1.01;
    // ddQ[2][1]  = 1.01;

    // backhoe dynamics for testing

    ddQ[1][0] = (-g * gr[0] * gr[0] * m[0] * r[0] * s1 * gr[1] * gr[1] -
                 1 * g * gr[0] * gr[0] * m[1] * r[0] * s1 * gr[1] * gr[1] +
                 1 * c2 * g * gr[0] * gr[0] * m[1] * r[1] * s12 * gr[1] * gr[1] -
                 1 * c2 * dinth[0][0] * dinth[0][0] * m[1] * r[1] * r[1] * s2 * gr[1] * gr[1] -
                 1 * dinth[0][0] * dinth[0][0] * m[1] * r[1] * r[1] * s2 * gr[1] * gr[1] +
                 k1 * (-0.5 * gr[0] * gr[0] - 0.5 * dinth[0][0] * dinth[0][0] * k1 * k1) * m[2] * r[2] * r[2] *
                     sin(2 * bhbangled - (2 * inth[0][0] * k1) / gr[0]) * gr[1] * gr[1] +
                 1 * g * gr[0] * gr[0] * k1 * m[2] * r[2] * sin(bhbangled - (inth[0][0] * k1) / gr[0]) * gr[1] * gr[1] -
                 2 * dinth[0][0] * dinth[1][0] * gr[0] * m[1] * r[1] * r[1] * s2 * gr[1] -
                 1 * dinth[1][0] * dinth[1][0] * gr[0] * gr[0] * m[1] * r[1] * r[1] * s2) /
                (gr[0] * gr[1] * gr[1] *
                 (-m[0] * r[0] * r[0] + c2 * c2 * gr[1] * gr[1] * m[1] * r[1] * r[1] +
                  2 * c2 * gr[1] * gr[1] * m[1] * r[1] * r[1] + gr[1] * gr[1] * m[1] * r[1] * r[1] -
                  2 * c2 * m[1] * r[1] * r[1] - 2 * m[1] * r[1] * r[1] - 0.5 * k1 * k1 * m[2] * r[2] * r[2] +
                  0.5 * k1 * k1 * m[2] * r[2] * r[2] * cos(2 * bhbangled - (2 * inth[0][0] * k1) / gr[0])));

    ddQ[2][0] =
        ((g * gr[0] * gr[0] * s12 - dinth[0][0] * dinth[0][0] * dinth[1][0] * dinth[1][0] * r[1] * s2) *
             (m[0] * r[0] * r[0] + 2 * c2 * m[1] * r[1] * r[1] + 2 * m[1] * r[1] * r[1] +
              0.5 * k1 * k1 * m[2] * r[2] * r[2] -
              0.5 * k1 * k1 * m[2] * r[2] * r[2] * cos(2 * bhbangled - (2 * inth[0][0] * k1) / gr[0])) -
         (c2 + 1) * r[1] *
             (k1 * (0.5 * gr[0] * gr[0] + 0.5 * dinth[0][0] * dinth[0][0] * k1 * k1) * m[2] * r[2] * r[2] *
                  sin(2 * bhbangled - (2 * inth[0][0] * k1) / gr[0]) * gr[1] * gr[1] +
              g * gr[0] * gr[0] * (m[0] * r[0] * s1 + m[1] * r[0] * s1 + m[1] * r[1] * s12 -
                                   k1 * m[2] * r[2] * sin(bhbangled - (inth[0][0] * k1) / gr[0])) *
                  gr[1] * gr[1] +
              dinth[1][0] * gr[0] * (dinth[1][0] * gr[0] + 2 * dinth[0][0] * gr[1]) * m[1] * r[1] * r[1] * s2)) /
        (gr[0] * gr[0] *
         ((c2 + 1) * (c2 + 1) * gr[1] * m[1] * r[1] * r[1] * r[1] -
          (2 * r[1] * (0.5 * m[0] * r[0] * r[0] + c2 * m[1] * r[1] * r[1] + m[1] * r[1] * r[1] +
                       0.25 * k1 * k1 * m[2] * r[2] * r[2] -
                       0.25 * k1 * k1 * m[2] * r[2] * r[2] * cos(2 * bhbangled - (2 * inth[0][0] * k1) / gr[0]))) /
              gr[1]));
  };

  void executeCB(const central_drive_control::central_driveGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1 / dt);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    // feedback_.sequence.push_back(0);
    // feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Setting backhoe position to %i ", action_name_.c_str(), goal->order);
    // calculate error
    error_Q[0][0] = (Q[0][0] / gr[0] - set_goal[0]);
    error_Q[1][0] = (Q[1][0] / gr[1] - set_goal[1]);
    // start executing the action
    // for(int i=1; i >= 30; i++)

    // set the goal for the central drive
    set_goal[0] = ((goal->order) / 100) * (2 * M_PI);
    set_goal[1] = 0;

    while (1)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      // storing the current value in the sequence...

      // calculate error
      error_Q[0][0] = (Q[0][0] / gr[0] - set_goal[0]);
      error_Q[1][0] = (Q[1][0] / gr[1] - set_goal[1]);

      // update acceleration
      central_drive_dynamics(Q, dQ);

      // update position
      Q[0][0] = Q[0][0] + dQ[0][0] * dt;
      Q[1][0] = Q[1][0] + dQ[0][0] * dt;

      // update velocity
      dQ[0][0] = dQ[0][0] + ddQ[0][0] * dt - error_Q[0][0] * kp;
      dQ[1][0] = dQ[1][0] + ddQ[1][0] * dt;

      // print stuff for debugging
      // ROS_INFO("error_Q [%d]",error_Q[0][0]);
      ROS_INFO("ddQ [%d]", ddQ[0][0]);

      // push feedback
      feedback_.sequence.push_back(error_Q[0][0]);

      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if (success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "central_drive");

  central_drive central("central_drive");
  ros::spin();

  return 0;
}
