#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/LinkStates.h"
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

class ImpedanceController
{
public:
  ImpedanceController()
  {
    force_pub = n_.advertise<geometry_msgs::Wrench>("/published_force", 100);
    position_sub = n_.subscribe("/gazebo/link_states", 1, &ImpedanceController::chatterCallback, this);
  }

  void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
  {
    world.position.x = 0;
    world.position.y = 0;
    world.position.z = 0;
    world.orientation.x = 0;
    world.orientation.y = 0;
    world.orientation.z = 0;
    world.orientation.w = 1;

    tf::Quaternion desired_orientation(
          world.orientation.x,
          world.orientation.y,
          world.orientation.z,
          world.orientation.w);

    tf::Matrix3x3 md(desired_orientation);
    md.getRPY(roll_desired, pitch_desired, yaw_desired);
    
    tf::Quaternion actual_orientation(
          msg->pose[1].orientation.x,
          msg->pose[1].orientation.y,
          msg->pose[1].orientation.z,
          msg->pose[1].orientation.w);

    tf::Matrix3x3 ma(actual_orientation);
    ma.getRPY(roll_actual, pitch_actual, yaw_actual);

    pos_desired <<  world.position.x, world.position.y, world.position.z,roll_desired ,pitch_desired, yaw_desired;

    pos_current <<  msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z,roll_actual,pitch_actual,yaw_actual;

    vel_desired << 0, 0, 0, 0, 0, 0;

    vel_current <<  msg->twist[1].linear.x, msg->twist[1].linear.y, msg->twist[1].linear.z, msg->twist[1].angular.x, msg->twist[1].angular.y, msg->twist[1].angular.z;

    Force = b*(vel_desired-vel_current)+k*(pos_desired-pos_current);

    ROS_INFO("Force");
    ROS_INFO("x = %f",Force[0]);
    ROS_INFO("y = %f",Force[1]);
    ROS_INFO("z = %f",Force[2]);
    ROS_INFO("roll = %f",Force[3]);
    ROS_INFO("pitch = %f",Force[4]);
    ROS_INFO("yaw = %f",Force[5]);

    geometry_msgs::Wrench force_msg;

    force_msg.force.x = Force[0];
    force_msg.force.y = Force[1];
    force_msg.force.z = Force[2];
    force_msg.torque.x = Force[3];
    force_msg.torque.y = Force[4];
    force_msg.torque.z = Force[5];

    force_pub.publish(force_msg);

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher force_pub;
  ros::Subscriber position_sub;
  geometry_msgs::Pose world;
  Eigen::VectorXd vel_current = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd pos_current = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd vel_desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd pos_desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force = Eigen::VectorXd::Zero(6);
  double b = 1;
  double k = 100;
  double roll_desired, pitch_desired, yaw_desired;
  double roll_actual, pitch_actual, yaw_actual;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Impedance_node");
  ImpedanceController impedance; 
  ros::spin();
  return 0;
}



