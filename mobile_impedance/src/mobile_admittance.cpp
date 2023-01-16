
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include "gazebo_msgs/LinkStates.h"
#include <mobile_impedance/velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
class AdmittanceController
{
public:
  AdmittanceController()
  {
    force_sub = n_.subscribe("/published_force", 1, &AdmittanceController::AdmittanceCallback, this);
    state_sub = n_.subscribe("/gazebo/link_states", 1, &AdmittanceController::StateCallback, this);
    joint_state_sub = n_.subscribe("/mobile_base/joint_states", 1, &AdmittanceController::JointStateCallback, this);
    fl_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_l_ew_joint_velocity_controller/command", 100);
    fr_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_r_ew_joint_velocity_controller/command", 100);
    rl_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_l_ew_joint_velocity_controller/command", 100);
    rr_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_r_ew_joint_velocity_controller/command", 100);

    fl_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_l_rotate_joint_position_controller/command", 100);
    fr_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_r_rotate_joint_position_controller/command", 100);
    rl_rotation_pub = n_.advertise<std_msgs::Float64>("//mobile_base/r_l_rotate_joint_position_controller/command", 100);
    rr_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_r_rotate_joint_position_controller/command", 100);

  }

  void AdmittanceCallback(const geometry_msgs::Wrench::ConstPtr& force_msg)
  {
    
    Force << force_msg->force.x, force_msg->force.y, force_msg->force.z, force_msg->torque.x, force_msg->torque.y, force_msg->torque.z;
  
  }

  void StateCallback(const gazebo_msgs::LinkStates::ConstPtr& state_msg)
  {                    
    tf::Quaternion actual_orientation(
          state_msg->pose[1].orientation.x,
          state_msg->pose[1].orientation.y,
          state_msg->pose[1].orientation.z,
          state_msg->pose[1].orientation.w);

    tf::Matrix3x3 ma(actual_orientation);
    ma.getRPY(roll_actual, pitch_actual, yaw_actual);

    current_pose << state_msg->pose[1].position.x, state_msg->pose[1].position.y, state_msg->pose[1].position.z, roll_actual,pitch_actual,yaw_actual;
    current_vel << state_msg->twist[1].linear.x, state_msg->twist[1].linear.y, state_msg->twist[1].linear.z, state_msg->twist[1].angular.x, 
                    state_msg->twist[1].angular.y, state_msg->twist[1].angular.z;

    //model position & orientation
    Vx  = state_msg->pose[1].position.x;
    Vy  = state_msg->pose[1].position.y;
    // steer_angle = yaw_actual;
    act_angle = yaw_actual;


    
  }

  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
  {
    // each wheel angle


    wheel_theta_fl = joint_state_msg->position[0];
    wheel_theta_fr = joint_state_msg->position[1];
    wheel_theta_rl = joint_state_msg->position[2];
    wheel_theta_rr = joint_state_msg->position[3];

    wheel_theta_dot_fl_ew = joint_state_msg->velocity[0];
    wheel_theta_dot_rl_ew = joint_state_msg->velocity[2];
    wheel_theta_dot_rr_ew = joint_state_msg->velocity[3];
    wheel_theta_dot_fr_ew = joint_state_msg->velocity[1];


  }

  void run(){
    ros::Rate loop_rate(100);
    while(ros::ok)
    {
      M =   1;
      B =   10;

      vel = Force/B*(1-exp(-dt/(M/B)));
      desired_pose << 0,0,0,0,0,0;
  
      // chassis_vel << vel(0),vel(1),0; // yaw 오차값 너무 크면 안됨 -> 파라미터 값을 impedance에서 줄이던, admittance에서 줄이던 해야함
      chassis_vel << vel(0),vel(1),vel(5);
      // chassis_vel << 0,0,0;


      jacobian_pinv<<  1,  0,  -b,
                       0,  1,   a,
                       1,  0,  -b,
                       0,  1,  -a,
                       1,  0,   b,
                       0,  1,  -a,
                       1,  0,   b,
                       0,  1,   a;

      // std::cout << "jacobian" << std::endl << jacobian_pinv << std::endl << "chassis_vel" << std::endl << chassis_vel << std::endl;
  
      joint_vel << wheel_theta_dot_fl_ew, wheel_theta_dot_rl_ew, wheel_theta_dot_rr_ew, wheel_theta_dot_fr_ew;
   
      Eigen::VectorXd V = jacobian_pinv * chassis_vel;

      thetalist << atan(V(1)/V(0)), atan(V(3)/V(2)), atan(V(5)/V(4)), atan(V(7)/V(6));

      joint_vel << V(0)/cos(thetalist(0)), V(2)/cos(thetalist(1)), V(4)/cos(thetalist(2)), V(6)/cos(thetalist(3));

      velocity1.data =  joint_vel(0);
      velocity2.data =  -joint_vel(1);
      velocity3.data =  -joint_vel(2);
      velocity4.data =  -joint_vel(3);

      theta1.data = thetalist(0);
      theta2.data = thetalist(1);
      theta3.data = thetalist(2);
      theta4.data = thetalist(3);

      fl_velocity_pub.publish(velocity1);
      rl_velocity_pub.publish(velocity2);
      rr_velocity_pub.publish(velocity3);
      fr_velocity_pub.publish(velocity4);

      fl_rotation_pub.publish(theta1);
      rl_rotation_pub.publish(theta2);
      rr_rotation_pub.publish(theta3);
      fr_rotation_pub.publish(theta4);
      
      // ROS_WARN_STREAM( velocity1 << ", " << velocity2 << ", " << velocity3 << ", " << velocity4);
      // ROS_WARN_STREAM( theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4);

      std::cout << "vel" << std::endl << chassis_vel << std::endl;
      loop_rate.sleep();
      
    }
    
  }



private: 
  ros::NodeHandle n_; 
  ros::Subscriber force_sub;
  ros::Subscriber state_sub;
  ros::Subscriber joint_state_sub;
  ros::Publisher fl_velocity_pub;
  ros::Publisher fr_velocity_pub;
  ros::Publisher rl_velocity_pub;
  ros::Publisher rr_velocity_pub;

  ros::Publisher fl_rotation_pub;
  ros::Publisher fr_rotation_pub;
  ros::Publisher rl_rotation_pub;
  ros::Publisher rr_rotation_pub;

  Eigen::VectorXd Force = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd chassis_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);
  
  Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(4);

  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(4);


  Eigen::MatrixXd jacobian_pinv = Eigen::MatrixXd::Zero(8,3);

  double roll_actual, pitch_actual, yaw_actual;
  double dt = 0.01;
  double M,B,K;

  std_msgs::Float64 velocity1;
  std_msgs::Float64 velocity2;
  std_msgs::Float64 velocity3;
  std_msgs::Float64 velocity4;

  std_msgs::Float64 theta1;
  std_msgs::Float64 theta2;
  std_msgs::Float64 theta3;
  std_msgs::Float64 theta4;
 
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_node");
  ros::AsyncSpinner spinner(10);
  spinner.start();

  AdmittanceController admittance; 
  admittance.run();
  return 0;
}