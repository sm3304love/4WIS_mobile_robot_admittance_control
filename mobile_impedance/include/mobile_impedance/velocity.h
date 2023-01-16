#pragma once

#include <eigen3/Eigen/Dense>


#define PI 3.141592


//wheel distance
double a = 0.3;
double b = 0.3;//300mm?

// each wheel angle
double wheel_theta_fl = 0;
double wheel_theta_rl = 0;
double wheel_theta_rr = 0;
double wheel_theta_fr = 0;



double act_angle = 0;
// model position & orientation
double Vx  = 0;
double Vy  = 0;


Eigen::Vector3d Pos; 

// wheel position
double x_w_1 = a;
double x_w_2 = -a;
double x_w_3 = -a;
double x_w_4 = a;

double y_w_1 = b;
double y_w_2 = b;
double y_w_3 = -b;
double y_w_4 = -b;

double r_inv = 1/0.173;

// wheel position matrix 
Eigen::MatrixXd fl(1,2);
Eigen::MatrixXd rl(1,2);
Eigen::MatrixXd rr(1,2);
Eigen::MatrixXd fr(1,2);

// position matrix 
Eigen::MatrixXd P(8,3);
Eigen::MatrixXd X(8,4);
Eigen::MatrixXd Xp(8,4);


double wheel_theta_dot_fl_ew = 0;
double wheel_theta_dot_rl_ew = 0;
double wheel_theta_dot_rr_ew = 0;
double wheel_theta_dot_fr_ew = 0;

double wheel_theta_dot_fl_rot = 0;
double wheel_theta_dot_rl_rot = 0;
double wheel_theta_dot_rr_rot = 0;
double wheel_theta_dot_fr_rot = 0;
