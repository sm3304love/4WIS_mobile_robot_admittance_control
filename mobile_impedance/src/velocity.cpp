#include "mobile_impedance/velocity.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;

void matrix() 
{
// W1
 fl(0,0) = x_w_1;
 fl(0,1) = y_w_1;

// W2
 rl(0,0) = x_w_2;
 rl(0,1) = y_w_2;

// W3
 rr(0,0) = x_w_3;
 rr(0,1) = y_w_3;

// W4
 fr(0,0) = x_w_4;
 fr(0,1) = y_w_4;

 Pos << Vx, Vy, act_angle;


} ;




