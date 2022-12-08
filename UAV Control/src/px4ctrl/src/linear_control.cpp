#include "linear_control.h"
#include <iostream>
#include <ros/ros.h>
#include <math.h>
using namespace std;
LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
  
      //compute disired acceleration
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
  
      //supposed to be readonly, compute thrust by acc 
      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
  
      //compute control attitude in the BODY frame
      u.q = Eigen::Quaterniond(1.0,0.0,0.0,0.0);
  /* WRITE YOUR CODE HERE */





  /* WRITE YOUR CODE HERE */
  //u.q=...;u.thrust=...;

  //位置控制：
  //先求Pi,c的二次导
  //a_c(0)=des.a(0)+gain.Kv0*(des.v(0)-odom.v(0))+gain.Kp0*(des.p(0)-odom.p(0));
  des_acc(0)=des.a(0)+1.5*(des.v(0)-odom.v(0))+0.5625*(des.p(0)-odom.p(0));
  des_acc(1)=des.a(1)+1.5*(des.v(1)-odom.v(1))+0.5625*(des.p(1)-odom.p(1));
  des_acc(2)=des.a(2)+1.5*(des.v(2)-odom.v(2))+0.5625*(des.p(2)-odom.p(2))+9.8;
  //求解推力u1
  u.thrust=computeDesiredCollectiveThrustSignal(des_acc);
  //求当前测量的angle,先x，再y，再z
  Eigen::Vector3d currunt_angle;
  currunt_angle(0)=atan2(2*(odom.q.w()*odom.q.x()+odom.q.y()*odom.q.z()),1-2*(odom.q.x()*odom.q.x()+odom.q.y()*odom.q.y()));
  currunt_angle(1)=asin(2*(odom.q.w()*odom.q.y()-odom.q.z()*odom.q.x()));
  currunt_angle(2)=atan2(2*(odom.q.w()*odom.q.z()+odom.q.y()*odom.q.x()),1-2*(odom.q.y()*odom.q.y()+odom.q.z()*odom.q.z()));
  //求angle(c),先x，再y，再z
  Eigen::Vector3d currunt_angle_c;
  currunt_angle_c(0)=(des_acc(0)*sin(currunt_angle(2))-des_acc(1)*cos(currunt_angle(2)))/9.8;
  currunt_angle_c(1)=(des_acc(0)*cos(currunt_angle(2))+des_acc(1)*sin(currunt_angle(2)))/9.8;
  currunt_angle_c(2)=0;

  //计算四元数，u.q
  u.q.w()=cos(currunt_angle_c(0)/2)*cos(currunt_angle_c(1)/2)*cos(currunt_angle_c(2)/2)+sin(currunt_angle_c(0)/2)*sin(currunt_angle_c(1)/2)*sin(currunt_angle_c(2)/2);
  u.q.x()=sin(currunt_angle_c(0)/2)*cos(currunt_angle_c(1)/2)*cos(currunt_angle_c(2)/2)-cos(currunt_angle_c(0)/2)*sin(currunt_angle_c(1)/2)*sin(currunt_angle_c(2)/2);
  u.q.y()=cos(currunt_angle_c(0)/2)*sin(currunt_angle_c(1)/2)*cos(currunt_angle_c(2)/2)+sin(currunt_angle_c(0)/2)*cos(currunt_angle_c(1)/2)*sin(currunt_angle_c(2)/2);
  u.q.z()=cos(currunt_angle_c(0)/2)*cos(currunt_angle_c(1)/2)*sin(currunt_angle_c(2)/2)-sin(currunt_angle_c(0)/2)*sin(currunt_angle_c(1)/2)*cos(currunt_angle_c(2)/2);
  
  //Eigen::Quaterniond q_east;
  //toRotationMatrix();Eigen::Vector3d
  //进行姿态转换
  Eigen::Matrix3d q_odom;
  Eigen::Matrix3d q_imu;
  Eigen::Matrix3d q_des;
  Eigen::Matrix3d q_east;
  q_des=u.q.toRotationMatrix();
  q_odom=odom.q.toRotationMatrix();
  q_imu=imu.q.toRotationMatrix();
  q_east=q_imu*q_odom.inverse()*q_des;
  u.q=q_east;
  // cout << u.q << endl;





  //used for debug
  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

