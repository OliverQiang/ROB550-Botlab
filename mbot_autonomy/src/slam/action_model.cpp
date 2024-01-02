#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.005f)
, min_dist_(0.000)
, min_theta_(0.000)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        resetPrevious(odometry);
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = odometry.theta -  previousOdometry_.theta;

    dx_ = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    dtheta_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    
    float direction = 1.0;
    // printf("detha_ before %f \n ", dtheta_);


    if (std::abs(dtheta_) > M_PI/2.0){
        dtheta_ = angle_diff(M_PI, dtheta_);
        direction = -1.0;
    }
    // printf("detha_ after %f \n ", dtheta_);
    
    dtheta2_ =  angle_diff(deltaTheta, dtheta_);

    // moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);
    moved_ = (dx_ > min_dist_) || (std::abs(deltaTheta) > min_theta_); 
    
    if (moved_){
        thetaStd_ = std::sqrt(k1_* std::abs(dtheta_) * std::abs(dtheta_));
        xStd_ =  std::sqrt(k2_ * std::abs(dx_) * std::abs(dx_));
        thetaStd2_ = std::sqrt(k1_ * std::abs(dtheta2_) * std::abs(dx_));

        // thetaStd_ = (k1_* std::abs(dtheta_));
        // xStd_ =  (k2_ * std::abs(dx_));
        // thetaStd2_ = (k1_ * std::abs(dtheta2_));

        // thetaStd_ = std::sqrt(k1_* std::abs(dtheta_));
        // xStd_ =  std::sqrt(k2_ * std::abs(dx_));
        // thetaStd2_ = std::sqrt(k1_ * std::abs(dtheta2_));
    }

    dx_ *= direction;

    previousOdometry_ = odometry; 
    utime_ = odometry.utime;

    // printf("dx_ %f \n", dx_);
    return moved_;

    // return false;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    // mbot_lcm_msgs:: pose2D_t parent = sample.pose;

    float sampledRot1 = std::normal_distribution<>(dtheta_, thetaStd_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(dx_, xStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(dtheta2_, thetaStd2_)(numberGenerator_);
    
    newSample.pose.x += sampledTrans*cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans*sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);

    newSample.weight = sample.weight;
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;
 
    // printf("new sample from apply action %f %f %f \n",newSample.pose.x, newSample.pose.y, newSample.pose.theta);
    return newSample;
}








