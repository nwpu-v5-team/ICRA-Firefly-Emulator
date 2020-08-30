//
// Created by heihei on 2020/3/4.
//

#ifndef INFANTRY2020_INFANTRY2020_NODE_H_
#define INFANTRY2020_INFANTRY2020_NODE_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "roborts_msgs/GimbalAngle.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class Infantry2020Publisher {
public:
  Infantry2020Publisher() {

    ros::NodeHandle nh1("~");

    infantry_cmd_vel_pub_ =
        nh1.advertise<geometry_msgs::Twist>("gazebo_cmd_vel", 1);

    infantry_gimbal_yaw_pub_ =
        nh1.advertise<std_msgs::Float64>("yaw_position_controller/command", 1);
    infantry_gimbal_pitch_pub_ = nh1.advertise<std_msgs::Float64>(
        "pitch_position_controller/command", 1);

    infantry_cmd_vel_sub_ = nh1.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 1, &Infantry2020Publisher::CmdVelCallBack, this);

    infantry_gimbal_pose_sub_ = nh1.subscribe<roborts_msgs::GimbalAngle>(
        "cmd_gimbal_angle", 1, &Infantry2020Publisher::GimbalAngleCallBack,
        this);

    infantry_gimbal_now_pose_sub_ = nh1.subscribe<sensor_msgs::JointState>(
        "joint_states", 1, &Infantry2020Publisher::GimbalNowPose, this);

    chassis_red1_pose_sub_ = nh1.subscribe<nav_msgs::Odometry>(
        "/red1/ground_truth/state", 1, &Infantry2020Publisher::Red1PoseCallBack,
        this);
    chassis_red2_pose_sub_ = nh1.subscribe<nav_msgs::Odometry>(
        "/red2/ground_truth/state", 1, &Infantry2020Publisher::Red2PoseCallBack,
        this);
    chassis_blue1_pose_sub_ = nh1.subscribe<nav_msgs::Odometry>(
        "/blue1/ground_truth/state", 1,
        &Infantry2020Publisher::Blue1PoseCallBack, this);
    chassis_blue2_pose_sub_ = nh1.subscribe<nav_msgs::Odometry>(
        "/blue2/ground_truth/state", 1,
        &Infantry2020Publisher::Blue2PoseCallBack, this);

    chassis_red1_pose_pub_ =
        nh1.advertise<geometry_msgs::PoseStamped>("/red1/true_pose", 1);
    chassis_red2_pose_pub_ =
        nh1.advertise<geometry_msgs::PoseStamped>("/red2/true_pose", 1);
    chassis_blue1_pose_pub_ =
        nh1.advertise<geometry_msgs::PoseStamped>("/blue1/true_pose", 1);
    chassis_blue2_pose_pub_ =
        nh1.advertise<geometry_msgs::PoseStamped>("/blue2/true_pose", 1);
  }

  ~Infantry2020Publisher() = default;

  void CmdVelCallBack(const geometry_msgs::Twist::ConstPtr &msg) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular = msg->angular;
    cmd_vel.linear = msg->linear;
    cmd_vel.linear.z = -cmd_vel.linear.z;
    cmd_vel.linear.y = -cmd_vel.linear.y;
    cmd_vel.linear.x = -cmd_vel.linear.x;

    cmd_vel.linear.x = cmd_vel.linear.x;
    cmd_vel.linear.y = cmd_vel.linear.y;
    cmd_vel.linear.z = cmd_vel.linear.z;
    cmd_vel.angular.x = cmd_vel.angular.x;
    cmd_vel.angular.y = cmd_vel.angular.y;
    cmd_vel.angular.z = cmd_vel.angular.z;

    infantry_cmd_vel_pub_.publish(cmd_vel);
  }

  void GimbalAngleCallBack(const roborts_msgs::GimbalAngle::ConstPtr &msg) {
    if (msg->pitch_mode == 0) {
      std_msgs::Float64 angle;
      angle.data = msg->pitch_angle;
      infantry_gimbal_pitch_pub_.publish(angle);
    } else if (msg->pitch_mode == 1) {
      std_msgs::Float64 angle;
      angle.data = msg->pitch_angle + gimbal1_now_pitch;
      infantry_gimbal_pitch_pub_.publish(angle);
    } else {
      ROS_WARN("GimbalAngle 1 doesn't have mode pitch");
    }

    if (msg->yaw_mode == 0) {
      std_msgs::Float64 angle;
      angle.data = msg->yaw_angle;
      infantry_gimbal_yaw_pub_.publish(angle);
    } else if (msg->yaw_mode == 1) {
      std_msgs::Float64 angle;
      angle.data = msg->yaw_angle + gimbal1_now_yaw;
      infantry_gimbal_yaw_pub_.publish(angle);
    } else {
      ROS_WARN("GimbalAngle 1 doesn't have mode yaw");
    }
  }

  void GimbalNowPose(const sensor_msgs::JointState::ConstPtr &msg) {
    gimbal1_now_pitch = msg->position[4];
    gimbal1_now_yaw = msg->position[5];
  }

  void Red1PoseCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation = msg->pose.pose.orientation;
    pose.pose.position = msg->pose.pose.position;
    chassis_red1_pose_pub_.publish(pose);
  }

  void Red2PoseCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation = msg->pose.pose.orientation;
    pose.pose.position = msg->pose.pose.position;
    chassis_red2_pose_pub_.publish(pose);
  }

  void Blue1PoseCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation = msg->pose.pose.orientation;
    pose.pose.position = msg->pose.pose.position;
    chassis_blue1_pose_pub_.publish(pose);
  }

  void Blue2PoseCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation = msg->pose.pose.orientation;
    pose.pose.position = msg->pose.pose.position;
    chassis_blue2_pose_pub_.publish(pose);
  }

private:
  ros::Publisher infantry_cmd_vel_pub_;

  ros::Publisher infantry_gimbal_yaw_pub_;
  ros::Publisher infantry_gimbal_pitch_pub_;

  ros::Subscriber infantry_cmd_vel_sub_;

  ros::Subscriber infantry_gimbal_pose_sub_;

  ros::Subscriber infantry_gimbal_now_pose_sub_;
  double gimbal1_now_yaw{};
  double gimbal1_now_pitch{};

  ros::Subscriber chassis_red1_pose_sub_;
  ros::Subscriber chassis_red2_pose_sub_;
  ros::Subscriber chassis_blue1_pose_sub_;
  ros::Subscriber chassis_blue2_pose_sub_;
  ros::Publisher chassis_red1_pose_pub_;
  ros::Publisher chassis_red2_pose_pub_;
  ros::Publisher chassis_blue1_pose_pub_;
  ros::Publisher chassis_blue2_pose_pub_;
};

#endif // INFANTRY2020_INFANTRY2020_NODE_H_
