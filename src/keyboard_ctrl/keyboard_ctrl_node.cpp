#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "keyboard_ctrl.hpp"

geometry_msgs::Twist set_cmd_vel;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Twist cur_cmd_vel;

void current_status_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pose.pose.position.x = msg->pose.pose.position.x;
  current_pose.pose.position.y = msg->pose.pose.position.y;
  current_pose.pose.position.z = msg->pose.pose.position.z;

  cur_cmd_vel = msg->twist.twist;
}

void output_cmd_vel(const geometry_msgs::Twist _twist) {
  std::cout << "linear: x=" << _twist.linear.x << " y=" << _twist.linear.y
            << " z=" << _twist.linear.z << std::endl;
  std::cout << "angular: z=" << _twist.angular.z << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_ctrl_node");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>(
      "ground_truth/state", 1, current_status_cb);

  ros::Rate loop_rate(20.0);

  int key_input = 0;

  ros::Time last_request = ros::Time::now();

  init_keyboard();

  std::cout << "Copyright (c) 2019 Kehan.Xue." << std::endl
            << "All rights reserved.         " << std::endl
            << std::endl;

  std::cout << "You can put 'c' to quit!" << std::endl;

  set_cmd_vel.linear.z = 0;
  set_cmd_vel.linear.y = 0;
  set_cmd_vel.linear.x = 0;

  set_cmd_vel.angular.x = 0;
  set_cmd_vel.angular.y = 0;
  set_cmd_vel.angular.z = 0;

  while (ros::ok() && key_input != 'c') {
    ros::spinOnce();

    if (kbhit()) {
      key_input = readch();
      std::cout << "Your input:" << (char)key_input << '\n';

      switch (key_input) {
      case 'w':
        set_cmd_vel.linear.x += 0.1;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'W':
        set_cmd_vel.linear.x = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      case 's':
        set_cmd_vel.linear.x -= 0.1;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'S':
        set_cmd_vel.linear.x = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'a':
        set_cmd_vel.linear.y += 0.1;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'A':
        set_cmd_vel.linear.y = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'd':
        set_cmd_vel.linear.y -= 0.1;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'D':
        set_cmd_vel.linear.y = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'q':
        set_cmd_vel.angular.z += 0.1;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'Q':
        set_cmd_vel.angular.z = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'e':
        set_cmd_vel.angular.z -= 0.1;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'E':
        set_cmd_vel.angular.z = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      case 'o':
        set_cmd_vel.linear.x = 0;
        set_cmd_vel.linear.y = 0;
        set_cmd_vel.linear.z = 0;
        set_cmd_vel.angular.x = 0;
        set_cmd_vel.angular.y = 0;
        set_cmd_vel.angular.z = 0;
        output_cmd_vel(set_cmd_vel);
        break;

      default:
        break;
      }
    }

    cmd_vel_pub.publish(set_cmd_vel);

    loop_rate.sleep();
  }
  close_keyboard();

  return 0;
}
