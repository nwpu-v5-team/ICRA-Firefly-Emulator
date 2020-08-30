//
// Created by heihei on 2020/4/13.
//

#include "std_msgs/Bool.h"
#include <ros/ros.h>

#include "keyboard_ctrl.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "simu_shoot_node");
  ros::NodeHandle nh;

  ros::Publisher shoot_pub_1_ = nh.advertise<std_msgs::Bool>("/blue1/shoot", 1);
  ros::Publisher shoot_pub_2_ = nh.advertise<std_msgs::Bool>("/blue2/shoot", 1);

  ros::Rate loop_rate(20.0);

  int key_input = 0;

  init_keyboard();

  std::cout << "Copyright (c) 2019 Kehan.Xue." << std::endl
            << "All rights reserved.         " << std::endl
            << std::endl;

  std::cout << "You can put 'c' to quit!" << std::endl;

  while (ros::ok() && key_input != 'c') {
    ros::spinOnce();

    if (kbhit()) {
      key_input = readch();
      std::cout << "Your input:" << (char)key_input << '\n';

      switch (key_input) {
      case 'u': {
        std_msgs::Bool shoot;
        shoot.data = true;
        shoot_pub_1_.publish(shoot);
        break;
      }

      case 'i': {
        std_msgs::Bool shoot;
        shoot.data = true;
        shoot_pub_2_.publish(shoot);
        break;
      }

      default:
        break;
      }
    }

    loop_rate.sleep();
  }
  close_keyboard();

  return 0;
}
