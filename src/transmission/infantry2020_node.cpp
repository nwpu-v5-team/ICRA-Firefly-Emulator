//
// Created by heihei on 2020/3/4.
//

#include "infantry2020_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "icra_robomaster_emulator_node");
  Infantry2020Publisher infantry_2020_publisher;

  ros::spin();
  return 0;
}