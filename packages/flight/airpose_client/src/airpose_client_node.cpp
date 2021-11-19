//
// Created by glawless on 01.09.17.
//

#include <airpose_client/AirPoseClient.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "airpose_client");

  if (argc < 3) {
    ROS_WARN("Usage: rosrun airpose_client airpose_client_node HOST PORT");
    return EXIT_FAILURE;
  }

  airpose_client::AirPoseClient airPoseClient(argv[1], argv[2]);

  ros::spin();

  return EXIT_SUCCESS;
}