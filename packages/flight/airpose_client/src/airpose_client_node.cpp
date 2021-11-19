//
// Created by glawless on 01.09.17.
//

#include <neural_network_detector/NNDetector.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "airpose_client");

  if (argc < 3) {
    ROS_WARN("Usage: rosrun airpose_client neural_network_detector_node HOST PORT");
    return EXIT_FAILURE;
  }

  neural_network_detector::NNDetector nn_detector(argv[1], argv[2]);

  ros::spin();

  return EXIT_SUCCESS;
}