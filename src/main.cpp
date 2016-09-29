#include "romeo_moveit_actions/simplepickplace.hpp"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "moveit_simple_action");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  moveit_simple_actions::SimplePickPlace server_pickplace;

  server_pickplace.run();

  ROS_INFO_STREAM_NAMED("moveit_simple_action", "Shutting down.");

  spinner.stop();
  ros::shutdown();

  return 0;
}
