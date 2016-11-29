/*
 * Copyright 2016 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

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
