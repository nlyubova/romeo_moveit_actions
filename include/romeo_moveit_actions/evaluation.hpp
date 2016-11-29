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

#ifndef EVALUATION_HPP
#define EVALUATION_HPP

#include <ros/ros.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/action.hpp"

typedef shape_msgs::SolidPrimitive sprimitive;
typedef moveit_visual_tools::MoveItVisualToolsPtr mvistools;

namespace moveit_simple_actions
{

class Evaluation
{
public:

  Evaluation(ros::NodeHandle *nh,
             const bool &verbose,
             const std::string &base_frame);

  //! @brief initialization
  void init(const double &block_size_x,
            const double &block_size_y,
            const double floor_to_base_height,
            Action *action_left,
            Action *action_right);

  //! @brief testing grasping or approximate grasping
  void testReach(ros::NodeHandle &nh,
                 ros::Publisher *pub_obj_pose,
                 ros::Publisher *pub_obj_poses,
                 ros::Publisher *pub_obj_moveit,
                 const bool pickVsReach,
                 const bool test_poses_rnd=false);

  //! @brief printing the successfully reached positions
  void printStat();

  //! @brief checking if the pose is within the working space (close enough)
  bool inWorkSpace(geometry_msgs::Pose pose,
                   const bool x=true,
                   const bool y=true,
                   const bool z=true);

  //! @brief getting the X size of the working space
  float getXmax();

  //! @brief getting the Y size of the working space
  float getYmax();

protected:
  //! @brief generating test poses in a regular maner
  geometry_msgs::PoseArray generatePosesGrid();

  //! @brief generating poses in a random maner
  geometry_msgs::PoseArray generatePosesRnd(const int poses_nbr);

  //! @brief testing a single hand
  int testReachSingleHand(Action *action,
                          ros::Publisher *pub_obj_pose,
                          ros::Publisher *pub_obj_poses,
                          ros::Publisher *pub_obj_moveit,
                          const bool pickVsReach,
                          geometry_msgs::PoseArray &poses_validated);

  //! @brief printing the test poses
  void printStat(const geometry_msgs::PoseArray &poses,
                 const int &targets_nbr);

  //pointer to the action class for the left arm
  Action *action_left_;

  //pointer to the action class for the right arm
  Action *action_right_;

  //verbose or not
  bool verbose_;

  //robot's base_frame
  std::string base_frame_;

  //step to test the working space
  float test_step_;

  //number of attempts
  int attempts_nbr_;

  //planning time
  double planning_time_;

  //size X of a default object
  double block_size_x_;

  //size Y of a default object
  double block_size_y_;

  //shift of the robot's base to teh floor
  double floor_to_base_height_;

  /** working space in X dim min */
  float x_min_;
  /** working space in X dim max */
  float x_max_;
  /** working space in Y dim min */
  float y_min_;
  /** working space in Y dim max */
  float y_max_;
  /** working space in Z dim min */
  float z_min_;
  /** working space in Z dim max */
  float z_max_;

  //default zero pose
  geometry_msgs::Pose pose_zero_;

  //successfully reached positions
  geometry_msgs::PoseArray poses_success_;

  //total targets number
  int targets_nbr_;

  //default object to grasp
  MetaBlock *block_;

  //default table
  MetaBlock *table_;
};
}
#endif // EVALUATION_HPP
