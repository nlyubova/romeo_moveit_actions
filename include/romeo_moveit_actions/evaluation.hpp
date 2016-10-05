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
  bool inWorkSpace(geometry_msgs::Pose pose);

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
  double test_step_;

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

  //working space of the robot
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

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
