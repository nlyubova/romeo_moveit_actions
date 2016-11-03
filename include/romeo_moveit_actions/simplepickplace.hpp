#ifndef SIMPLEACTIONS_HPP
#define SIMPLEACTIONS_HPP

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <cstdlib>
//#include <mutex>          // std::mutex

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/action.hpp"
#include "romeo_moveit_actions/objprocessing.hpp"
#include "romeo_moveit_actions/evaluation.hpp"

namespace moveit_simple_actions
{

class SimplePickPlace
{
public:
  SimplePickPlace();

  //! @brief main cycle
  void run();

protected:
  //! @brief load params
  void loadParams();

  //! @brief create a table object
  MetaBlock createTable();

  //! @brief switch between the left and right arms
  void switchArm(Action **action_now);

  //! @brief create and publish an object
  void createObj(const MetaBlock &block);

  //! @brief publish the object at new position
  void resetBlock(MetaBlock *block);

  //! @brief get collision objects from the topic /collision_object
  void getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);

  //! @brief clean the object list based on the timestamp
  void cleanObjects(std::vector<MetaBlock> *objects,
                    const bool list_erase=true);

  //! @brief check if the block exists
  bool checkObj(int &block_id);

  //! @brief publish all collision blocks in MoveIt
  void publishAllCollObj(std::vector<MetaBlock> *blocks);

  //! @brief move closer to the object
  void moveToObject(MetaBlock *block, const std::string &plan_group);

  //! @brief move to direction
  void moveTo(geometry_msgs::Twist *msg_twist);

  //node handle
  ros::NodeHandle nh_, nh_priv_;

  //robot's name
  std::string robot_name_;

  //verbosity
  bool verbose_;

  //robot's base_frame
  std::string base_frame_;

  //dimenssion x of a default object
  double block_size_x_;

  //dimenssion y of a default object
  double block_size_y_;

  //shift of the robot's base to teh floor
  float floor_to_base_height_;

  //allowing to use wheels
  bool use_wheels_;

  //object processing
  Objprocessing objproc_;

  //evaluation of reaching/grasping
  Evaluation evaluation_;

  //state of re-drawing the world
  bool env_shown_;

  //name of the current support surface
  std::string support_surface_;

  //instance of an Action class for the left arm
  Action *action_left_;

  //instance of an Action class for the right arm
  Action *action_right_;

  //visual tools pointer used for scene visualization
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  //current MoveIt scene
  moveit::planning_interface::PlanningSceneInterface current_scene_;

  //set of available objects
  std::vector<MetaBlock> blocks_;

  //set of available surfaces
  std::vector<MetaBlock> blocks_surfaces_;

  //subscriber to get objects from /collision_object
  ros::Subscriber sub_obj_coll_;

  //publisher of objects poses
  ros::Publisher pub_obj_poses_;

  //publisher of the current object pose
  ros::Publisher pub_obj_pose_;

  //publisher of the currect target pose
  ros::Publisher pub_target_pose_;

  //publisher of collision objects to /collision_world
  ros::Publisher pub_obj_moveit_;

  //publisher of velocity
  ros::Publisher pub_cmd_;

  //current object position
  geometry_msgs::PoseStamped msg_obj_pose_;

  //all objects positions
  geometry_msgs::PoseArray msg_obj_poses_;

  //default object pose for the left arm
  geometry_msgs::Pose pose_default_;

  //default object pose for the right arm
  geometry_msgs::Pose pose_default_r_;

  //default object pose at zero
  geometry_msgs::Pose pose_zero_;

  //all successfully reached positions
  std::vector <geometry_msgs::Pose> stat_poses_success_;

  //optimal base_link pose to easier grasp objects
  tf::Stamped<tf::Pose> pose_optimal_;

  //transform listener
  tf::TransformListener listener_;

  //processing rate
  ros::Rate rate_;
};
}

#endif // SIMPLEACTIONS_HPP
