#ifndef ACTION_HPP
#define ACTION_HPP

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>

//for showing grasps
#include <moveit_visual_tools/moveit_visual_tools.h>

// Forward kinematics to have final pose of trajectory
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/postures.hpp"

#define FLAG_NO_MOVE        1
#define FLAG_MOVE           2

namespace moveit_simple_actions
{

class Action
{
public:
  Action(ros::NodeHandle *nh_,
         moveit_visual_tools::MoveItVisualToolsPtr &visual_tools,
         const std::string arm_,
         const std::string robot_name);

  //pick an object with a grasp generator
  bool pickAction(MetaBlock *block, const std::string surface_name,
                  const int attempts_nbr=0,
                  const double planning_time=0.0);

  //pick an object
  bool placeAction(MetaBlock *block, const std::string surface_name);

  //pick an object without a grasp generator
  bool pickDefault(MetaBlock *block, const std::string surface_name);

  //reaching positions generated by moveit simple grasps

  //plan a motion trajectory = computePlanButtonClicked in MoveIt
  bool graspPlan(MetaBlock *block, const std::string surface_name);

  //plan and show all possible motion trajectories
  bool graspPlanAllPossible(MetaBlock *block, const std::string surface_name);

  //execute the planned motion trajectory
  bool executeAction();

  //reaching default grasping pose
  float reachGrasp(MetaBlock *block, const std::string surface_name);

  //reaching the pre-grasp pose
  bool reachPregrasp(geometry_msgs::Pose pose_target, const std::string surface_name);

  //reaching the top of an object
  bool reachAction(geometry_msgs::Pose pose_target, const std::string surface_name="");

  //go to the pose
  bool poseHand(const int pose_id);

  //get the current pose
  geometry_msgs::Pose getPose();

  void poseHandOpen();
  void poseHandClose();
  bool poseHeadDown();
  bool poseHeadZero();

  //set the tolerance
  void setTolerance(const double value);

  //detach the collision object
  void detachObject(const std::string &block_name);

  //attach the collision object
  void attachObject(const std::string &block_name);

  //get the base_link
  std::string getBaseLink();

  //the current arm name
  const std::string arm_;

private:
  void publishPlanInfo(moveit::planning_interface::MoveGroup::Plan plan, geometry_msgs::Pose pose_target);

  void setPlanningTime(const double value);
  void setToleranceStep(const double value);
  void setToleranceMin(const double value);
  void setMaxVelocityScalingFactor(const double value);
  void setVerbose(bool verbose);
  void setAttemptsMax(int value);
  void setFlag(int flag);

  std::vector<moveit_msgs::Grasp> generateGrasps(MetaBlock *block);
  std::vector<geometry_msgs::Pose> configureForPlanning(const std::vector<moveit_msgs::Grasp> &grasps);

  //active end effector
  const std::string end_eff_;

  //the name of planning group
  const std::string plan_group_;

  Posture posture_;

  //grasp configuration
  moveit_simple_grasps::GraspData grasp_data_;

  //interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  //grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  //for planning actions
  boost::shared_ptr<moveit::planning_interface::MoveGroup::Plan> current_plan_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  //publisher for object poses
  ros::Publisher pub_obj_pose, pub_obj_poses;

  //publish final pose of trajectory
  ros::Publisher pub_plan_pose_;
  ros::Publisher pub_plan_traj_;
  ros::ServiceClient client_fk_;

  bool verbose_;

  //number of attempts when doing an approximate action
  int attempts_max_;

  //planning time
  double planning_time_;

  //planning library
  std::string planner_id_;

  //minimum tolerance to reach
  double tolerance_min_;

  //the tolerance step to vary
  double tolerance_step_;

  //maximum velocity factor
  double max_velocity_scaling_factor_;

  int flag_;
};



}

#endif // ACTION_HPP
