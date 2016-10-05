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
  Action(ros::NodeHandle *nh,
         const std::string &arm,
         const std::string &hand,
         const std::string &robot_name);

  //! @brief initialize the visual tools
  void initVisualTools(moveit_visual_tools::MoveItVisualToolsPtr &visual_tools);

  //! @brief pick an object with a grasp generator
  bool pickAction(MetaBlock *block,
                  const std::string surface_name,
                  const int attempts_nbr=0,
                  const double planning_time=0.0);

  //! @brief pick an object
  bool placeAction(MetaBlock *block,
                   const std::string surface_name);

  //! @brief pick an object without a grasp generator
  bool pickDefault(MetaBlock *block,
                   const std::string surface_name);

  //! @brief plan a trajectory = computePlanButtonClicked in MoveIt
  bool graspPlan(MetaBlock *block,
                 const std::string surface_name);

  //! @brief plan and show all possible motion trajectories
  bool graspPlanAllPossible(MetaBlock *block,
                            const std::string surface_name);

  //! @brief execute the planned motion trajectory
  bool executeAction();

  //! @brief reach default grasping pose
  bool reachGrasp(MetaBlock *block,
                  const std::string surface_name,
                  const int attempts_nbr=0,
                  const double planning_time=0.0);

  //! @brief reach the top of an object
  bool reachAction(geometry_msgs::Pose pose_target,
                   const std::string surface_name="",
                   const int attempts_nbr=1);

  //! @brief go to the pose
  bool poseHand(const int pose_id);

  //! @brief get the current pose
  geometry_msgs::Pose getPose();

  //! @brief go to the pose that opens hand
  void poseHandOpen();

  //! @brief go to the pose that closes hand
  void poseHandClose();

  //! @brief go to the pose that moves the head down
  bool poseHeadDown();

  //! @brief go to the pose that moves the head to zero
  bool poseHeadZero();

  //! @brief set the tolerance
  void setTolerance(const double value);

  //! @brief detach the collision object
  void detachObject(const std::string &block_name);

  //! @brief attach the collision object
  void attachObject(const std::string &block_name);

  //! @brief get the base_link
  std::string getBaseLink();

  //! @brief compute the distance to the object
  float computeDistance(MetaBlock *block);

  //! @brief compute the distance to the object
  float computeDistance(MetaBlock *block,
                        const bool &x,
                        const bool &y,
                        const bool &z);

  //! @brief get the grasping pose for the object pose
  geometry_msgs::Pose objGraspPose(MetaBlock *block);

  //! @brief check if the correct arm is used
  bool checkArm(const int &hand_id,
                const double &obj_y=0);

  //! @brief release the object
  void releaseObject(MetaBlock *block);

  //! @brief set support surface name
  void setSurface(const std::string &support_surface);

  //! @brief lift the arm
  bool lift();

  //name of the planning group
  const std::string plan_group_;

private:
  void publishPlanInfo(moveit::planning_interface::MoveGroup::Plan plan,
                       geometry_msgs::Pose pose_target);

  //! @brief set the planning time
  void setPlanningTime(const double value);

  //! @brief set the tolerance step
  void setToleranceStep(const double value);

  //! @brief set the minimum tolerance
  void setToleranceMin(const double value);

  //! @brief set the maximum velocity scaling factor
  void setMaxVelocityScalingFactor(const double value);

  //! @brief set verbosity
  void setVerbose(bool verbose);

  //! @brief set the maximum number of attempts
  void setAttemptsMax(int value);

  //! @brief set the flag if action execution is allowed
  void setFlag(int flag);

  //! @brief generate all possible grasps
  std::vector<moveit_msgs::Grasp> generateGrasps(MetaBlock *block);

  //! @brief configure for planning
  std::vector<geometry_msgs::Pose> configureForPlanning(
      const std::vector<moveit_msgs::Grasp> &grasps);

  //active end effector
  const std::string end_eff_;

  Posture posture_;

  //grasp configuration
  moveit_simple_grasps::GraspData grasp_data_;

  //interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  //grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  //for planning actions
  boost::shared_ptr<moveit::planning_interface::MoveGroup::Plan> current_plan_;

  //visual tools pointer used for scene visualization
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  //the current MoveIt scene
  moveit::planning_interface::PlanningSceneInterface current_scene_;

  //publisher for object poses
  ros::Publisher pub_obj_pose, pub_obj_poses;

  //publish final pose of trajectory
  ros::Publisher pub_plan_pose_;
  ros::Publisher pub_plan_traj_;
  ros::ServiceClient client_fk_;

  bool verbose_;

  //maximum number of attempts to do action
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

  //the the distance to object allowing to grasp it
  float dist_th_;

  //flag to allow motion
  int flag_;

  //the name of the support surface
  std::string support_surface_;

  //the current attached object
  std::string object_attached_;
};



}

#endif // ACTION_HPP
