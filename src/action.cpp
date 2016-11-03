#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/robot_state.h>

#include "romeo_moveit_actions/action.hpp"

namespace moveit_simple_actions
{

Action::Action(ros::NodeHandle *nh,
               const std::string &arm,
               const std::string &hand,
               const std::string &robot_name):
  verbose_(false),
  attempts_max_(3),
  planning_time_(20.0), //=5 in GUI
  planner_id_("RRTConnectkConfigDefault"), //RRTConnect
  tolerance_min_(0.04), //tested on Pepper, works from 0.03
  tolerance_step_(0.02),
  max_velocity_scaling_factor_(1.0), //=1.0 in GUI
  dist_th_(0.08f),
  flag_(FLAG_MOVE),
  end_eff_(hand),
  plan_group_(arm),
  posture_(robot_name, end_eff_, plan_group_)
{
  /* ROS_INFO_STREAM(<< "Planning group: " << plan_group_
                  << " end effector: " << end_eff_); */

  nh->getParam("tolerance_min", tolerance_min_);

  if (robot_name == "nao")
    planning_time_ = 30.0;

  // Create MoveGroup for the planning group
  move_group_.reset(new move_group_interface::MoveGroup(plan_group_));
  move_group_->setGoalTolerance(tolerance_min_);
  move_group_->setPlanningTime(planning_time_);
  move_group_->setPlannerId(planner_id_);
  move_group_->setNumPlanningAttempts(4);
  //move_group_->setGoalPositionTolerance(0.1); //0.0001
  //move_group_->setGoalOrientationTolerance(0.1); //0.001
  //move_group_->setMaxVelocityScalingFactor(1.0);

  // Load grasp generator
  if (!grasp_data_.loadRobotGraspData(*nh, end_eff_))
  {
    ROS_ERROR("The grasp data cannot be loaded");
    ros::shutdown();
  }

  //generate grasps at PI/angle_resolution increments
  grasp_data_.angle_resolution_ = 30;
  /* grasp depth, when <= 0 the grasps is from other side
   * of the object or from bottom */
  grasp_data_.grasp_depth_ = 0.01;
  //grasp_data_.approach_retreat_desired_dist_ = 0.3; //0.2 //as bigger better grasp
  //grasp_data_.approach_retreat_min_dist_ = 0.05; //0.06
  /*std::cout << "grasp_data.approach_retreat_desired_dist_=" << grasp_data_.approach_retreat_desired_dist_ << std::endl
               << " grasp_data.approach_retreat_min_dist_=" << grasp_data_.approach_retreat_min_dist_ << std::endl
               << " grasp_data_.grasp_depth_= " << grasp_data_.grasp_depth_ << std::endl;*/

  if (grasp_data_.pre_grasp_posture_.points.size() > 0)
    for (int i=0; i<grasp_data_.pre_grasp_posture_.joint_names.size(); ++i)
    {
      if (grasp_data_.pre_grasp_posture_.joint_names[i].find("Hand") != std::string::npos)
      {
        if (grasp_data_.pre_grasp_posture_.points.size() > 0)
          posture_.initHandPose(grasp_data_.pre_grasp_posture_.points.at(0).positions[i], 0);
      }
      if (grasp_data_.grasp_posture_.joint_names[i].find("Hand") != std::string::npos)
      {
        if (grasp_data_.grasp_posture_.points.size() > 0)
          posture_.initHandPose(grasp_data_.grasp_posture_.points.at(0).positions[i], 1);
      }
    }

  // Load Grasp generator
  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(moveit_visual_tools::MoveItVisualToolsPtr()));

  pub_obj_pose = nh->advertise<geometry_msgs::PoseStamped>("/pose_target", 10);
  pub_obj_poses = nh->advertise<geometry_msgs::PoseStamped>("/pose_targets", 10);

  pub_plan_pose_ = nh->advertise<geometry_msgs::PoseStamped>("/pose_plan", 10);
  pub_plan_traj_ = nh->advertise<moveit_msgs::RobotTrajectory>("/trajectory", 10);
  client_fk_ = nh->serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
}

void Action::initVisualTools(moveit_visual_tools::MoveItVisualToolsPtr &visual_tools)
{
  visual_tools_ = visual_tools;

  // Load Grasp generator
  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));
}

bool Action::pickDefault(MetaBlock *block,
                         const std::string surface_name)
{
  bool done = false;

  std::vector<moveit_msgs::Grasp> grasps(1);

  moveit_msgs::Grasp g;
  g.grasp_pose.header.frame_id = block->name_;
  g.grasp_pose.pose = block->pose_;
  g.grasp_pose.pose = grasp_data_.grasp_pose_to_eef_pose_;

  ROS_INFO_STREAM(" -- grasp_data_.ee_parent_link_ "<< grasp_data_.ee_parent_link_);
  g.pre_grasp_approach.direction.header.frame_id = grasp_data_.ee_parent_link_;
  g.pre_grasp_approach.direction.vector.x = 0;
  g.pre_grasp_approach.direction.vector.y = 0;
  g.pre_grasp_approach.direction.vector.z = -1;
  g.pre_grasp_approach.min_distance = 0.06; //0.01;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = grasp_data_.ee_parent_link_;
  g.post_grasp_retreat.direction.vector.x = 0;
  g.post_grasp_retreat.direction.vector.y = 0;
  g.post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
  g.post_grasp_retreat.min_distance = 0.06;
  g.post_grasp_retreat.desired_distance = 0.2;

  g.pre_grasp_posture.header.frame_id = grasp_data_.ee_parent_link_;
  g.grasp_posture.header.frame_id = grasp_data_.ee_parent_link_;
  if (grasp_data_.grasp_posture_.joint_names.size() > 0)
  {
    g.pre_grasp_posture.joint_names.resize(1);
ROS_INFO_STREAM( " -- " << grasp_data_.grasp_posture_.joint_names[0]);
    g.pre_grasp_posture.joint_names[0] = grasp_data_.grasp_posture_.joint_names[0];
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 0.0;

    g.grasp_posture.joint_names.resize(1);
ROS_INFO_STREAM(" -- " << g.pre_grasp_posture.joint_names[0]);
    g.grasp_posture.joint_names[0] = g.pre_grasp_posture.joint_names[0];
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 1.0;
  }
  std::cout << "-- pickDefault g " << g << std::endl;

    grasps[0] = g;

    // Prevent collision with table
    move_group_->setSupportSurfaceName(surface_name);

    /* an optional list of obstacles that
     * can be touched/pushed/moved in the course of grasping */
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(block->name_);

    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;

    if (move_group_->pick(block->name_, grasps)){
      done = true;

      return true;
    }
    sleep(0.7);
  //}
}

bool Action::executeAction()
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Execution of the planned action");

  if (!move_group_)
    return false;

  if (current_plan_ && flag_ == FLAG_MOVE)
    success = move_group_->execute(*current_plan_);

  if (verbose_ && success)
      ROS_INFO_STREAM("Execute success! \n\n");

  // If the function is called and FLAG_NO_MOVE return true
  // because only call executeAction if plan is success
  return success || flag_ == FLAG_NO_MOVE;
}

bool Action::graspPlan(MetaBlock *block, const std::string surface_name)
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Planning " << block->name_
                    << " at pose " << block->pose_);

  double tolerance_cur =  move_group_->getGoalPositionTolerance();
  move_group_->setGoalTolerance(0.1);//0.05

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  if (!move_group_)
    return false;

  //move_group_->setApproximateJointValueTargets(target, move_group_->getEndEffectorLink().c_str());
  move_group_->setPoseTargets(configureForPlanning(generateGrasps(block)),
                              move_group_->getEndEffectorLink().c_str());

  current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());
  success = move_group_->plan(*current_plan_);
  if (!success)
    current_plan_.reset();
  else
    publishPlanInfo(*current_plan_, block->pose_);

  if (verbose_ && success)
      ROS_INFO_STREAM("Grasp planning success! \n\n");

  move_group_->setGoalTolerance(tolerance_cur);

  return success;
}

geometry_msgs::PoseStamped Action::getGraspPose(MetaBlock *block)
{
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();

  goal.header.frame_id = block->base_frame_;
  goal.pose = block->pose_;
  goal.pose.position.x += grasp_data_.grasp_pose_to_eef_pose_.position.x;
  goal.pose.position.y += grasp_data_.grasp_pose_to_eef_pose_.position.y;
  goal.pose.position.z += grasp_data_.grasp_pose_to_eef_pose_.position.z;

  goal.pose.orientation = grasp_data_.grasp_pose_to_eef_pose_.orientation;

  if (block->base_frame_ == "base_link")
  {
    /*geometry_msgs::PoseStamped pose_transform = block->getTransformed(&listener_, "base_link");
    goal.header.frame_id = pose_transform.header.frame_id;

    goal.pose = pose_transform.pose;
    goal.pose.position.x += grasp_data_.grasp_pose_to_eef_pose_.position.x;
    goal.pose.position.y += grasp_data_.grasp_pose_to_eef_pose_.position.y;
    goal.pose.position.z += grasp_data_.grasp_pose_to_eef_pose_.position.z;*/
  }
  return goal;
}

float Action::computeDistance(MetaBlock *block)
{
  geometry_msgs::PoseStamped goal = getGraspPose(block);
  geometry_msgs::Pose current(move_group_->getCurrentPose().pose);

  float dist = sqrt((goal.pose.position.x - current.position.x)
                    * (goal.pose.position.x - current.position.x)
                    + (goal.pose.position.y - current.position.y)
                    * (goal.pose.position.y - current.position.y)
                    + (goal.pose.position.z - current.position.z)
                    * (goal.pose.position.z - current.position.z));
  return dist;
}

float Action::computeDistance(geometry_msgs::Pose goal)
{
  geometry_msgs::Pose current(move_group_->getCurrentPose().pose);

  /*ROS_INFO_STREAM("-- " << (goal.position.x - current.position.x) << " "
                  << (goal.position.y - current.position.y) << " "
                  << (goal.position.z - current.position.z) << " " );*/

  float dist = sqrt((goal.position.x - current.position.x)
                    * (goal.position.x - current.position.x)
                    + (goal.position.y - current.position.y)
                    * (goal.position.y - current.position.y)
                    + (goal.position.z - current.position.z)
                    * (goal.position.z - current.position.z));
  return dist;
}

bool Action::poseHeadZero()
{
  return posture_.poseHeadZero();
}

bool Action::poseHeadDown()
{
  return posture_.poseHeadDown();
}

bool Action::poseHand(const int pose_id)
{
  double tolerance_cur = move_group_->getGoalPositionTolerance();
  move_group_->setGoalTolerance(0.05);
  bool res = posture_.poseHand(end_eff_, plan_group_, pose_id);
  move_group_->setGoalTolerance(tolerance_cur);
  return res;
}

void Action::poseHandOpen()
{
  posture_.poseHandOpen(end_eff_);
}

void Action::poseHandClose()
{
  posture_.poseHandClose(end_eff_);
}

geometry_msgs::Pose Action::getPose()
{
  geometry_msgs::PoseStamped pose_now;
  pose_now.header.stamp = ros::Time::now();

  pose_now.pose = move_group_->getCurrentPose().pose;
  pose_now.header.frame_id = grasp_data_.base_link_;

  /*geometry_msgs::Pose pose = move_group_->getCurrentPose().pose;
  //ROS_INFO_STREAM("current pose is " << pose_now);
  if (pose.header.frame_id.compare(grasp_data_.base_link_) == 0)
    pose_now.pose = pose;
  else
  {
    tf::Stamped<tf::Pose> pose_tr = blocks_[block_id].getTransform(&listener_, grasp_data_.base_link_);
    pose_now.pose.position.x = pose_tr.getOrigin().x();
    pose_now.pose.position.y = pose_tr.getOrigin().y();
    pose_now.pose.position.z = pose_tr.getOrigin().z();
    pose_now.pose.orientation.x = pose_tr.getRotation().x;
    pose_now.pose.orientation.y = pose_tr.getRotation().y;
    pose_now.pose.orientation.z = pose_tr.getRotation().z;
    pose_now.pose.orientation.w = pose_tr.getRotation().w;
  }*/

  pose_now.pose.position.x -= grasp_data_.grasp_pose_to_eef_pose_.position.x;
  pose_now.pose.position.y -= grasp_data_.grasp_pose_to_eef_pose_.position.y;
  pose_now.pose.position.z -= grasp_data_.grasp_pose_to_eef_pose_.position.z;
  pose_now.pose.orientation = grasp_data_.grasp_pose_to_eef_pose_.orientation;
  //ROS_INFO_STREAM("current pose is " << pose_now);

  pub_obj_pose.publish(pose_now);

  return pose_now.pose;
}

void Action::setTolerance(const double value)
{
  move_group_->setGoalTolerance(value);
  if (verbose_)
    ROS_INFO_STREAM("The Goal Position Tolerance = "
                    << move_group_->getGoalPositionTolerance());
}

void Action::releaseObject(MetaBlock *block)
{
  move_group_->detachObject(block->name_);

  //remove the collision temporally
  std::vector<std::string> objects;
  objects.push_back(block->name_);
  current_scene_.removeCollisionObjects(objects);

  //go to the required pose
  poseHand(2);
  ros::Duration(1.0).sleep();
  poseHandOpen();

  //add the collision object back
  std::vector<moveit_msgs::CollisionObject> coll_objects;
  coll_objects.push_back(block->collObj_);
  current_scene_.addCollisionObjects(coll_objects);
}

bool Action::reachGrasp(MetaBlock *block,
                        const std::string surface_name,
                        int attempts_nbr,
                        double planning_time)
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Reaching at position = "
                    << block->pose_.position.x << " "
                    << block->pose_.position.y << " "
                    << block->pose_.position.z);

  if (attempts_nbr == 0)
    attempts_nbr = attempts_max_;

  if (planning_time == 0.0)
    planning_time = planning_time_;

  //clean object temporally or allow to touch it
  block->removeBlock(&current_scene_);
  ros::Duration(0.2).sleep();

  geometry_msgs::PoseStamped pose = getGraspPose(block);
sleep(1.0);

  if (!reachAction(pose, surface_name, attempts_nbr))
    return false;
sleep(8.0);
  //compute the distance to teh object
  float dist = computeDistance(pose.pose);
  //close the hand, if the object is close enough
  if (dist < dist_th_)
  {
    poseHandClose();
    success = true;
  }

  //publish the object
  block->publishBlock(&current_scene_);

  //attach the object
  if (success)
  {
    move_group_->attachObject(block->name_, grasp_data_.ee_parent_link_);
    object_attached_ = block->name_;
  }

  //if (verbose_)
    ROS_INFO_STREAM("Distance to the object= " << dist
                    << "; is grasped=" << success);

  return success;
}

bool Action::reachAction(geometry_msgs::PoseStamped pose_target,
                         const std::string surface_name,
                         const int attempts_nbr)
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Planning to the pose " << pose_target);

  if (!move_group_)
    return false;

  current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  move_group_->setPoseTarget(pose_target,
                             move_group_->getEndEffectorLink().c_str());

  //publish the target pose
  pub_obj_pose.publish(pose_target);
  /*sleep(0.5);
  pub_obj_pose.publish(move_group_->getPoseTarget());*/

  double tolerance = tolerance_min_;
  int attempts = 0;

  //find a planning solution while increasing tolerance
  while (!success && (attempts < attempts_nbr))
  {
    move_group_->setGoalTolerance(tolerance);
    success = move_group_->plan(*current_plan_);

    if (verbose_ && success)
      ROS_INFO_STREAM("Reaching success with tolerance " << tolerance << "\n\n");

    if (!success)
    {
      tolerance += tolerance_step_;
      //if (verbose_)
        ROS_INFO_STREAM("Planning retry with the tolerance " << tolerance);
    }
    ++attempts;
  }

  //find an approximate solution
  if (!success)
  {
    move_group_->setApproximateJointValueTarget(pose_target,
                                                move_group_->getEndEffectorLink().c_str());
    success = move_group_->plan(*current_plan_);
    //if (verbose_ && success)
      ROS_INFO_STREAM("Reaching success with approximate joint value");
  }

  if (success)
  {
    //publishPlanInfo(*current_plan_, pose_target.pose);
    success = executeAction();
  }else
    current_plan_.reset();

  return success;
}

bool Action::graspPlanAllPossible(MetaBlock *block,
                                  const std::string surface_name)
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Planning all possible grasps to " << block->pose_);

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  std::vector<geometry_msgs::Pose> targets =
      configureForPlanning(generateGrasps(block));

  if (targets.size() == 0)
    return false;

  moveit::planning_interface::MoveGroup::Plan plan;

  double tolerance_cur =  move_group_->getGoalPositionTolerance();
  move_group_->setGoalTolerance(0.1);

  int counts = 0;
  std::vector<geometry_msgs::Pose>::iterator it=targets.begin();
  for (; it!=targets.end();++it)
  {
    //move_group_->setPoseTarget(*it, move_group_->getEndEffectorLink().c_str());
    //move_group_->setPositionTarget(it->position.x, it->position.y, it->position.z, move_group_->getEndEffectorLink().c_str());
    success = move_group_->setApproximateJointValueTarget(*it, move_group_->getEndEffectorLink().c_str());
    success = move_group_->plan(plan);
    if (success)
      ++counts;
  }
  if (verbose_)
    ROS_INFO_STREAM( "Planning success for "
                     << counts << " generated poses! \n\n");

  move_group_->setGoalTolerance(tolerance_cur);

  return success;
}

std::vector<moveit_msgs::Grasp> Action::generateGrasps(MetaBlock *block)
{
  std::vector<moveit_msgs::Grasp> grasps;
  if (block->name_.empty())
  {
    ROS_INFO_STREAM("No object choosen to grasp");
    return grasps;
  }

  //if (verbose_)
    //visual_tools_->deleteAllMarkers();

  geometry_msgs::Pose pose = block->pose_;
//ROS_INFO_STREAM(" -- " << grasp_data_.base_link_);
  simple_grasps_->generateBlockGrasps(pose, grasp_data_, grasps);

  /*if (verbose_)
  {
    double speed = 0.01;
    visual_tools_->publishGrasps(grasps, speed); //grasp_data_.ee_parent_link_
    visual_tools_->deleteAllMarkers();
    sleep(0.5);
  }*/

  if (grasps.size() > 0)
  {
    /* an optional list of obstacles that we have semantic information about
     * and that can be touched/pushed/moved in the course of grasping */
    std::vector<std::string> allowed_touch_objects(1);
    allowed_touch_objects[0] = block->name_;
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;
  }

  return grasps;
}

std::vector<geometry_msgs::Pose> Action::configureForPlanning(
    const std::vector<moveit_msgs::Grasp> &grasps)
{
  std::vector<geometry_msgs::Pose> targets(grasps.size());

  if (grasps.size() > 0)
  {
    std::vector<moveit_msgs::Grasp>::const_iterator it_grasp = grasps.begin();
    std::vector<geometry_msgs::Pose>::iterator it_pose = targets.begin();
    for (; it_grasp!=grasps.end(); ++it_grasp, ++it_pose)
    {
      *it_pose = it_grasp->grasp_pose.pose;
    }
  }

  return targets;
}

bool Action::pickAction(MetaBlock *block, 
                        const std::string surface_name,
                        int attempts_nbr,
                        double planning_time)
{
  bool success(false);

  //if (verbose_)
    ROS_INFO_STREAM("Pick at pose "
                    << block->pose_.position.x << " "
                    << block->pose_.position.y << " "
                    << block->pose_.position.z);

  if (attempts_nbr == 0)
    attempts_nbr = attempts_max_;

  if (planning_time == 0.0)
    planning_time = planning_time_;

  std::vector<moveit_msgs::Grasp> grasps = generateGrasps(block);
  if (grasps.size() == 0)
    return false;

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  move_group_->setPlanningTime(planning_time);

  double tolerance = tolerance_min_;
  int attempts = 0;
  //find a planning solution while increasing tolerance
  while (!success && (attempts < attempts_nbr))
  {
    move_group_->setGoalTolerance(tolerance);
    success = move_group_->pick(block->name_, grasps);

    if (!success)
    {
      tolerance += tolerance_step_;

      if (verbose_)
        ROS_INFO_STREAM("Try planning with the tolerance " << tolerance);
    }
    ++attempts;
  }
  move_group_->setGoalTolerance(tolerance_min_);

  if (verbose_ & success)
    ROS_INFO_STREAM("Pick success with tolerance " << tolerance << "\n\n");

  return success;
}

bool Action::placeAction(MetaBlock *block,
                         const std::string surface_name)
{
  if (verbose_)
    ROS_INFO_STREAM("Placing " << block->name_
                    << " at pose " << block->goal_pose_);

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  std::vector<moveit_msgs::PlaceLocation> place_locations;

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = grasp_data_.base_link_;
  pose_stamped.header.stamp = ros::Time::now();

  // Create 360 degrees of place location rotated around a center
  //for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
  //{
    pose_stamped.pose = block->goal_pose_;

    // Create new place location
    moveit_msgs::PlaceLocation place_loc;
    place_loc.place_pose = pose_stamped;

    // Approach
    moveit_msgs::GripperTranslation pre_place_approach;
    pre_place_approach.direction.header.stamp = ros::Time::now();
    // The distance the origin of a robot link needs to travel
    pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_;
    // half of the desired? Untested.
    pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_;
    pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
    pre_place_approach.direction.vector.x = 0;
    pre_place_approach.direction.vector.y = 0;
    //Approach direction (negative z axis)  // TODO: document this assumption
    pre_place_approach.direction.vector.z = 0.1; //-1 //
    place_loc.pre_place_approach = pre_place_approach;

    // Retreat
    /*moveit_msgs::GripperTranslation post_place_retreat(pre_place_approach);
    post_place_retreat.direction.vector.x = 0;
    post_place_retreat.direction.vector.y = 0;
    post_place_retreat.direction.vector.z = -1; //1 // Retreat direction
    place_loc.post_place_retreat = post_place_retreat;*/
    // Post place posture - use same as pre-grasp posture (the OPEN command)
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

    place_locations.push_back(place_loc);
  //}

  //to test
  move_group_->setStartState(*move_group_->getCurrentState());

  bool success = move_group_->place(block->name_, place_locations);
  if (verbose_)
  {
    if (success)
      ROS_INFO_STREAM("Place success! \n\n");
    else
      ROS_ERROR_STREAM_NAMED("simple_actions:","Place failed.");
  }

  return success;
}

void Action::publishPlanInfo(moveit::planning_interface::MoveGroup::Plan plan,
                             geometry_msgs::Pose pose_target)
{
  // Get the last position of the trajectory plan and transform that joints values
  // into pose of end effector in /base_link frame
  // Then that is published to /pose_plan and the trajectory to /trajectory topics
  //TODO: Check if using directly robotStatePtr changes the real robot
  int num_points = plan.trajectory_.joint_trajectory.points.size();
  moveit::core::RobotStatePtr robotStatePtr = move_group_->getCurrentState();
  robotStatePtr->setJointGroupPositions(plan_group_, plan.trajectory_.joint_trajectory.points[num_points-1].positions);
  moveit_msgs::RobotState robotStateMsg;
  moveit::core::robotStateToRobotStateMsg(*robotStatePtr, robotStateMsg);

  std::vector<std::string> links_vect;
  links_vect.push_back(move_group_->getEndEffectorLink());

  moveit_msgs::GetPositionFK srv;
  srv.request.header.frame_id = "/base_link"; //grasp_data_.ee_parent_link_; //TODO: To check
  srv.request.fk_link_names = links_vect;
  srv.request.robot_state = robotStateMsg;

  if (client_fk_.call(srv))
  {
    if(srv.response.pose_stamped.size() > 0)
    {
      int eef_index = srv.response.fk_link_names.size() - 1;

      pub_plan_pose_.publish(srv.response.pose_stamped[eef_index]);

      if(verbose_)
      {
        // Compute the distance between the last pose of the trajectory plan
        // and the target pose
        double x_target = pose_target.position.x;
        double y_target = pose_target.position.y;
        double z_target = pose_target.position.z;

        double x_pose = srv.response.pose_stamped[eef_index].pose.position.x;
        double y_pose = srv.response.pose_stamped[eef_index].pose.position.y;
        double z_pose = srv.response.pose_stamped[eef_index].pose.position.z;

        double error = sqrt(pow(x_target-x_pose,2)
                            + pow(y_target-y_pose,2)
                            + pow(z_target-z_pose,2));

        ROS_INFO_STREAM("Distance of last trajectory pose from target pose: "
                        << error << " meters");
      }
    }else
    {
        ROS_WARN_STREAM("No result of service /compute_fk \nMoveitCodeError: "
                        << srv.response.error_code);
    }
  }
  else
  {
    ROS_WARN("Failed to call service /compute_fk");
  }
  pub_plan_traj_.publish(plan.trajectory_);
}

void Action::setPlanningTime(const double value)
{
  planning_time_ = value;
  move_group_->setPlanningTime(value);
  if(verbose_)
    ROS_INFO_STREAM("Planning time set to " << value);
}

void Action::setToleranceStep(const double value)
{
  tolerance_step_ = value;
  if(verbose_)
    ROS_INFO_STREAM("Tolerance step set to " << value);
}

void Action::setToleranceMin(const double value)
{
  tolerance_min_ = value;
  if(verbose_)
    ROS_INFO_STREAM("Tolerance min set to " << value);
}

void Action::setMaxVelocityScalingFactor(const double value)
{
  max_velocity_scaling_factor_ = value;
  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
  if(verbose_)
    ROS_INFO_STREAM("Max velocity scaling factor set to " << value);
}

void Action::setVerbose(bool verbose)
{
  verbose_ = verbose;
  if(verbose_)
    ROS_INFO_STREAM("Verbose set to " << verbose);
}

void Action::setAttemptsMax(int value)
{
  attempts_max_ = value;
  if(verbose_)
    ROS_INFO_STREAM("Attempts max set to " << value);
}

void Action::setFlag(int flag)
{
  if(flag == FLAG_MOVE || flag == FLAG_NO_MOVE)
  {
    flag_ = flag;
    if(verbose_)
      ROS_INFO_STREAM("Flag set to " << flag);
  }else
      ROS_WARN_STREAM("No value: " << flag
                      << " for flag, will remain as: " << flag_);
}

void Action::detachObject(const std::string &block_name)
{
  move_group_->detachObject(block_name);
}

void Action::attachObject(const std::string &block_name)
{
  move_group_->attachObject(block_name, grasp_data_.ee_group_);
}

std::string Action::getBaseLink()
{
  return grasp_data_.base_link_;
}

bool Action::checkArm(const int &hand_id,
                      const bool &is_close)
{
  if (hand_id == 0)
  {
    if (is_close)
    {
      //if needed change the arm
      if (plan_group_.find("right") != std::string::npos)
        return false;
    }
    else
      if (plan_group_.find("left") != std::string::npos)
        return false;
  }
  else if (hand_id == 1)
  {
    if (plan_group_.find("right") != std::string::npos)
      return false;
  }
  else if (hand_id == 2)
  {
    if (plan_group_.find("left") != std::string::npos)
      return false;
  }
  return true;
}

void Action::setSurface(const std::string &support_surface)
{
  support_surface_ = support_surface;
}

bool Action::lift()
{
  bool success(false);

  if (!object_attached_.empty())
    move_group_->detachObject(object_attached_);

  geometry_msgs::Pose pose(move_group_->getCurrentPose().pose);
  pose.position.z += 0.05;

  if (verbose_)
    ROS_INFO_STREAM("Lifting the object to"
                    << pose.position.x << " "
                    << pose.position.y << " "
                    << pose.position.z);

  //update the plan
  current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());

  move_group_->setPoseTarget(pose, move_group_->getEndEffectorLink().c_str());
  pub_obj_pose.publish(move_group_->getPoseTarget());

  success = move_group_->plan(*current_plan_);

  if (verbose_ && success)
    ROS_INFO_STREAM("Lift success.");

  //find an approximate solution
  if (!success)
  {
    move_group_->setApproximateJointValueTarget(pose,
                                                move_group_->getEndEffectorLink().c_str());
    success = move_group_->plan(*current_plan_);

    if (verbose_ && success)
      ROS_INFO_STREAM("Lift success with approximate action.");
  }

  if (!success)
    current_plan_.reset();

  return success;
}

}
