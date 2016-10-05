#include <Eigen/Eigen>
#include <stdlib.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{

MetaBlock::MetaBlock(const std::string name,
          const geometry_msgs::Pose pose,
          const uint shapeType,
          const double size_x,
          const double size_y,
          const double size_z,
          ros::Time timestamp):
    name_(name),
    pose_(pose),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    timestamp_(timestamp),
    base_frame_("base_link")
{
  if (pose_.position.y < 0)
    pose_.orientation.y *= -1;

  goal_pose_ = pose_;
  if (pose_.position.y < 0)
    goal_pose_.position.y -= 0.2;
  else
    goal_pose_.position.y += 0.2;

  //setshape
  sprimitive solidPrimitive;
  if (shapeType == sprimitive::CYLINDER)
  {
    solidPrimitive.type = sprimitive::CYLINDER;
    solidPrimitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<sprimitive::CYLINDER>::value);
    solidPrimitive.dimensions[sprimitive::CYLINDER_HEIGHT] = size_y;
    solidPrimitive.dimensions[sprimitive::CYLINDER_RADIUS] = size_x;
  }
  else
  {
    solidPrimitive.type = sprimitive::BOX;
    solidPrimitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<sprimitive::BOX>::value);
    solidPrimitive.dimensions[sprimitive::BOX_X] = size_x;
    solidPrimitive.dimensions[sprimitive::BOX_Y] = size_y;
    solidPrimitive.dimensions[sprimitive::BOX_Z] = size_z;
  }
  shape_ = solidPrimitive;

  //create collision object
  collObj_.header.stamp = ros::Time::now();
  collObj_.header.frame_id = base_frame_;
  collObj_.id = name_;
  collObj_.operation = mcollobj::ADD;
  collObj_.primitives.resize(1);
  if (shape_.dimensions.size() > 0)
    collObj_.primitives[0] = shape_;
  collObj_.primitive_poses.resize(1);
  collObj_.primitive_poses[0] = pose_;
}

MetaBlock::MetaBlock(const std::string name,
          const geometry_msgs::Pose pose,
          const shape_msgs::Mesh mesh,
          const object_recognition_msgs::ObjectType type,
          ros::Time timestamp):
    name_(name),
    pose_(pose),
    timestamp_(timestamp),
    base_frame_("odom")
{
  pose_.orientation.x = -1.0;
  pose_.orientation.y = 0.0;
  pose_.orientation.z = 0.0;
  pose_.orientation.w = 0.0;

  goal_pose_ = pose_;
  goal_pose_.position.x = 0.4;
  goal_pose_.position.y = 0.3;
  goal_pose_.position.z = -0.05;
  goal_pose_.orientation.x = -1.0;
  goal_pose_.orientation.y = 0.0;
  goal_pose_.orientation.z = 0.0;
  goal_pose_.orientation.w = 0.0;
  if (pose_.position.y < 0)
    goal_pose_.position.y *= -1;

  mesh_ = mesh;
  type_ = type;
}

void MetaBlock::updatePose(const geometry_msgs::Pose &pose)
{
  pose_ = pose;
  if (collObj_.primitive_poses.size() > 0)
    collObj_.primitive_poses[0] = pose;
}

void MetaBlock::updatePoseVis(const geometry_msgs::Pose &pose)
{
  if (collObj_.primitive_poses.size() > 0)
    collObj_.primitive_poses[0] = pose;
}

mcollobj MetaBlock::wrapToCollObj(const std::vector <shape_msgs::Mesh> &meshes)
{
  collObj_.header.stamp = ros::Time::now();

  if (!meshes.empty())
  {
    collObj_.meshes.push_back(meshes[0]);
    collObj_.mesh_poses.push_back(pose_);
    /* ROS_INFO_STREAM("-- mesh found: msg_obj_collision.meshes.size()="
     * << msg_obj_collision.meshes.size()); */
  }
  else
    if (collObj_.primitive_poses.size() > 0)
      collObj_.primitive_poses[0] = pose_;

  return collObj_;
}

void MetaBlock::removeBlock(mscene *current_scene)
{
  // Remove/Add collision object
  std::vector<std::string> objects_id;
  objects_id.resize(1);
  objects_id[0] = name_;
  current_scene->removeCollisionObjects(objects_id);
}

void MetaBlock::updatePose(ros::Publisher *pub_obj_moveit,
                           const geometry_msgs::Pose &pose)
{
  updatePose(pose);
  pub_obj_moveit->publish(collObj_);
}
}
