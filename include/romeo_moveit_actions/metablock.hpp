#ifndef METABLOCK_H
#define METABLOCK_H

// ROS
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

typedef shape_msgs::SolidPrimitive sprimitive;
typedef moveit_msgs::CollisionObject mcollobj;
typedef moveit::planning_interface::PlanningSceneInterface mscene;

namespace moveit_simple_actions
{

class MetaBlock
{
public:
  MetaBlock(const std::string name,
            const geometry_msgs::Pose pose,
            const uint shapeType,
            const double size_x,
            const double size_y,
            const double size_z,
            ros::Time timestamp=ros::Time::now());

  MetaBlock(const std::string name,
            const geometry_msgs::Pose pose,
            const shape_msgs::Mesh mesh,
            const object_recognition_msgs::ObjectType type,
            ros::Time timestamp=ros::Time::now());

  //! @brief update the object pose
  void updatePose(const geometry_msgs::Pose &pose);

  //! @brief update the object's pose visually but not its pose
  void updatePoseVis(const geometry_msgs::Pose &pose);

  //! @brief wrap to collision object
  mcollobj wrapToCollObj(const std::vector <shape_msgs::Mesh> &meshes);

  //! @brief remoev the object
  void removeBlock(mscene *current_scene);

  //! @brief update the pose and publish
  void updatePose(ros::Publisher *pub_obj_moveit,
                  const geometry_msgs::Pose &pose);

  //object name
  std::string name_;

  //the current position
  geometry_msgs::Pose pose_;

  //the goal position
  geometry_msgs::Pose goal_pose_;

  //corresponding collision object
  moveit_msgs::CollisionObject collObj_;

  //x dimenssion
  double size_x_;

  //y dimenssion
  double size_y_;

  //z dimenssion
  double size_z_;

  //timestamp of creation
  ros::Time timestamp_;

  //the base frame
  std::string base_frame_;

  //corresponding object type in DB
  object_recognition_msgs::ObjectType type_;

protected:
  //the object's shape
  shape_msgs::SolidPrimitive shape_;

  //the object's mesh
  shape_msgs::Mesh mesh_;
};
}

#endif // METABLOCK_H
