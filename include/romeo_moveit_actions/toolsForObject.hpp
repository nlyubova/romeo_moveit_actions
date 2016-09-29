#ifndef TOOLSFOROBJECT_H
#define TOOLSFOROBJECT_H

#include <geometry_msgs/Pose.h>

#include "romeo_moveit_actions/metablock.hpp"

namespace moveit_simple_actions
{

void setPose(geometry_msgs::Pose *pose,
             const double &x,
             const double &y,
             const double &z,
             const double &ox,
             const double &oy,
             const double &oz,
             const double &ow);

void setPose(geometry_msgs::Pose *pose,
             const double &x,
             const double &y,
             const double &z);

int findObj(const std::vector<MetaBlock> &blocks,
            const std::string name);

std::vector<std::string> getObjectsList(const std::vector<MetaBlock> &blocks);

std::vector<std::string> getObjectsOldList(std::vector<MetaBlock> *objects);

void swapPoses(geometry_msgs::Pose *pose1,
               geometry_msgs::Pose *pose2);
}

#endif // TOOLSFOROBJECT_H
