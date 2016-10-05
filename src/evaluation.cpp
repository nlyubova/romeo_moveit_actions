#include "romeo_moveit_actions/evaluation.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{

Evaluation::Evaluation(ros::NodeHandle *nh,
                       const bool &verbose,
                       const std::string &base_frame):
  verbose_(verbose),
  base_frame_(base_frame),
  test_step_(0.0),
  x_min_(0.0),
  x_max_(0.0),
  y_min_(0.0),
  y_max_(0.0),
  z_min_(0.0),
  z_max_(0.0),
  attempts_nbr_(2),
  planning_time_(20.0)
{
  setPose(&pose_zero_, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);

  std::string robot_name("romeo");
  nh->getParam("robot_name", robot_name);
  nh->getParam("test_step", test_step_);
  nh->getParam("x_min", x_min_);
  nh->getParam("x_max", x_max_);
  nh->getParam("y_min", y_min_);
  nh->getParam("y_max", y_max_);
  nh->getParam("z_min", z_min_);
  nh->getParam("z_max", z_max_);

  if (robot_name == "nao")
  {
    test_step_ = (test_step_==0.0)?0.03:test_step_;
    x_min_ = (x_min_==0.0)?0.1:x_min_;
    x_max_ = (x_max_==0.0)?0.21:x_max_;
    y_min_ = (y_min_==0.0)?0.12:y_min_;
    y_max_ = (y_max_==0.0)?0.24:y_max_;
    z_min_ = (z_min_==0.0)?-0.07:z_min_;
    z_max_ = (z_max_==0.0)?0.05:z_max_;
  }
  else if (robot_name == "pepper")
  {
    test_step_ = (test_step_==0.0)?0.02:test_step_;
    x_min_ = (x_min_==0.0)?0.2:x_min_;
    x_max_ = (x_max_==0.0)?0.3:x_max_;
    y_min_ = (y_min_==0.0)?0.12:y_min_;
    y_max_ = (y_max_==0.0)?0.26:y_max_;
    z_min_ = (z_min_==0.0)?-0.17:z_min_;
    z_max_ = (z_max_==0.0)?0.05:z_max_;
  }
  else if (robot_name == "romeo")
  {
    test_step_ = (test_step_==0.0)?0.02:test_step_;
    x_min_ = (x_min_==0.0)?0.38:x_min_;
    x_max_ = (x_max_==0.0)?0.5:x_max_;
    y_min_ = (y_min_==0.0)?0.12:y_min_;
    y_max_ = (y_max_==0.0)?0.24:y_max_;
    z_min_ = (z_min_==0.0)?-0.17:z_min_;
    z_max_ = (z_max_==0.0)?-0.08:z_max_;
  }
}

void Evaluation::init(const double &block_size_x,
                      const double &block_size_y,
                      const double floor_to_base_height,
                      Action *action_left,
                      Action *action_right)
{
  block_size_x_ = block_size_x;
  block_size_y_ = block_size_y;
  floor_to_base_height_ = floor_to_base_height;

  action_left_ = action_left;
  action_right_ = action_right;

  //create a default block
  block_ = new MetaBlock("Virtual1",
                         pose_zero_,
                         sprimitive::CYLINDER,
                         block_size_x_,
                         block_size_y_,
                         0.0);

  //create a default table
  double height = -floor_to_base_height_ + z_min_;
  double width = y_max_*2.0;
  double depth = 0.35;
  geometry_msgs::Pose pose;
  setPose(&pose,
          x_min_ + depth/2.0,
          0.0,
          floor_to_base_height_ + height/2.0);

  table_ = new MetaBlock("table", pose, sprimitive::BOX, depth, width, height);

}

geometry_msgs::PoseArray Evaluation::generatePosesGrid()
{
  std::vector<MetaBlock> blocks_test;
  geometry_msgs::PoseArray poses;
  poses.header.frame_id = base_frame_;

  /*//detect objects to get any mesh
  ROS_INFO_STREAM(" Looking for objects");
  ros::Time start_time = ros::Time::now();
  while ((blocks_.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
  {
    objproc.triggerObjectDetection();
  }*/

  if (verbose_)
    ROS_INFO_STREAM(" Generating goals in the target space");

  MetaBlock block(*block_);

  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);

  int count = 0;
  for (double y=y_min; y<=y_max; y+=y_step)
    for (double z=z_min_; z<=z_max_; z+=test_step_)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        block.pose_.position.x = x;
        block.pose_.position.y = y;
        block.pose_.position.z = z;
        blocks_test.push_back(block);

        poses.poses.push_back(block.pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  y_min = -y_max_;
  y_max = -y_min_;
  for (double y=y_min; y<=y_max; y+=y_step)
    for (double z=z_min_; z<=z_max_; z+=test_step_)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        block.pose_.position.x = x;
        block.pose_.position.y = y;
        block.pose_.position.z = z;
        blocks_test.push_back(block);

        poses.poses.push_back(block.pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  //if (verbose_)
    ROS_INFO_STREAM("Total number of generated poses=" << count);
  return poses;
}

geometry_msgs::PoseArray Evaluation::generatePosesRnd(const int poses_nbr)
{
  std::vector<MetaBlock> blocks_test;
  geometry_msgs::PoseArray poses;
  poses.header.frame_id = "base_link";

  int count = 0;
  while (count < poses_nbr){
    geometry_msgs::Pose pose(pose_zero_);
    pose.position.x = 0.35f + float(rand() % 150)/1000.0f;
    pose.position.y = float(rand() % 90)/100.0f - 0.45;
    pose.position.z = -0.23f + (float(rand() % 230)/1000.0f);
    blocks_test.push_back(MetaBlock("BlockTest", pose, sprimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
    poses.poses.push_back(blocks_test.back().pose_);
    ++count;
  }
  return poses;
}

void Evaluation::testReach(ros::NodeHandle &nh,
                           ros::Publisher *pub_obj_pose,
                           ros::Publisher *pub_obj_poses,
                           ros::Publisher *pub_obj_moveit,
                           const bool pickVsReach,
                           const bool test_poses_rnd)
{
  geometry_msgs::PoseArray poses_test;
  if (test_poses_rnd)
    poses_test = generatePosesRnd(200);
  else
    poses_test = generatePosesGrid();

  //visualize all generated samples of the goal space
  pub_obj_poses->publish(poses_test);
  sleep(0.05);

  ros::Publisher pub_obj_poses_left =
      nh.advertise<geometry_msgs::PoseArray>("/poses_reachable_left", 100);
  ros::Publisher pub_obj_poses_right =
      nh.advertise<geometry_msgs::PoseArray>("/poses_reachable_right", 100);

  int targets_nbr = 0;
  geometry_msgs::PoseArray poses_success_ex;
  poses_success_ex.header.frame_id = action_right_->getBaseLink();
  targets_nbr += testReachSingleHand(action_left_,
                                     pub_obj_pose,
                                     &pub_obj_poses_left,
                                     pub_obj_moveit,
                                     pickVsReach,
                                     poses_success_ex);

  targets_nbr += testReachSingleHand(action_right_,
                                     pub_obj_pose,
                                     &pub_obj_poses_right,
                                     pub_obj_moveit,
                                     pickVsReach,
                                     poses_success_ex);

  poses_success_.poses.insert(poses_success_.poses.end(),
                              poses_success_ex.poses.begin(),
                              poses_success_ex.poses.end());

  targets_nbr_ += targets_nbr;

  //print all reachable poses
  //if (verbose_)
    printStat(poses_success_ex, targets_nbr);
}

int Evaluation::testReachSingleHand(Action *action,
                                    ros::Publisher *pub_obj_pose,
                                    ros::Publisher *pub_obj_poses,
                                    ros::Publisher *pub_obj_moveit,
                                    const bool pickVsReach,
                                    geometry_msgs::PoseArray &poses_success)
{
  moveit::planning_interface::PlanningSceneInterface current_scene;

  //Set the test space params
  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);
  if (action->plan_group_.find("right") != std::string::npos)
  {
    y_min = -y_max_;
    y_max = -y_min_;
  }

  MetaBlock block(*block_);

  int count_total = 0;
  for (double z=z_min_; z<=z_max_; z+=test_step_)
  {
    //update the table height
    table_->size_z_ = -floor_to_base_height_ + (z-block_size_y_/2.0);
    table_->pose_.position.z = floor_to_base_height_ + table_->size_z_/2.0;

    for (double y=y_min; y<=y_max; y+=y_step)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        ++count_total;

        table_->pose_.position.x = x - block_size_x_/2.0 + table_->size_x_/2.0,
        table_->updatePose(table_->pose_);
        std::vector<std::string> objects;
        objects.push_back(table_->name_);
        current_scene.removeCollisionObjects(objects);
        sleep(1.5);
        pub_obj_moveit->publish(table_->collObj_);

        // publish the pose
        geometry_msgs::PoseStamped msg_obj_pose;
        msg_obj_pose.header.frame_id = action->getBaseLink();
        msg_obj_pose.pose.position.x = x;
        msg_obj_pose.pose.position.y = y;
        msg_obj_pose.pose.position.z = z;
        pub_obj_pose->publish(msg_obj_pose);

        // publish the collision object
        block.updatePose(msg_obj_pose.pose);
        pub_obj_moveit->publish(block.collObj_);

        bool success(false);
        if (pickVsReach)
        {
          success = action->pickAction(&block,
                                       table_->name_,
                                       attempts_nbr_,
                                       planning_time_);
        }
        else
        {
          success = action->reachGrasp(&block,
                                       table_->name_,
                                       attempts_nbr_,
                                       planning_time_);
        }
        if (success)
        {
          //reset object, at first detach it
          action->detachObject(block.name_);

          poses_success.poses.push_back(block.pose_);
          pub_obj_poses->publish(poses_success);
        }
        //return the hand
        action->poseHand(1);
        sleep(1.5);

        //remove collision object
        block.removeBlock(&current_scene);
      }
  }

  //return the hand
  action->poseHand(1);
  sleep(1.5);
  return count_total;
}

void Evaluation::printStat()
{
  printStat(poses_success_, targets_nbr_);
}

void Evaluation::printStat(const geometry_msgs::PoseArray &poses,
                           const int &targets_nbr)
{
  ROS_INFO_STREAM("Exploration of reachable space \n"
                  << "Successfull grasps = " << poses.poses.size()
                  << "/ total " << targets_nbr << "\n"
                  << "Test params: \n"
                  << "Max attempts number = " << attempts_nbr_
                  << "\n planning time = " << planning_time_ << "\n"
                  << "The tested space: step = " << test_step_
                  << " the zone: \n"
                  << "x: [ " << x_min_ << " " << x_max_ << " ]\n"
                  << "y: [ " << y_min_ << " " << y_max_ << " ]\n"
                  << "z: [ " << z_min_ << " " << z_max_ << " ]\n"
                  << "Successfull grasp at positions [x y z] [x y z w]:");

  for (int i=0; i != poses.poses.size(); ++i)
  {
    std::cout << " [" << poses.poses[i].position.x << " "
              << poses.poses[i].position.y << " "
              << poses.poses[i].position.z << "] ["
              << poses.poses[i].orientation.x << " "
              << poses.poses[i].orientation.y << " "
              << poses.poses[i].orientation.z << " "
              << poses.poses[i].orientation.w << "]" << std::endl;
  }
}

bool Evaluation::inWorkSpace(geometry_msgs::Pose pose)
{
  bool res(false);
  if ((pose.position.x < x_max_) && (pose.position.x > x_min_)
      && (pose.position.y < y_max_) && (pose.position.y > y_min_)
      && (pose.position.z < z_max_) && (pose.position.z > z_min_))
    res = true;
  return res;
}

float Evaluation::getXmax()
{
  return x_max_;
}

float Evaluation::getYmax()
{
  return y_max_;
}

}
