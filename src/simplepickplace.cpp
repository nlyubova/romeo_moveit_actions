//publish messages with objects poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>

//#include <move_base_msgs/MoveBaseAction.h>

#include "romeo_moveit_actions/simplepickplace.hpp"
#include "romeo_moveit_actions/tools.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{
  void SimplePickPlace::loadParams()
  {
    setPose(&pose_zero_, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
    pose_default_ = pose_zero_;
    pose_default_r_ = pose_default_;

    nh_.getParam("robot_name", robot_name_);
    nh_.getParam("verbose", verbose_);

    if (robot_name_ == "nao")
    {
      floor_to_base_height_ = -0.325;
      block_size_x_ = 0.01;
      setPose(&pose_default_, 0.2, 0.1, 0.0);
      setPose(&pose_default_r_, 0.2, -0.1, 0.0);
    }
    else if (robot_name_ == "pepper")
    {
      floor_to_base_height_ = -0.78;
      block_size_x_ = 0.02;
      setPose(&pose_default_, 0.25, 0.2, -0.1);
      setPose(&pose_default_r_, 0.25, -0.2, -0.1);
    }
    else if (robot_name_ == "romeo")
    {
      block_size_x_ = 0.03;
      setPose(&pose_default_, 0.44, 0.15, -0.1);
      setPose(&pose_default_r_, 0.49, -0.25, -0.1);
    }
  }

  MetaBlock SimplePickPlace::createTable()
  {
    //create a table
    double height = -floor_to_base_height_ + (pose_default_.position.z-block_size_y_/2.0);
    double width = std::fabs(pose_default_r_.position.y*2.0) + block_size_x_/2.0;
    double depth = 0.35;
    geometry_msgs::Pose pose;
    setPose(&pose,
            pose_default_.position.x - block_size_x_/2.0 + depth/2.0,
            0.0,
            floor_to_base_height_ + height/2.0);

    MetaBlock table("table",
                    pose,
                    shape_msgs::SolidPrimitive::BOX,
                    depth,
                    width,
                    height);
    return table;
  }

  SimplePickPlace::SimplePickPlace():
      nh_("~"),
      nh_priv_(""),
      robot_name_("romeo"),
      verbose_(false),
      base_frame_("odom"),
      block_size_x_(0.03),
      block_size_y_(0.13),
      floor_to_base_height_(-1.0),
      env_shown_(false),
      objproc_(&nh_priv_),
      evaluation_(&nh_, verbose_, base_frame_),
      use_wheels_(true)
  {
    loadParams();

    // objects related initialization
    sub_obj_coll_ = nh_.subscribe<moveit_msgs::CollisionObject>("/collision_object", 100, &SimplePickPlace::getCollisionObjects, this);

    pub_obj_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/obj_poses", 1);
    pub_obj_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_current", 1);
    pub_obj_moveit_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //pub_moveto_ = nh_.advertise<geometry_msgs::PoseStamped>("cmd_moveto", 1000);

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("odom"));
    visual_tools_->setManualSceneUpdating(false);
    visual_tools_->setFloorToBaseHeight(floor_to_base_height_);
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    ros::Duration(1.0).sleep();

    //initialize the Action class for both arms
    std::string left_arm_name("left_arm");
    std::string right_arm_name("right_arm");
    nh_.getParam("left_arm_name", left_arm_name);
    nh_.getParam("right_arm_name", right_arm_name);
    action_left_ = new Action(&nh_, left_arm_name, "left_hand", robot_name_);
    action_right_ = new Action(&nh_, right_arm_name, "right_hand", robot_name_);
    action_left_->initVisualTools(visual_tools_);
    action_right_->initVisualTools(visual_tools_);
    //action_left_->setDefaultObjPose(pose_default_);
    //action_right_->setDefaultObjPose(pose_default_r_);

    msg_obj_pose_.header.frame_id = action_left_->getBaseLink();
    msg_obj_poses_.header.frame_id = action_left_->getBaseLink();

    //move the robots parts to init positions
    //if (promptUserQuestion("Should I move the head to look down?"))
      //action_left_->poseHeadDown();

    //move arms to the initial position
    action_left_->poseHand(1);
    action_right_->poseHand(1);
    ros::Duration(1.0).sleep();

    //create a possible table
    blocks_surfaces_.push_back(createTable());
    support_surface_ = blocks_surfaces_.back().name_;
    //remove surfaces and publish again
    if (robot_name_ == "romeo")
    {
      std::vector<std::string> objects = getObjectsOldList(&blocks_surfaces_);
      current_scene_.removeCollisionObjects(objects);
      sleep(1.0);
      //add a collision block
      if (blocks_surfaces_.size() > 0)
      {
        blocks_surfaces_.back().publishBlock(&current_scene_);
        env_shown_ = true;
      }
    }

    evaluation_.init(block_size_x_,
                     block_size_y_,
                     floor_to_base_height_,
                     action_left_,
                     action_right_);

    printTutorial(robot_name_);
  }


  void SimplePickPlace::switchArm(Action **action)
  {
    (*action)->poseHand(1);

    if ((*action)->plan_group_.find("left") != std::string::npos)
      *action = action_right_;
    else
      *action = action_left_;

    (*action)->poseHand(2);
    ROS_INFO_STREAM("Switching the active arm to " << (*action)->plan_group_);
  }

  void SimplePickPlace::createObj(const MetaBlock &block)
  {
    blocks_.push_back(block);
    msg_obj_pose_.pose = block.pose_;
    pub_obj_pose_.publish(msg_obj_pose_);

    //publish the collision object
    blocks_.back().publishBlock(&current_scene_);
  }

  void SimplePickPlace::publishAllCollObj(std::vector<MetaBlock> *blocks)
  {
    if (blocks->size() > 0)
    {
      std::vector<moveit_msgs::CollisionObject> coll_objects;

      std::vector<MetaBlock>::iterator block=blocks->begin();
      for (; block != blocks->end(); ++block)
        coll_objects.push_back(block->collObj_);

      if (coll_objects.size() > 0)
        current_scene_.addCollisionObjects(coll_objects);
    }
  }

  bool SimplePickPlace::checkObj(int &block_id)
  {
    if ((block_id >= 0) && (block_id < blocks_.size()))
      return true;
    else
      false;
  }

  void SimplePickPlace::run()
  {
    int block_id = -1;
    int hand_id = 0; //0: any, 1: left, 2:right
    Action *action;
    action = action_left_;
    action->poseHand(2);
    ros::Duration(1.0).sleep();

    static ros::Rate rate(12.0);

    //create a virtual object
    createObj(MetaBlock("Virtual1",
                        pose_default_,
                        shape_msgs::SolidPrimitive::CYLINDER,
                        block_size_x_,
                        block_size_y_,
                        0.0));
    block_id = 0;

    sleep(1.0);
    if (blocks_.size() > 0)
      pose_optimal_ = blocks_[block_id].getTransform(&listener_);

    std::string actionName = "";

    //the main loop
    while(ros::ok())
    {
      //if there are some objects, take the first
      if ((block_id == -1) && (blocks_.size() > 0))
        block_id = 0;
      //if the object does not exist
      else if (block_id >= blocks_.size())
      {
        ROS_INFO_STREAM("--- The current object is outside of the list");
        block_id = -1;
        cleanObjects(&blocks_, true);
      }

      if (block_id >= 0)
      {
        //update the object's pose
        msg_obj_pose_.pose = blocks_[block_id].pose_;
        pub_obj_pose_.publish(msg_obj_pose_);
        ROS_INFO_STREAM("The current active object is "
                        << blocks_[block_id].name_
                        << " out of " << blocks_.size());
      }

      //ROS_INFO_STREAM("What do you want me to do ?");
      actionName = promptUserQuestionString();
      ROS_INFO_STREAM("Action chosen '" << actionName
                      << "' object_id=" << block_id
                      << " the arm active=" << action->plan_group_);

      // Pick the object with a grasp generator
      if ((checkObj(block_id)) && (actionName == "g"))
      {
        bool success = action->pickAction(&blocks_[block_id], support_surface_);

        if(success)
          stat_poses_success_.push_back(blocks_[block_id].pose_);
      }
      // Place the object with a default function
      else if ((checkObj(block_id)) && (actionName == "p"))
      {
        if(action->placeAction(&blocks_[block_id], support_surface_))
        {
          /* swap this block's start and the end pose
           * so that we can then move them back to position */
          swapPoses(&blocks_[block_id].pose_, &blocks_[block_id].goal_pose_);
          resetBlock(&blocks_[block_id]);
        }
      }
      //return the hand to the initial pose and relese the object
      else if (actionName == "i")
      {
        // Remove the attached object and the collision object
        if (checkObj(block_id))
          action->releaseObject(&blocks_[block_id]);
        else
          action->poseHand(2);
      }
      //return the hand to the initial pose
      else if ((actionName.length() == 3) || (actionName.compare(0,1,"i") == 0))
      {
        //remove the attached object and the collision object
        if (checkObj(block_id))
          resetBlock(&blocks_[block_id]);
        //go to the required pose
        action->poseHand(actionName.at(1));
      }
      //exit the application
      else if (actionName == "q")
        break;
      //clean a virtual box on the left arm side
      else if (actionName == "lb")
      {
        cleanObjects(&blocks_);
        createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::BOX, block_size_x_, block_size_y_, 0.0));
      }
      //create a virtual cylinder on the left hand side
      else if (actionName == "lc")
      {
        cleanObjects(&blocks_);
        createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
      }
      //create a virtual box on the right hand side
      else if (actionName == "rb")
      {
        cleanObjects(&blocks_);
        createObj(MetaBlock("Virtual1", pose_default_r_, shape_msgs::SolidPrimitive::BOX, block_size_x_, block_size_y_, 0.0));
      }
      //create a virtual cylinder on the right hand side
      else if (actionName == "rc")
      {
        cleanObjects(&blocks_);
        createObj(MetaBlock("Virtual1", pose_default_r_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
      }
      else if (actionName == "d") //detect objects
      {
        cleanObjects(&blocks_);

        objproc_.triggerObjectDetection();

        // publish all objects as collision blocks
        publishAllCollObj(&blocks_);
      }
      //continuous object detection
      else if (actionName == "dd")
      {
        cleanObjects(&blocks_, true);

        ROS_INFO_STREAM("Object detection is running...");
        ros::Time start_time = ros::Time::now();
        while ((blocks_.size() <= 0)
               && (ros::Time::now()-start_time < ros::Duration(10.0)))
        {
          objproc_.triggerObjectDetection();
          rate.sleep();
        }
        if (verbose_)
          ROS_INFO_STREAM(blocks_.size() << " objects detected");

        // publish all objects as collision blocks
        publishAllCollObj(&blocks_);
      }
      //plan possible movement
      else if ((checkObj(block_id)) && (actionName == "plan"))
      {
        //TODO: do not remove an object but allow a collision to it
        visual_tools_->cleanupCO(blocks_[block_id].name_);
        action->graspPlan(&blocks_[block_id], support_surface_);
        resetBlock(&blocks_[block_id]);
      }
      //plan all possible movements
      else if ((checkObj(block_id)) && (actionName == "a"))
      {
        //TODO: do not remove an object but allow a collision to it
        visual_tools_->cleanupCO(blocks_[block_id].name_);
        action->graspPlanAllPossible(&blocks_[block_id], support_surface_);
        resetBlock(&blocks_[block_id]);
      }
      //reaching based on default pose and grasp
      else if ((checkObj(block_id)) && (actionName == "u"))
      {
        //check the we use the correct arm
        /*if (block_id >= 0)
        {
          bool is_close(true);
          if (action_left_->computeDistance(&blocks_[block_id])
              > action_right_->computeDistance(&blocks_[block_id]))
            if (action->plan_group_.find("left") != std::string::npos)
              is_close = false;

          if (!action->checkArm(hand_id, is_close))
          {
            switchArm(&action);
            sleep(2.0);
            ROS_INFO_STREAM("//// switch the arm 378");
          }
        }
        else
        {
          if (!action->checkArm(hand_id))
          {
            switchArm(&action);
            sleep(2.0);
            ROS_INFO_STREAM("//// switch the arm 387");
          }
        }*/

        if (use_wheels_)
          moveToObject(&blocks_[block_id]);

        action->reachGrasp(&blocks_[block_id], support_surface_);
      }
      else if ((checkObj(block_id)) && (actionName == "lift"))
      {
        action->lift();
      }
      //reach from top
      else if ((checkObj(block_id)) && (actionName == "reachtop"))
      {
        //TODO: do not remove an object but allow a collision to it
        visual_tools_->cleanupCO(blocks_[block_id].name_);
        geometry_msgs::Pose pose = blocks_[block_id].pose_;
        pose.orientation.x = 0;
        pose.position.z += blocks_[block_id].size_x_*1.5;
        action->reachAction(pose, support_surface_);
        resetBlock(&blocks_[block_id]);
      }
      //pick an object without a grasp generator
      else if ((checkObj(block_id)) && (actionName == "b"))
      {
        action->pickDefault(&blocks_[block_id], support_surface_);
      }
      //execute the planned movement
      else if ((checkObj(block_id)) && (actionName == "execute"))
      {
        action->executeAction();
      }
      //print the current pose
      else if (actionName == "get_pose")
      {
        action->getPose();
      }
      //open hand
      else if (actionName == "hand_open")
      {
        action->poseHandOpen();
      }
      //close hand
      else if (actionName == "hand_close")
      {
        action->poseHandClose();
      }
      //process the next object
      else if (actionName == "next_obj")
        ++block_id;
      //test the goal space for picking
      else if (actionName == "test_pick")
      {
        cleanObjects(&blocks_surfaces_, false);
        cleanObjects(&blocks_);
        evaluation_.testReach(nh_,
                              &pub_obj_pose_,
                              &pub_obj_poses_,
                              &pub_obj_moveit_,
                              true);
        //draw the table again
        if (env_shown_)
          if (blocks_surfaces_.size() > 0)
            blocks_surfaces_.back().publishBlock(&current_scene_);
      }
      //test the goal space for reaching
      else if (actionName == "test_reach")
      {
        cleanObjects(&blocks_surfaces_, false);
        cleanObjects(&blocks_);
        evaluation_.testReach(nh_,
                              &pub_obj_pose_,
                              &pub_obj_poses_,
                              &pub_obj_moveit_,
                              false);
        //draw the table again
        if (env_shown_)
          if (blocks_surfaces_.size() > 0)
            blocks_surfaces_.back().publishBlock(&current_scene_);
      }
      //set the table height
      else if (actionName == "set_table_height")
      {
        //TODO make a function
        if (blocks_surfaces_.size() > 0)
        {
          blocks_surfaces_.back().size_z_ = promptUserValue("Set the table height to");
          blocks_surfaces_.back().pose_.position.z =
              floor_to_base_height_ + blocks_surfaces_.back().size_z_/2.0;
          blocks_surfaces_.back().publishBlock(&current_scene_);
        }
      }
      //clean the scene
      else if (actionName == "table")
      {
        if (env_shown_)
        {
          std::vector<std::string> objects = getObjectsList(blocks_surfaces_);
          current_scene_.removeCollisionObjects(objects);
          env_shown_ = false;
        }
        else
        {
          if (blocks_surfaces_.size() > 0)
          {
            blocks_surfaces_.back().publishBlock(&current_scene_);
            env_shown_ = true;
          }
        }
      }
      //moving the virtual object down
      else if (checkObj(block_id) && ((actionName == "x") || (actionName == "down")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.z -= 0.05;
        blocks_[block_id].updatePose(&current_scene_, pose);
      }
      //move the virtual object left
      else if (checkObj(block_id) && ((actionName == "s") || (actionName == "left")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.y -= 0.05;
        blocks_[block_id].updatePose(&current_scene_, pose);
      }
      //move the virtual object up
      else if (checkObj(block_id) && ((actionName == "e") || (actionName == "up")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.z += 0.05;
        blocks_[block_id].updatePose(&current_scene_, pose);
      }
      //move the virtual object right
      else if (checkObj(block_id) && ((actionName == "f") || (actionName == "right")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.y += 0.05;
        blocks_[block_id].updatePose(&current_scene_, pose);
      }
      //move the virtual object
      else if (checkObj(block_id) && ((actionName == "c") || (actionName == "farther")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.x += 0.05;
        blocks_[block_id].updatePose(&current_scene_, pose);
      }
      //move the virtual object closer
      else if (checkObj(block_id) && ((actionName == "r") || (actionName == "closer")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.x -= 0.05;
        blocks_[block_id].updatePose(&current_scene_, pose);
      }
      //compute the distance to the object
      else if (checkObj(block_id) && (actionName == "compute_distance"))
      {
        ROS_INFO_STREAM("The distance to the object = "
                        << action->computeDistance(&blocks_[block_id])
                        << ", linear: " << action->computeDistance(&blocks_[block_id], 1, 1, 0)
                        << ", in x: " << action->computeDistance(&blocks_[block_id], 1, 0, 0)
                        << ", in y: " << action->computeDistance(&blocks_[block_id], 0, 1, 0));

        blocks_[block_id].getTransform(&listener_);

        moveToObject(&blocks_[block_id]);
      }
      //set the tolerance
      else if (actionName == "j")
        action->setTolerance(promptUserValue("Set the value: "));
      //switch the active arm
      else if (actionName == "switcharm")
      {
        hand_id = promptUserInt("Choose the arm; 0: any; 1: left; 2: right");
      }
      //move the head down
      else if (actionName == "look_down")
        action->poseHeadDown();
      //move the head to zero
      else if (actionName == "look_zero")
        action->poseHeadZero();
      //print the statistics on grasps
      else if (actionName == "stat")
      {
        ROS_INFO_STREAM("Successfully grasped objects at the locations: ");
        std::vector <geometry_msgs::Pose>::iterator it = stat_poses_success_.begin();
        for (; it != stat_poses_success_.end(); ++it)
        {
          ROS_INFO_STREAM(" [" << it->position.x << " "
                          << it->position.y << " "
                          << it->position.z << "] ["
                          << it->orientation.x << " "
                          << it->orientation.y << " "
                          << it->orientation.z << " "
                          << it->orientation.w << "]");
        }
      }
      //print the statistics on grasps
      else if (actionName == "stat_evaluation")
      {
        evaluation_.printStat();
      }

      if (actionName == "q")
        break;

      //ros::spinOnce();
      //rate.sleep();
    }
  }

  void SimplePickPlace::moveTo(geometry_msgs::Twist *msg_twist)
  {
    ROS_INFO_STREAM("Moving closer to object ["
                    << msg_twist->linear.x << " "
                    << msg_twist->linear.y << " "
                    << msg_twist->angular.z << "]");

    pub_cmd_.publish(*msg_twist);
    ros::spinOnce();
    sleep(1.0);

    //stop the robot
    msg_twist->linear.x = 0.0;
    msg_twist->linear.y = 0.0;
    msg_twist->angular.z = 0.0;

    pub_cmd_.publish(*msg_twist);
    ros::spinOnce();
    sleep(2.0);
  }

  void SimplePickPlace::moveToObject(MetaBlock *block)
  {
    geometry_msgs::Twist msg_twist;
    msg_twist.linear.x = msg_twist.linear.y = msg_twist.linear.z;
    msg_twist.angular.x = msg_twist.angular.y = msg_twist.angular.z = 0.0;

    //compute the object pose in the robot's frame
    tf::Stamped<tf::Pose> pose_real = block->getTransform(&listener_);

    //compute the distandce to the object
    float dist_real = sqrt(pose_real.getOrigin().x() * pose_real.getOrigin().x()
                      + pose_real.getOrigin().y() * pose_real.getOrigin().y());
    float dist_opt = sqrt(pose_optimal_.getOrigin().x() * pose_optimal_.getOrigin().x()
                          + pose_optimal_.getOrigin().y() * pose_optimal_.getOrigin().y());
    float dist = dist_real - dist_opt;
    //ROS_INFO_STREAM("Current distance to the optimal pose " << dist);

    if (fabs(dist) > 0.21f) //evaluation_->getXmax();
    {
      float theta = pose_real.getOrigin().x()
          / (dist_real * fabs(pose_real.getOrigin().x()));
      theta = theta / 360.0f * 3.1416f;

      //if colinear than theta  = 90?
      if (fabs(pose_real.getOrigin().x()) == 0)
        theta = 1.41372;

      //turn the robot to align with the object's X axis
      msg_twist.angular.z = -theta;
      //moveTo(&msg_twist);

      //move the robot
      msg_twist.linear.x = dist + 0.0011;
      //msg_twist.angular.z = 0.0;
      moveTo(&msg_twist);
    }
  }

  //clean the object list based on the timestamp
  void SimplePickPlace::cleanObjects(std::vector<MetaBlock> *objects,
                                     const bool list_erase)
  {
    std::vector<std::string> objects_list = getObjectsOldList(objects);
    current_scene_.removeCollisionObjects(objects_list);

    //remove from the memory
    if (list_erase)
    {
      objects->clear();
      msg_obj_poses_.poses.clear();
      pub_obj_poses_.publish(msg_obj_poses_);
    }
  }

  void SimplePickPlace::resetBlock(MetaBlock *block)
  {
    // Remove attached object
    if(block->name_ != support_surface_)
    {
      action_left_->detachObject(block->name_);
      action_right_->detachObject(block->name_);
    }
    sleep(0.2);

    // Remove/Add collision object
    //visual_tools_->cleanupCO(block->name_);
    //visual_tools_->processCollisionObjectMsg(block->wrapToCollisionObject);

    block->publishBlock(&current_scene_);
  }

  void SimplePickPlace::getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    //cleanObjects(&blocks_, true);

    try
    {
      if ((msg->meshes.size() > 0) && (msg->mesh_poses.size() > 0))
      {
        geometry_msgs::Pose pose = msg->mesh_poses[0];
        //if (evaluation_.inWorkSpace(pose))
        {
          //check if exists
          int idx = findObj(blocks_, msg->id);

          //pose.position.z += 0.0576;
          if (idx == -1) //if not found, create a new one
          {
            blocks_.push_back(MetaBlock(msg->id, pose, msg->meshes.front(), msg->type, msg->header.stamp));
            msg_obj_poses_.poses.push_back(pose);
          }
          else if ((idx >= 0) && (idx < blocks_.size()))
          {
            blocks_[idx].updatePose(pose);
            if (idx >= msg_obj_poses_.poses.size())
              msg_obj_poses_.poses.resize(idx+1);
            msg_obj_poses_.poses[idx] = pose;
          }
        }
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }

} //namespace
