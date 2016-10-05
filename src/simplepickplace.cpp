//publish messages with objects poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>

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

    if (robot_name_ == "romeo")
    {
      blocks_surfaces_.push_back(createTable());
      support_surface_ = blocks_surfaces_.back().name_;
    }

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

    //Move the robots parts to init positions
    //if (promptUserQuestion("Should I move the head to look down?"))
      //action_left_->poseHeadDown();

    //move arms to the initial position
    action_left_->poseHand(1);
    action_right_->poseHand(1);
    ros::Duration(1.0).sleep();

    // Remove table and publish gain
    std::vector<std::string> objects = getObjectsOldList(&blocks_surfaces_);
    current_scene_.removeCollisionObjects(objects);
    sleep(1.0);
    if (blocks_surfaces_.size() > 0)
    {
      pub_obj_moveit_.publish(blocks_surfaces_.back().collObj_);
      env_shown_ = true;
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
    pub_obj_moveit_.publish(block.collObj_);
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

      if (block_id >= 0)
      {
        if (!action->checkArm(hand_id, blocks_[block_id].pose_.position.y))
          switchArm(&action);
      }
      else
      {
        if (!action->checkArm(hand_id))
          switchArm(&action);
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

        //if not succeded then try with another arm
        /*if(!success)
        {
          switchArm(&action);
          success = action->pickAction(&blocks_[block_id], support_surface_);
        }*/

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
        if (blocks_.size() > 0)
        {
          std::vector<MetaBlock>::iterator block=blocks_.begin();
          for (; block != blocks_.end(); ++block)
          {
            //visual_tools_->cleanupCO(block->name_);
            moveit_msgs::CollisionObject msg =
                block->wrapToCollObj(objproc_.getMeshFromDB(block->type_));
            if (!msg.meshes.empty())
              pub_obj_moveit_.publish(msg);
            else
              pub_obj_moveit_.publish(block->collObj_);
          }
        }
      }
      //continuous object detection
      else if (actionName == "dd")
      {
        cleanObjects(&blocks_);

        ros::Time start_time = ros::Time::now();
        while ((blocks_.size() <= 0)
               && (ros::Time::now()-start_time < ros::Duration(10.0)))
        {
          ROS_INFO_STREAM("Object detection is running...");
          objproc_.triggerObjectDetection();
        }
        ROS_INFO_STREAM(blocks_.size() << " objects detected");


        // publish all objects as collision blocks
        if (blocks_.size() > 0)
        {
          std::vector<MetaBlock>::iterator block=blocks_.begin();
          for (; block != blocks_.end(); ++block)
          {
            moveit_msgs::CollisionObject msg =
                block->wrapToCollObj(objproc_.getMeshFromDB(block->type_));
            if (!msg.primitive_poses.empty())
              pub_obj_moveit_.publish(msg);
            else
              pub_obj_moveit_.publish(block->collObj_);
          }
        }
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
        if (use_wheels_)
        {
          //block pose after moving
          geometry_msgs::Pose pose_obj = blocks_[block_id].pose_;
          bool do_move(false);
          geometry_msgs::Twist msg_twist;
          msg_twist.linear.x = msg_twist.linear.y = msg_twist.angular.z = 0.0;

          float dist = action->computeDistance(&blocks_[block_id], 1, 0, 0);
          if (dist > 0.243f) //evaluation_->getXmax();
          {
            do_move = true;
            msg_twist.linear.x = dist - pose_default_.position.x;
            pose_obj.position.x -= msg_twist.linear.x;
          }
          dist = action->computeDistance(&blocks_[block_id], 0, 1, 0);
          /*if (std::fabs(dist) > std::fabs(evaluation_.getYmax()))
          {
            do_move = true;
            if (pose_obj.position.y > 0)
              msg_twist.linear.y = dist - pose_default_.position.y;
            else
              msg_twist.linear.y = dist + pose_default_.position.y;
            pose_obj.position.y -= msg_twist.linear.y;
          }*/

          if (do_move)
          {
            //remember the initial pose
            tf::Stamped<tf::Pose> transformed_pose_robot;
            tf::TransformListener listener;

            tf::Stamped<tf::Pose> tf_obj(
                  tf::Pose(
                    tf::Quaternion(blocks_[block_id].pose_.orientation.x,
                                   blocks_[block_id].pose_.orientation.y,
                                   blocks_[block_id].pose_.orientation.z,
                                   blocks_[block_id].pose_.orientation.w),
                    tf::Vector3(blocks_[block_id].pose_.position.x,
                                blocks_[block_id].pose_.position.y,
                                blocks_[block_id].pose_.position.z)),
                ros::Time(0), "odom"); //blocks_[block_id].base_frame_
            try
            {
              //the the base current bose
              listener.transformPose("base_link", tf_obj, transformed_pose_robot);
            }
            catch (tf::TransformException ex)
            {
             ROS_ERROR("%s",ex.what());
            }

            sleep(1.0);
            try
            {
              listener.transformPose("base_link", tf_obj, transformed_pose_robot);
            }
            catch (tf::TransformException ex)
            {
             ROS_ERROR("%s",ex.what());
            }

            listener.transformPose("base_link", tf_obj, transformed_pose_robot);
            ROS_INFO_STREAM("The object to base "
                           << " " << transformed_pose_robot.getOrigin().x()
                           << " " << transformed_pose_robot.getOrigin().y()
                           << " " << transformed_pose_robot.getOrigin().z()
                           );

            ROS_INFO_STREAM("Moving closer to (x y)"
                            << msg_twist.linear.x << " "
                            << msg_twist.linear.y);
            pub_cmd_.publish(msg_twist);
            ros::spinOnce();
            sleep(1.0);
            msg_twist.linear.x = msg_twist.linear.y = msg_twist.angular.z = 0.0;
            pub_cmd_.publish(msg_twist);
            ros::spinOnce();
            sleep(2.0);

            try
            {
              /*listener.transformPose("/", pose_robot, transformed_pose_robot);*/

             //the the base current bose
             listener.transformPose("base_link", tf_obj, transformed_pose_robot);
             ROS_INFO_STREAM("The object to base "
                            << " " << transformed_pose_robot.getOrigin().x()
                            << " " << transformed_pose_robot.getOrigin().y()
                            << " " << transformed_pose_robot.getOrigin().z()
                            );
            }
            catch (tf::TransformException ex)
            {
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
            }

            //update the object position
            blocks_[block_id].updatePose(pose_obj);
          }
        }

        //if the object is far, than switch the arm
        /*if (action->computeDistance(&blocks_[block_id]) > 0.4f)
          switchArm(&action);*/

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
          pub_obj_moveit_.publish(blocks_surfaces_.back().collObj_);
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
        pub_obj_moveit_.publish(blocks_surfaces_.back().collObj_);
      }
      //set the table height
      else if (actionName == "set_table_height")
      {
        //TODO make a function
        blocks_surfaces_.front().size_z_ = promptUserValue("Give the table height");
        blocks_surfaces_.front().pose_.position.z =
            floor_to_base_height_ + blocks_surfaces_.front().size_z_/2.0;
        pub_obj_moveit_.publish(blocks_surfaces_.front().collObj_);
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
          pub_obj_moveit_.publish(blocks_surfaces_.front().collObj_);
          env_shown_ = true;
        }
      }
      //moving the virtual object down
      else if (checkObj(block_id) && ((actionName == "x") || (actionName == "down")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.z -= 0.05;
        blocks_[block_id].updatePose(&pub_obj_moveit_, pose);
      }
      //move the virtual object left
      else if (checkObj(block_id) && ((actionName == "s") || (actionName == "left")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.y -= 0.05;
        blocks_[block_id].updatePose(&pub_obj_moveit_, pose);
      }
      //move the virtual object up
      else if (checkObj(block_id) && ((actionName == "e") || (actionName == "up")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.z += 0.05;
        blocks_[block_id].updatePose(&pub_obj_moveit_, pose);
      }
      //move the virtual object right
      else if (checkObj(block_id) && ((actionName == "f") || (actionName == "right")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.y += 0.05;
        blocks_[block_id].updatePose(&pub_obj_moveit_, pose);
      }
      //move the virtual object
      else if (checkObj(block_id) && ((actionName == "c") || (actionName == "farther")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.x += 0.05;
        blocks_[block_id].updatePose(&pub_obj_moveit_, pose);
      }
      //move the virtual object closer
      else if (checkObj(block_id) && ((actionName == "r") || (actionName == "closer")))
      {
        geometry_msgs::Pose pose(blocks_[block_id].pose_);
        pose.position.x -= 0.05;
        blocks_[block_id].updatePose(&pub_obj_moveit_, pose);
      }
      //compute the distance to the object
      else if (checkObj(block_id) && (actionName == "compute_distance"))
      {
        ROS_INFO_STREAM("The distance to the object = "
                        << action->computeDistance(&blocks_[block_id])
                        << ", in x: " << action->computeDistance(&blocks_[block_id], 1, 0, 0)
                        << ", in y: " << action->computeDistance(&blocks_[block_id], 0, 1, 0));
      }
      else if (checkObj(block_id) && (actionName == "tf"))
      {
        //remember the initial pose
        tf::Stamped<tf::Pose> transformed_pose_robot;
        tf::TransformListener listener;

        tf::Stamped<tf::Pose> tf_obj(
              tf::Pose(
                tf::Quaternion(blocks_[block_id].pose_.orientation.x,
                               blocks_[block_id].pose_.orientation.y,
                               blocks_[block_id].pose_.orientation.z,
                               blocks_[block_id].pose_.orientation.w),
                tf::Vector3(blocks_[block_id].pose_.position.x,
                            blocks_[block_id].pose_.position.y,
                            blocks_[block_id].pose_.position.z)),
            ros::Time(0), blocks_[block_id].base_frame_);
        try
        {
          //the the base current bose
          listener.transformPose("odom", tf_obj, transformed_pose_robot);
          sleep(1.0);
          listener.transformPose("odom", tf_obj, transformed_pose_robot);
          ROS_INFO_STREAM("The object to base "
                         << " " << transformed_pose_robot.getOrigin().x()
                         << " " << transformed_pose_robot.getOrigin().y()
                         << " " << transformed_pose_robot.getOrigin().z()
                         );
        }
        catch (tf::TransformException ex)
        {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
        }
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

    pub_obj_moveit_.publish(block->collObj_);
  }

  void SimplePickPlace::getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    //cleanObjects(&blocks_, false);

    try
    {
      if (msg->meshes.size() > 0)
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
      ros::Duration(1.0).sleep();
    }
  }

} //namespace
