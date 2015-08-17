/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt, Team ViGIR, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef BDI_TO_TF__
#define BDI_TO_TF__

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

namespace vigir_bdi_utils{

  class BdiToTf{
  public:

    BdiToTf(ros::NodeHandle& nh_in, ros::NodeHandle& pnh_in)
      : nh_(nh_in)
      , pelvis_available_(false)
      , bdi_z_lower_limit_(0.15)
    {
      pelvis_pose_pub_ = nh_in.advertise<geometry_msgs::PoseStamped>("/flor/state/pelvis_pose_world",1 , false);
      worldmodel_reset_pub_ = nh_in.advertise<std_msgs::String>("/flor/worldmodel/syscommand", 1, false);
      bdi_to_world_pose_pub_ = nh_in.advertise<geometry_msgs::PoseStamped>("/flor/state/bdi_to_world_pose",1 , true);

      //atlas_sim_interface_state_sub_ = nh_in.subscribe("/atlas/atlas_sim_interface_state", 1, &BdiToTf::atlasSimInterfaceStateCallback, this);
      //atlas_imu_sub_ = nh_in.subscribe("/atlas/imu", 1, &BdiToTf::atlasImuCallback, this);
      //sys_command_sub_ = nh_in.subscribe("/syscommand", 1, &BdiToTf::sysCommandCallback, this);

      robot_pose_sub_ = nh_in.subscribe("/flor/controller/atlas_pose", 1, &BdiToTf::robotPoseCallback, this);

      left_foot_pose_sub_ = nh_in.subscribe("/flor/controller/left_foot/pose", 1, &BdiToTf::leftFootPoseCallback, this);
      right_foot_pose_sub_ = nh_in.subscribe("/flor/controller/right_foot/pose", 1, &BdiToTf::rightFootPoseCallback, this);

      pose_publish_timer_ = pnh_in.createTimer(ros::Duration(0.05), &BdiToTf::posePublishTimerCallback, this, false);

      pelvis_transform_.frame_id_ = "/world";
      pelvis_transform_.child_frame_id_ = "/pelvis";

      pelvis_bdi_transform_.frame_id_ = "/world";
      pelvis_bdi_transform_.child_frame_id_ = "/pelvis_bdi";

      left_foot_transform_.frame_id_ = "/world";
      left_foot_transform_.child_frame_id_ = "/left_foot_bdi";

      right_foot_transform_.frame_id_ = "/world";
      right_foot_transform_.child_frame_id_ = "/right_foot_bdi";

      bdi_to_world_transform_.stamp_ = ros::Time::now();
      bdi_to_world_transform_.frame_id_ = "/world";
      bdi_to_world_transform_.child_frame_id_ = "/pelvis_bdi";
      bdi_to_world_transform_.setIdentity();


      geometry_msgs::PoseStamped bdi_to_world_pose;
      tfTransformToPoseStampedMsg(bdi_to_world_transform_, bdi_to_world_pose);
      bdi_to_world_pose_pub_.publish(bdi_to_world_pose);

      ankle_to_bdi_frame_.setIdentity();
      ankle_to_bdi_frame_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    }

    ~BdiToTf()
    {

    }

    void sysCommandCallback(const std_msgs::String::ConstPtr& msg)
    {
      if (msg->data == "reset"){

        //this->queryBdiToWorldTransform();

        // Publish bdi -> world pose (latched)
        geometry_msgs::PoseStamped bdi_to_world_pose;
        tfTransformToPoseStampedMsg(bdi_to_world_transform_, bdi_to_world_pose);
        bdi_to_world_pose_pub_.publish(bdi_to_world_pose);

        // Align reset time with default reset time of Pronto. Risk reduction as this will still work
        // even if the full Pronto setup fails.
        ros::Duration reset_wait_duration(3.4);

        ROS_INFO("Resetting BDI/IMU state estimator internal state, waiting %f seconds before resetting world model", reset_wait_duration.toSec());
        reset_timer_ = nh_.createTimer(reset_wait_duration, &BdiToTf::resetTimerCallback,this, true);

      }
    }

    void resetTimerCallback(const ros::TimerEvent& event)
    {
      std_msgs::String msg;
      msg.data = "reset";
      worldmodel_reset_pub_.publish(msg);
    }


    void queryBdiToWorldTransform()
    {
      if (pelvis_available_){
        ROS_INFO("Setting fixed transform between BDI and world frame");

        bdi_to_world_transform_ = pelvis_bdi_transform_;

        tf::Vector3 tmp = bdi_to_world_transform_.getOrigin();
        tmp.setZ(0.0);

        bdi_to_world_transform_.setOrigin(tmp);

        tf::Matrix3x3 rot = bdi_to_world_transform_.getBasis();

        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);

        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, yaw);

        bdi_to_world_transform_.setRotation(quat);
        bdi_to_world_transform_.stamp_ = ros::Time::now();

      }else{
        ROS_ERROR("Cannot reset pose estimate as no pelvis transform available");
      }
    }


    void robotPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        std::vector<tf::StampedTransform> transforms;

        const geometry_msgs::Point& vec (msg->pose.position);

        // Check for weirdness in BDI position (mainly intended to catch NANs)
        if ( (fabs(vec.x) < 10000000.0) && (fabs(vec.y) < 10000000.0) && (fabs(vec.z) < 10000000.0)){

            pelvis_bdi_transform_.setOrigin(tf::Vector3(vec.x, vec.y, vec.z));

        }else{
            //We just keep previous pose if BDI reported one is weird
            ROS_WARN_THROTTLE(2.0,"BDI position estimate beyond bounds, not updating pelvis position, only orientation");
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(msg->pose.orientation, orientation);

        pelvis_bdi_transform_.setRotation(orientation);

        pelvis_bdi_transform_.stamp_ = msg->header.stamp;

        // Only need bdi transform in tf for debugging, could be switched off for minor b/w saving
        if (true){
            transforms.push_back(pelvis_bdi_transform_);
        }

        // To get pelvis pose in /world frame, have to multiply with inverse bdi to world transform
        pelvis_transform_.setData(bdi_to_world_transform_.inverse() * pelvis_bdi_transform_);
        pelvis_transform_.stamp_ = pelvis_bdi_transform_.stamp_;

        transforms.push_back(pelvis_transform_);

        pelvis_available_ = true;


        tfb_.sendTransform(transforms);



    }

    /*
    void atlasSimInterfaceStateCallback(const atlas_msgs::AtlasSimInterfaceState::ConstPtr& msg )
    {
      std::vector<tf::StampedTransform> transforms;

      ros::Time now = ros::Time::now();


      // --------------- left foot tf ---------------
      left_foot_transform_.stamp_ = now;

      const geometry_msgs::Point& vec_l (msg->foot_pos_est[0].position);
      left_foot_transform_.setOrigin(tf::Vector3(vec_l.x, vec_l.y, vec_l.z));

      tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->foot_pos_est[0].orientation, orientation);

      left_foot_transform_.setRotation(orientation);

      //Transform as needed to be in our world frame
      left_foot_transform_.setData(bdi_to_world_transform_.inverse() * left_foot_transform_);

      transforms.push_back(left_foot_transform_);


      // --------------- right foot tf ---------------
      right_foot_transform_.stamp_ = now;


      const geometry_msgs::Point& vec_r (msg->foot_pos_est[1].position);
      right_foot_transform_.setOrigin(tf::Vector3(vec_r.x, vec_r.y, vec_r.z));

      //tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->foot_pos_est[1].orientation, orientation);

      right_foot_transform_.setRotation(orientation);

      //Transform as needed to be in our world frame
      right_foot_transform_.setData(bdi_to_world_transform_.inverse() * right_foot_transform_);

      transforms.push_back(right_foot_transform_);


      // --------------- pelvis tf (only published if imu orientation available) ---------------
      if (latest_imu_msg_){

        const geometry_msgs::Vector3& vec (msg->pos_est.position);

        // Check for weirdness in BDI position (mainly intended to catch NANs)
        if ( (fabs(vec.x) < 10000000.0) && (fabs(vec.y) < 10000000.0) && (fabs(vec.z) < 10000000.0)){

          if (vec.z > bdi_z_lower_limit_){
            // pelvis_bdi_transform_ is the pose as directly reported by BDI sim interface
            pelvis_bdi_transform_.setOrigin(tf::Vector3(vec.x, vec.y, vec.z));
          }else{
            pelvis_bdi_transform_.setOrigin(tf::Vector3(vec.x, vec.y, bdi_z_lower_limit_));
          }

        }else{
          //We just keep previous pose if BDI reported one is weird
          ROS_WARN_THROTTLE(2.0,"BDI position estimate beyond bounds, not updating pelvis position, only orientation");
        }


        tf::quaternionMsgToTF(latest_imu_msg_->orientation, orientation);

        pelvis_bdi_transform_.setRotation(orientation);

        pelvis_bdi_transform_.stamp_ = latest_imu_msg_->header.stamp;

        // Only need bdi transform in tf for debugging, could be switched off for minor b/w saving
        if (true){
          transforms.push_back(pelvis_bdi_transform_);
        }

        // To get pelvis pose in /world frame, have to multiply with inverse bdi to world transform
        pelvis_transform_.setData(bdi_to_world_transform_.inverse() * pelvis_bdi_transform_);
        pelvis_transform_.stamp_ = pelvis_bdi_transform_.stamp_;

        transforms.push_back(pelvis_transform_);

        pelvis_available_ = true;
      }

      tfb_.sendTransform(transforms);

    }


    void atlasImuCallback(const sensor_msgs::Imu::ConstPtr& msg )
    {
      latest_imu_msg_ = msg;
    }
    */

    void leftFootPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg )
    {
      latest_left_foot_pose_msg_ = msg;

      left_foot_transform_.stamp_ = msg->header.stamp;

      const geometry_msgs::Point& vec_l (msg->pose.position);
      left_foot_transform_.setOrigin(tf::Vector3(vec_l.x, vec_l.y, vec_l.z));

      tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->pose.orientation, orientation);

      left_foot_transform_.setRotation(orientation);

      left_foot_transform_.setData(ankle_to_bdi_frame_ * left_foot_transform_);

    }

    void rightFootPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg )
    {
      latest_right_foot_pose_msg_ = msg;

      right_foot_transform_.stamp_ = msg->header.stamp;

      const geometry_msgs::Point& vec_r (msg->pose.position);
      right_foot_transform_.setOrigin(tf::Vector3(vec_r.x, vec_r.y, vec_r.z));

      tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->pose.orientation, orientation);

      right_foot_transform_.setRotation(orientation);

      //Transform as needed to be in our world frame
      right_foot_transform_.setData(ankle_to_bdi_frame_ * right_foot_transform_);

    }

    void posePublishTimerCallback(const ros::TimerEvent& event)
    {
      if (pelvis_available_){
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = pelvis_transform_.frame_id_;
        msg.header.stamp = pelvis_transform_.stamp_;

        tfTransformToPoseMsg(pelvis_transform_, msg.pose);

        if (pelvis_pose_pub_.getNumSubscribers() > 0){
          pelvis_pose_pub_.publish(msg);
        }
      }

    }

    void tfTransformToPoseMsg(const tf::StampedTransform& transform, geometry_msgs::Pose& msg)
    {
      msg.position.x = transform.getOrigin().x();
      msg.position.y = transform.getOrigin().y();
      msg.position.z = transform.getOrigin().z();

      msg.orientation.w = transform.getRotation().getW();
      msg.orientation.x = transform.getRotation().getX();
      msg.orientation.y = transform.getRotation().getY();
      msg.orientation.z = transform.getRotation().getZ();
    }

    void tfTransformToPoseStampedMsg(const tf::StampedTransform& transform, geometry_msgs::PoseStamped& msg)
    {
      tfTransformToPoseMsg(transform, msg.pose);
      msg.header.stamp = transform.stamp_;
      msg.header.frame_id = transform.frame_id_;
    }


  protected:
    ros::Publisher pelvis_pose_pub_;
    ros::Publisher worldmodel_reset_pub_;
    ros::Publisher bdi_to_world_pose_pub_;

    //sensor_msgs::Imu::ConstPtr latest_imu_msg_;
    geometry_msgs::PoseStampedConstPtr latest_left_foot_pose_msg_;
    geometry_msgs::PoseStampedConstPtr latest_right_foot_pose_msg_;

    ros::Subscriber atlas_sim_interface_state_sub_;
    ros::Subscriber atlas_imu_sub_;
    ros::Subscriber sys_command_sub_;

    ros::Subscriber robot_pose_sub_;

    ros::Subscriber left_foot_pose_sub_;
    ros::Subscriber right_foot_pose_sub_;

    tf::TransformBroadcaster tfb_;

    ros::Timer pose_publish_timer_;
    ros::Timer reset_timer_;

    tf::StampedTransform bdi_to_world_transform_;
    tf::StampedTransform pelvis_transform_;
    tf::StampedTransform pelvis_bdi_transform_;
    tf::StampedTransform left_foot_transform_;
    tf::StampedTransform right_foot_transform_;

    tf::Transform ankle_to_bdi_frame_;


    ros::NodeHandle nh_;

    bool pelvis_available_;
    double bdi_z_lower_limit_;

  };

}

#endif

