#ifndef ROBORTS_DECISION_SENTRY_BEHAVIOR_H
#define ROBORTS_DECISION_SENTRY_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "roborts_msgs/GimbalAngle.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <geometry_msgs/PoseStamped.h>

namespace roborts_decision {
class SentryBehavior {
 public:
  SentryBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard,
               const std::string & proto_file_path) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) {
        yaw_count_ =  0;
        pitch_count_ = 0;
        last_time_ = ros::Time::now();
        gimbal_angle_.pitch_angle = 0;
        gimbal_angle_.yaw_angle = 0;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.yaw_mode = false;
        gimbal_publisher_ = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 10, this);
        yaw_direction = true;
        pitch_direction = true;
      }
      
  void Run() {
    geometry_msgs::PoseStamped gimbal_angle =  blackboard_->GetRobotGimbalBasePose();
    double roll ,pitch ,yaw ;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(gimbal_angle.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    float degree_10 = 0.174;
    float degree_45 = 0.785;
    float degree_6 = 0.1;
    float degree_3 = 0.052; 
    // pitch = pitch > degree_10 ? degree_10 : pitch;    // 0.523 rad = 30 degree
    // pitch = pitch < 0 ? 0 : pitch;
    yaw = yaw > degree_45 ? degree_45 : yaw; // 0.785 rad = 45 degree
    yaw = yaw < -degree_45 ? -degree_45 : yaw; 
    
    if (yaw_direction){
        gimbal_angle_.yaw_angle = yaw + degree_6;
    }else{
        gimbal_angle_.yaw_angle = yaw - degree_6;
    }
    if(gimbal_angle_.yaw_angle > degree_45){
        yaw_direction = false;
    }
    if(gimbal_angle_.yaw_angle <= -degree_45){
        yaw_direction = true;
    }


    // pitch_direction : True mean down  , False mean up  
    if (pitch_direction){
        gimbal_angle_.pitch_angle = pitch + degree_3;
    }else{
        gimbal_angle_.pitch_angle = pitch - degree_3;
    }
    if(pitch  > degree_10){
        pitch_direction = false;
    }
    if(pitch <= 0){
        pitch_direction = true;
    }

    
    gimbal_publisher_.publish(gimbal_angle_);
    last_time_ = ros::Time::now();
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~SentryBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  
  //! perception information
  Blackboard* const blackboard_;
  
  ros::NodeHandle nh_;
  roborts_msgs::GimbalAngle gimbal_angle_;
  ros::Publisher gimbal_publisher_;

  float yaw_count_;
  float pitch_count_;

  bool yaw_direction;
  bool pitch_direction;

  ros::Time last_time_;
  
};
}

#endif //ROBORTS_DECISION_SENTRY_BEHAVIOR_H
