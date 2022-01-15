#ifndef ROBORTS_DECISION_DEFEND_BEHAVIOR_H
#define ROBORTS_DECISION_DEFEND_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/SwingMode.h"
#include <roborts_msgs/GimbalInfo.h>

namespace roborts_decision {
class DefenseBehavior {
 public:
  DefenseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {
        gimbal_angle_.yaw_angle = 0;
        gimbal_angle_.yaw_mode = true;
        gimbal_publisher_ = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 50, this);
        swing_publisher_ = nh_.advertise<roborts_msgs::SwingMode>("swing_mode", 50, this);
        init_ = false;
  }

  void Run() {
    if(!init_){
        UpdateGobalFrame();
        init_ = true;
    }
    // 获取 map 坐标系下的 gimbal 的坐标
    swing_mode_.swing_mode = true;
    if(Update() != BehaviorState::IDLE){
        swing_mode_.guidance = true;
    }else{
        swing_mode_.guidance = false;
    }
    std::cout<< swing_mode_.guidance << std::endl;
    swing_publisher_.publish(swing_mode_);
    if(!blackboard_->info.has_my_enemy){
        float gyro_yaw = blackboard_->info.gyro_yaw;
        float toward_yaw = tf::getYaw(blackboard_->info.toward_goal.pose.orientation);
        float target_yaw = gyro_yaw - toward_yaw -  relative_yaw_;
        // target_yaw = (target_yaw + last_target_yaw_) * 0.5;
        gimbal_angle_.yaw_angle = -target_yaw;
        // last_target_yaw_ = target_yaw;
        gimbal_publisher_.publish(gimbal_angle_);
        std::cout << "relative_yaw : " <<  relative_yaw_ << std::endl;
    }

}

  void Cancel() {
    swing_mode_.swing_mode = false;
    if(Update() != BehaviorState::IDLE){
        swing_mode_.guidance = true;
    }else{
        swing_mode_.guidance = false;
    }
  }
  
  BehaviorState Update() {
    return chassis_executor_->Update();
  }
  
  void UpdateGobalFrame(){
      // 使陀螺仪的坐标系与 map 坐标系重合
        float gyro_yaw = blackboard_->info.gyro_yaw;
        geometry_msgs::PoseStamped gimbal_pose = blackboard_->GetRobotGimbalMapPose();
        float map_yaw = tf::getYaw(gimbal_pose.pose.orientation);
        relative_yaw_ = gyro_yaw - map_yaw;
        ROS_INFO("relative_yaw_ : %f  gyro_yaw: %f  map_yaw:%f  "  , relative_yaw_ , gyro_yaw , map_yaw);
  }

  ~DefenseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  ros::NodeHandle nh_;
  roborts_msgs::GimbalAngle gimbal_angle_;
  roborts_msgs::SwingMode swing_mode_;
  ros::Publisher gimbal_publisher_;
  ros::Publisher swing_publisher_;
  ros::Subscriber odom_info_;
  double relative_yaw_;
  double last_target_yaw_;
  bool init_;
};
}

#endif //ROBORTS_DECISION_DEFEND_BEHAVIOR_H
