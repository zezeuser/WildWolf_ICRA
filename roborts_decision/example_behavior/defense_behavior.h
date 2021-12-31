#ifndef ROBORTS_DECISION_DEFEND_BEHAVIOR_H
#define ROBORTS_DECISION_DEFEND_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class DefenseBehavior {
 public:
  DefenseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {
  }

  void Run() {
    float move_dist = 0.20;               //车身左右移动的距离
    geometry_msgs::PoseStamped swing_goal;
    swing_goal = blackboard_->GetRobotMapPose();
    swing_goal.header.frame_id = "map";
    double yaw = tf::getYaw(swing_goal.pose.orientation);
    ROS_INFO("Got pose.");
    float move_x = sin(yaw) * move_dist;    // x轴方向上移动距离
    float move_y = -cos(yaw) * move_dist;   // y轴方向上移动距离
    tf::Quaternion tf_Quaternion;
    swing_goal.pose.position.z = 0;
    ros::Rate rate(10);
    ROS_INFO("Is swing");
    // 更新 x, y 轴坐标
    // swing_goal.pose.position.x += move_x;
    // swing_goal.pose.position.y += move_y;
    double swing_goal_yaw  = yaw + 0.5;
    tf_Quaternion = tf::createQuaternionFromYaw(swing_goal_yaw);
    tf::quaternionTFToMsg(tf_Quaternion , swing_goal.pose.orientation);
    auto executor_state = Update();
    chassis_executor_->Execute(swing_goal);
    int count = 0;
    while(count > 200){   
        count ++ ;
    }
    count = 0;
    // swing_goal.pose.position.x -= move_x;
    // swing_goal.pose.position.y -= move_y;
    swing_goal_yaw  = yaw - 0.5;
    tf_Quaternion = tf::createQuaternionFromYaw(swing_goal_yaw);
    tf::quaternionTFToMsg(tf_Quaternion , swing_goal.pose.orientation);
    chassis_executor_->Execute(swing_goal);
    while(count > 200){   
        count ++ ;
    }
}

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }


  ~DefenseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  bool cancel_;
};
}

#endif //ROBORTS_DECISION_DEFEND_BEHAVIOR_H
