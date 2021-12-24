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
    auto executor_state = Update();
    geometry_msgs::PoseStamped my = blackboard_->GetRobotMapPose();
    float move_dst = 0.20;               //车身左右移动的距离
    geometry_msgs::PoseStamped swing_goal;
    swing_goal.header.frame_id = "map";
    swing_goal = blackboard_->GetRobotMapPose();
    ROS_INFO("Got pose.");
    tf::Quaternion cur_q;
    tf::quaternionMsgToTF(swing_goal.pose.orientation, cur_q);
    double r, p, y;
    tf::Matrix3x3(cur_q).getRPY(r, p, y); // 从位置信息中获取车身的角度
    float move_x = asin(y) * move_dst;    // x轴方向上移动距离
    float move_y = -acos(y) * move_dst;   // y轴方向上移动距离
    swing_goal.pose.position.z = 0;
    ros::Rate rate(20);
    ROS_INFO("Is swing");
    // 更新 x, y 轴坐标
    while(blackboard_->CanDodge()){
        if (blackboard_->info.has_my_enemy){
                swing_goal.pose.position.x += move_x;
                swing_goal.pose.position.y += move_y;
                chassis_executor_->Execute(swing_goal);
                rate.sleep();
                swing_goal.pose.position.x -= move_x;
                swing_goal.pose.position.y -= move_y;
                chassis_executor_->Execute(swing_goal);
                rate.sleep();
        }
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
