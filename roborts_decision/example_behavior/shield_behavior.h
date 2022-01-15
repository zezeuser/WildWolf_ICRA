#ifndef ROBORTS_DECISION_SHIELD_BEHAVIOR_H
#define ROBORTS_DECISION_SHIELD_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ShieldBehavior {
 public:
  ShieldBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       count(0),
                                                       count_limit(100) {
    shield_position_.header.frame_id = "map";
    shield_position_.pose.orientation.x = 0;
    shield_position_.pose.orientation.y = 0;
    shield_position_.pose.orientation.z = 0;
    shield_position_.pose.orientation.w = 1;

    shield_position_.pose.position.x = 0;
    shield_position_.pose.position.y = 0;
    shield_position_.pose.position.z = 0;

    starting_buff = false;
  }

  void Run() {
    
    auto executor_state = Update();
    UpdateShieldZoon();
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    auto dx = shield_position_.pose.position.x - robot_map_pose.pose.position.x;
    auto dy = shield_position_.pose.position.y - robot_map_pose.pose.position.y;
    geometry_msgs::PoseStamped enemy_position = blackboard_->GetEnemy();
    blackboard_->SetMyToward(enemy_position);
    shield_position_.pose.orientation = blackboard_->GetRelativeQuaternion(shield_position_,enemy_position);

    if (executor_state != BehaviorState::RUNNING) {
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2
        && !blackboard_->IsBombAllyGoal(shield_position_)
        && !starting_buff) {
          chassis_executor_->Execute(shield_position_);
          blackboard_->SetMyGoal(shield_position_);
      }
      else if (starting_buff){
           if (blackboard_->info.has_my_enemy || blackboard_->info.valid_front_camera_armor){
               geometry_msgs::PoseStamped enemy = blackboard_->GetEnemy();
               robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, robot_map_pose);
               chassis_executor_->Execute(robot_map_pose);
               blackboard_->SetMyGoal(robot_map_pose);
           }
      }  
    }

    if (std::sqrt(std::pow(dx,2) + std::pow(dy,2))<= blackboard_->threshold.near_dist  && !starting_buff){
        new_thread = new std::thread(boost::bind(&ShieldBehavior::CountLoop, this));
        starting_buff = true;
        blackboard_->info.is_shielding = true;
    }

    if (count >= count_limit){
        std::cout<<"Shield is finised!" << std::endl;
        blackboard_->info.is_shielding = false;
        starting_buff = false;
        new_thread->join();
        count = 0;
    }
  }

  void Cancel() {
      chassis_executor_->Cancel();
      starting_buff = false;
      blackboard_->info.is_shielding = false;
      count = 0;
    
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void UpdateShieldZoon(){
    shield_position_ = blackboard_ -> info.my_shield;
  }

  void CountLoop(){
     ros::Rate loop(10);
     while (count < count_limit){
        count++;
        loop.sleep();
     }  
  }

  ~ShieldBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped shield_position_;

 

  int count, count_limit;
  bool starting_buff;
  std::thread * new_thread;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
