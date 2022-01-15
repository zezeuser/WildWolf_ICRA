#ifndef ROBORTS_DECISION_RELOAD_BEHAVIOR_H
#define ROBORTS_DECISION_RELOAD_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision
{
    class ReloadBehavior
    {
        public:
                ReloadBehavior(ChassisExecutor *&chassis_executor,
                               Blackboard *&blackboard,
                               const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                                     blackboard_(blackboard),
                                                                     count(0),
                                                                     count_limit(20)
                {

                        start_supplying = false;
                        reload_position_.header.frame_id = "map";
                        reload_position_.pose.orientation.x = 0;
                        reload_position_.pose.orientation.y = 0;
                        reload_position_.pose.orientation.z = 0;
                        reload_position_.pose.orientation.w = 1;

                        reload_position_.pose.position.x = 0;
                        reload_position_.pose.position.y = 0;
                        reload_position_.pose.position.z = 0;
                }

void Run() {
        auto executor_state = Update();
        UpdateReloadZoon();
        auto robot_map_pose = blackboard_->GetRobotMapPose();
        auto dx = reload_position_.pose.position.x - robot_map_pose.pose.position.x;
        auto dy = reload_position_.pose.position.y - robot_map_pose.pose.position.y;
        geometry_msgs::PoseStamped enemy_position = blackboard_->GetEnemy();
        blackboard_->SetMyToward(enemy_position);
        reload_position_.pose.orientation = blackboard_->GetRelativeQuaternion(reload_position_,enemy_position);

        if (executor_state != BehaviorState::RUNNING) {
        if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2
            && !blackboard_->IsBombAllyGoal(reload_position_)
            && !start_supplying) {
            chassis_executor_->Execute(reload_position_);
            blackboard_->SetMyGoal(reload_position_);
        }
        else if (start_supplying){
            if (blackboard_->info.has_my_enemy || blackboard_->info.valid_front_camera_armor){
                geometry_msgs::PoseStamped enemy = blackboard_->GetEnemy();
                robot_map_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, robot_map_pose);
                chassis_executor_->Execute(robot_map_pose);
                blackboard_->SetMyGoal(robot_map_pose);
            }
        }  
        }

        if (std::sqrt(std::pow(dx,2) + std::pow(dy,2))<= blackboard_->threshold.near_dist  && !start_supplying){
            new_thread = new std::thread(boost::bind(&ReloadBehavior::CountLoop, this));
            start_supplying = true;
            blackboard_->info.is_supplying = true;
        }

        if (count >= count_limit){
            std::cout<<"Shield is finised!" << std::endl;
            blackboard_->info.is_supplying = false;
            start_supplying = false;
            new_thread->join();
            count = 0;
        }
}

void CountLoop(){
     ros::Rate loop(10);
     while (count < count_limit){
        count++;
        loop.sleep();
     }  
}

void Cancel()
{
        chassis_executor_->Cancel();
        count = 0;
        start_supplying = false;
}

BehaviorState Update()
{
        return chassis_executor_->Update();
}

void UpdateReloadZoon()
{
        reload_position_ = blackboard_->info.my_reload;
}

~ReloadBehavior() = default;

    private:
        //! executor
        ChassisExecutor *const chassis_executor_;
        //! perception information
        Blackboard *const blackboard_;
        //! boot position
        geometry_msgs::PoseStamped reload_position_;

        int count, count_limit;
        std::thread *new_thread;
        bool start_supplying;
    };
}

#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
