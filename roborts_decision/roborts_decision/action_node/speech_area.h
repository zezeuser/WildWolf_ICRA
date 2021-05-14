#ifndef ROBORTS_DECISION_SPEECH_AREA_H
#define ROBORTS_DECISION_SPEECH_AREA_H
#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "goal_factory.h"
#include "../executor/chassis_executor.h"
#include <actionlib/client/simple_action_client.h>
//#include "roborts_msgs/GimbalSwingAction.h"
#include "roborts_msgs/GimbalActionlib.h"
namespace roborts_decision{
    class Speech_area : public ActionNode {
        public:
            Speech_area(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("Speech_area", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
                    chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
                }

            virtual ~Speech_area() = default;

        private:
            std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
            GoalFactory::GoalFactoryPtr goal_factory_ptr_;
            ros::Time start_time_;
            virtual void OnInitialize() {
                start_time_=ros::Time::now();
                ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
            };

            virtual BehaviorState Update() {
                ROS_INFO("star speech_goal");
                ros::Time now_time=ros::Time::now();
                if (blackboard_ptr_ ->GetSpeechcount() == 9)    //停止
                {
                    goal_factory_ptr_ ->CancelGoal();
                }
                
                ROS_INFO("search time:%f",(now_time-start_time_).toSec());
                if((now_time-start_time_).toSec()>3.0){
	                goal_factory_ptr_->CancelGoal();
                    blackboard_ptr_->SetSwing(false);
    	            return BehaviorState::SUCCESS;
                }

                goal_factory_ptr_->Speech_go();    
                ROS_INFO("GO ！");
                blackboard_ptr_->SetActionState(chassis_executor_ptr_->Update());   
                return blackboard_ptr_->GetActionState();
            }

            virtual void OnTerminate(BehaviorState state) {
                switch (state){
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelGoal();
                    ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                break;
                case BehaviorState::SUCCESS:
                    blackboard_ptr_->SetSwing(false);
                    ROS_INFO("%s %s SUCCESS!",name_.c_str(),__FUNCTION__);
                break;
                case BehaviorState::FAILURE:
                    ROS_INFO("%s %s FAILURE!",name_.c_str(),__FUNCTION__);
                break;
                default:
                    ROS_INFO("%s %s ERROR!",name_.c_str(),__FUNCTION__);
                return;
                }
        }

    };

}//namespace roborts_decision

#endif

