#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/reload_behavior.h"
#include "example_behavior/shield_behavior.h"
#include "example_behavior/test_behavior.h"
#include "example_behavior/ambush_behavior.h"
#include "example_behavior/attack_behavior.h"
#include "example_behavior/defense_behavior.h"

enum BehaviorStateEnum{
        INIT = -1,
        BACKBOOT = 0,
        CHASE=1,
        SEARCH=2,
        ESCAPE=3,
        PATROL=4,
        RELOAD=5,
        SHIELD=6,
        AMBUSH=7,
        ATTACK=8

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rmul_behavior_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/rmul_behave.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor;
    auto blackboard = new roborts_decision::Blackboard(full_path);
    if(blackboard->info.can_shoot){
        blackboard->StartFirWhl();
    }
    // Behavior State Enum
    BehaviorStateEnum last_state, cur_state;
    last_state = BehaviorStateEnum::INIT;
    cur_state = BehaviorStateEnum::INIT;

    roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);
    roborts_decision::ReloadBehavior     reload_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::ShieldBehavior     shield_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::TestBehavior      test_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::AmbushBehavior    ambush_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::AttackBehavior    attack_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::DefenseBehavior   defense_behavior(chassis_executor, blackboard, full_path);
    ros::Rate rate(10);
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    // for filter noise command
    unsigned int count=0;
    const unsigned int count_bound = 3;
    while(ros::ok()){
            ros::spinOnce();
            // printf("-----------------------------------------\n");
            // printf("Remaining Time:%d and is begin:%d\n", blackboard->info.remaining_time, blackboard->info.is_begin);
            // printf("\n");
            // printf("Ally hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.ally_remain_hp, blackboard->info.ally_remain_bullet, blackboard->info.ally.pose.position.x, blackboard->info.ally.pose.position.y);
            // printf("has_ally_enemy:%d, has_ally_first_enemy:%d, has_ally_second_endmy:%d\n", blackboard->info.has_ally_enemy, blackboard->info.has_ally_first_enemy, blackboard->info.has_ally_second_enemy);
            // printf("ally first enemy pose:(%f, %f), ally second enemy pose:(%f, %f)\n", blackboard->info.ally_first_enemy.pose.position.x, blackboard->info.ally_first_enemy.pose.position.y, blackboard->info.ally_second_enemy.pose.position.x, blackboard->info.ally_second_enemy.pose.position.y);
            // printf("\n");
            // printf("Buff activate:  bullet : %d , shield : %d\n", blackboard->info.bullet_buff_active , blackboard->info.shield_buff_active);
            printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.remain_hp, blackboard->info.remain_bullet, blackboard->GetRobotMapPose().pose.position.x, blackboard->GetRobotMapPose().pose.position.y);    
            printf("has_my_enemy:%d, has_first_enemy:%d, has_second_enemy:%d\n", blackboard->info.has_my_enemy, blackboard->info.has_first_enemy, blackboard->info.has_second_enemy);  
            printf("my first enemy pose:(%f, %f), my second enemy pose:(%f, %f)\n", blackboard->info.first_enemy.pose.position.x, blackboard->info.first_enemy.pose.position.y, blackboard->info.second_enemy.pose.position.x, blackboard->info.second_enemy.pose.position.y);
            printf("my goal:(%f, %f), ally goal:(%f, %f)\n", blackboard->info.my_goal.pose.position.x, blackboard->info.my_goal.pose.position.y, blackboard->info.ally_goal.pose.position.x, blackboard->info.ally_goal.pose.position.y);
            printf("Is Stuck:%d\n", blackboard->IsInStuckArea());
            printf("use_refree:%d\n", blackboard->info.use_refree);
            if(blackboard->info.is_begin || blackboard->info.use_refree){   //!!!!!!!!!!!!test
                // 若陷入障碍层中，发布速度走出障碍
                blackboard->IsInStuckArea();
                // shoot and dodge command when game is on!
                if (last_state != BehaviorStateEnum::ESCAPE){
                        if (blackboard->CanDodge()) {
                        blackboard->StartDodge(); 
                        // defense_behavior.Run();
                        }
                }
                if (blackboard->CanShoot()) blackboard->Shoot(blackboard->info.shoot_hz);
                // my pose
                geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();
                // attack originally for chasing
                if (blackboard->info.strategy == "attack"){
                        // state decision behavior
                        if (blackboard->info.remain_hp >= 400){
                            if (blackboard->info.remain_bullet > 0){
                                if (!blackboard->info.has_my_enemy  && !blackboard->info.is_chase){
                                        cur_state = BehaviorStateEnum::SEARCH;
                                }
                                else{
                                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                                        cur_state = BehaviorStateEnum::CHASE;
                                        }
                                        //    cur_state = BehaviorStateEnum::CHASE;
                                        //    cur_state = BehaviorStateEnum::AMBUSH;
                                        //   cur_state = BehaviorStateEnum::ATTACK;
                                }
                            }
                        }
                        else{
                            if (blackboard->info.remain_bullet > 0){
                                cur_state = BehaviorStateEnum::CHASE;
                            }
                            else{
                                cur_state = BehaviorStateEnum::ESCAPE;
                            }
                        }
                }

                // filter
                if (!(last_state == BehaviorStateEnum::RELOAD  ||  last_state == BehaviorStateEnum::SHIELD)){
                        count = last_state != cur_state ? count + 1: 0;
                        cur_state = count>=count_bound? cur_state: last_state;
                }
            }
                // cancel last state
            if ( (last_state != BehaviorStateEnum::INIT && last_state != cur_state)  || blackboard->info.remain_hp<=0 ){
                    switch (last_state){
                            case BehaviorStateEnum::CHASE:
                                    chase_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::ESCAPE:
                                    escape_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::SEARCH:
                                    search_behavior.Cancel();
                                    break;
                    }
            }

            // remain hp is 0, them dead!
            if (blackboard->info.remain_hp <=0){
                    ros::shutdown();
                    break;
            }


            switch (cur_state){
                    case BehaviorStateEnum::CHASE:
                            chase_behavior.Run();
                            std::cout<<"CHASE" << std::endl;
                            break;
                    case BehaviorStateEnum::ESCAPE:
                            escape_behavior.Run();
                            std::cout<<"ESCAPE" << std::endl;
                            break;
                    case BehaviorStateEnum::SEARCH:
                            search_behavior.Run();
                            std::cout<<"SEARCH" << std::endl;
                            break;

            }

            last_state = cur_state;

            rate.sleep();

    }

    return 0;
}
