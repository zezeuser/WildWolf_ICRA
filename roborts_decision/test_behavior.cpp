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
#include "example_behavior/nn_behavior.h"

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
        ATTACK=8,
        NN = 9,
        DEFEND = 10

};

int main(int argc, char **argv) {
        ros::init(argc, argv, "test_master");

        std::string full_path = ros::package::getPath("roborts_decision") + "/config/red_master.prototxt";
        auto blackboard = new roborts_decision::Blackboard(full_path);
        ros::Rate rate(10);

  // for filter noise command
        unsigned int count=0;
        const unsigned int count_bound = 3;
        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();
        while(ros::ok()){
        printf("-----------------------------------------\n");
        printf("Remaining Time:%d and is begin:%d\n", blackboard->info.remaining_time, blackboard->info.is_begin);
        printf("Times to supply:%d, to buff:%d\n", blackboard->info.times_to_supply, blackboard->info.times_to_buff);
        printf("\n");
        printf("Ally hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.ally_remain_hp, blackboard->info.ally_remain_bullet, blackboard->info.ally.pose.position.x, blackboard->info.ally.pose.position.y);
        printf("has_ally_enemy:%d, has_ally_first_enemy:%d, has_ally_second_endmy:%d\n", blackboard->info.has_ally_enemy, blackboard->info.has_ally_first_enemy, blackboard->info.has_ally_second_enemy);
        printf("ally first enemy pose:(%f, %f), ally second enemy pose:(%f, %f)\n", blackboard->info.ally_first_enemy.pose.position.x, blackboard->info.ally_first_enemy.pose.position.y, blackboard->info.ally_second_enemy.pose.position.x, blackboard->info.ally_second_enemy.pose.position.y);
        printf("\n");
        printf("Buff activate:  bullet : %d , shield : %d\n", blackboard->info.bullet_buff_active , blackboard->info.shield_buff_active);
        printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.remain_hp, blackboard->info.remain_bullet, blackboard->GetRobotMapPose().pose.position.x, blackboard->GetRobotMapPose().pose.position.y);    
        printf("has_my_enemy:%d, has_first_enemy:%d, has_second_enemy:%d\n", blackboard->info.has_my_enemy, blackboard->info.has_first_enemy, blackboard->info.has_second_enemy);  
        printf("my first enemy pose:(%f, %f), my second enemy pose:(%f, %f)\n", blackboard->info.first_enemy.pose.position.x, blackboard->info.first_enemy.pose.position.y, blackboard->info.second_enemy.pose.position.x, blackboard->info.second_enemy.pose.position.y);
        printf("my goal:(%f, %f), ally goal:(%f, %f)\n", blackboard->info.my_goal.pose.position.x, blackboard->info.my_goal.pose.position.y, blackboard->info.ally_goal.pose.position.x, blackboard->info.ally_goal.pose.position.y);
        printf("Is Stuck:%d\n", blackboard->IsInStuckArea());
        rate.sleep();
  }


  return 0;
}

