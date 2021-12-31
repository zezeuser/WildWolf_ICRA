/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// roborts_msgs 
#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"

#include "roborts_msgs/ShooterCmd.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/GameRobotHP.h"
#include "roborts_msgs/GameRobotBullet.h"

// #include "roborts_msgs/AllyPose.h"
#include "roborts_msgs/Target.h"
#include "roborts_msgs/DodgeMode.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/Aimtargeid.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

// communication.h
#include "communication.h"


// wifi part
#include <zmq.hpp>
#include <unistd.h>
#include <thread>



namespace roborts_decision{


struct Threshold{
        float near_dist;
        float near_angle;
        float heat_upper_bound;
        float detect_dist;

};

struct DecisionInfoPool{
        int remaining_time;
        int game_status;
        int shoot_hz;
        bool is_begin;
        bool is_master;
        bool team_blue;
        bool has_buff;
        bool has_ally;
        bool has_my_enemy;
        bool has_ally_enemy;
        bool has_first_enemy;
        bool has_second_enemy;
        bool can_shoot;
        bool can_dodge;
        bool is_supplying;
        bool is_shielding;
        bool got_last_enemy;
        bool use_refree;

        // ally enemy part
        bool has_ally_first_enemy;
        bool has_ally_second_enemy;
        //enemy hp 
        float emeny_first_hp;
        float emeny_second_hp;
        //enemy bullet
        float  emeny_first_bullet;
        float  emeny_second_bullet;
        //buff
        bool bullet_buff_active;
        bool shield_buff_active;

        bool is_hitted;
        bool is_chase;
        bool valid_camera_armor;
        bool valid_front_camera_armor;
        int remain_bullet;
        int remain_hp;
        int frequency;
        float speed;
        int ally_remain_bullet;
        int ally_remain_hp;
        int heat;
        double ally_dist;
        double first_enemy_dist;
        double second_enemy_dist;
        float ally_yaw;
        std::string strategy;
        geometry_msgs::PoseStamped ally;
        geometry_msgs::PoseStamped first_enemy;
        geometry_msgs::PoseStamped second_enemy;
        // ally enemy part
        geometry_msgs::PoseStamped ally_first_enemy;
        geometry_msgs::PoseStamped ally_second_enemy;
        // all goal
        geometry_msgs::PoseStamped my_goal;
        geometry_msgs::PoseStamped ally_goal;

        geometry_msgs::PoseStamped last_enemy;

        //reload 子弹区   shiled 补血
        geometry_msgs::PoseStamped my_reload;
        geometry_msgs::PoseStamped my_shield;
        geometry_msgs::PoseStamped opp_reload;
        geometry_msgs::PoseStamped opp_shield;
        geometry_msgs::PoseStamped start_position;
};


class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      is_in_shoot_state(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      context_(1),guardcontext_(1) ,masterSocket_(context_, ZMQ_REP), clientSocket_(context_, ZMQ_REQ),guardSocket_(guardcontext_,ZMQ_SUB){

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
   
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    // node handle
    ros::NodeHandle nh;
    // armor_info
    camera_armor_sub_ = nh.subscribe<roborts_msgs::ArmorsPos>("front_camera_robot_pos", 10, &Blackboard::FrontCameraArmorCallback, this);  // Front Camera        
    //referee_system
    robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 10, &Blackboard::RobotStatusCallback, this);
    robot_heat_sub_ = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat", 10, &Blackboard::RobotHeatCallback, this);
    robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 10, &Blackboard::RobotShootCallback, this);
    game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status", 10, &Blackboard::GameStatusCallback, this);
    robot_damage_sub_ = nh.subscribe<roborts_msgs::RobotDamage>("robot_damage", 30, &Blackboard::RobotDamageCallback, this);
    game_zone_sub_ = nh.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 10, &Blackboard::GameZoneCallback, this);
    emeny_hp_sub_ = nh.subscribe<roborts_msgs::GameRobotHP>("game_robot_hp", 30, &Blackboard::EnemyHpCallback, this);
    emeny_bullet_sub_ = nh.subscribe<roborts_msgs::GameRobotBullet>("game_robot_bullet", 30, &Blackboard::EnemyBulletCallback, this);
    //car_info
    cmd_gimbal_sub_ = nh.subscribe<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 10, &Blackboard::CmdGimbalCallback, this);
    vel_acc_sub_ = nh.subscribe<roborts_msgs::TwistAccel>("cmd_vel_acc", 10, &Blackboard::VelAccCallback, this);
    // pub
    shoot_pub_ = nh.advertise<roborts_msgs::ShooterCmd>("shoot_cmd", 10, this);
    ally_pub_ = nh.advertise<geometry_msgs::PoseStamped>("friend_pose", 10, this);
    dodge_pub_ = nh.advertise<roborts_msgs::DodgeMode>("dodge_mode", 10, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);
    target_id_pub_ = nh.advertise<roborts_msgs::Aimtargeid>("emeny_target_id",30,this);


    // all frame ID update
    std::string tf_prefix = tf::getPrefixParam(nh);
    pose_frameID = "base_link";
    gimbal_frameID = "gimbal";
    if (!tf_prefix.empty()) {
        pose_frameID = tf::resolve(tf_prefix, pose_frameID);
        gimbal_frameID = tf::resolve(tf_prefix, gimbal_frameID);
    }
    

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    use_camera = decision_config.use_camera();
    if (use_camera){
      ROS_INFO("Armor detection module wait for  connect!");
      armor_detection_actionlib_client_.waitForServer();
      ROS_INFO("Armor detection module  connected success!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }


    // initialize setting----------------------------------------------------------
    info.remaining_time = 299;
    info.shoot_hz = decision_config.shoot_hz();
    info.use_refree = decision_config.use_refree();
    info.is_begin = false;
    info.is_master = decision_config.master();
    info.team_blue = decision_config.isblue();
    info.can_shoot = decision_config.can_shoot();
    info.can_dodge = decision_config.can_dodge();
    info.has_buff = false;
    info.has_ally = false;
    info.has_my_enemy = false;
    info.has_ally_enemy = false;
    info.has_first_enemy = false;
    info.has_second_enemy = false;
    info.has_ally_first_enemy = false;
    info.has_ally_second_enemy = false;
    info.is_hitted = false;
    info.is_chase = false;
    info.is_supplying = false;
    info.is_shielding = false;
    info.got_last_enemy = false;

    info.valid_camera_armor = false;
    info.valid_front_camera_armor = false;
    info.remain_bullet = decision_config.remain_bullet();
    info.remain_hp = 2000;
    remain_hp = info.remain_hp;
    info.ally_remain_bullet = 50;
    info.ally_remain_hp = 2000;
    info.first_enemy_dist = 0;
    info.second_enemy_dist = 0;
    info.emeny_first_hp = 2000;
    info.emeny_second_hp = 2000;
    info.ally_dist = 0;
    info.heat = 0;
    info.frequency = 0;
    info.speed = 0.0;
    info.strategy = decision_config.strategy();

    // goal passport
    info.my_goal = InitMapPose();
    info.ally_goal = InitMapPose();
    
    //buff init
    info.my_shield = InitMapPose();
    info.my_reload = InitMapPose();
    info.opp_reload = InitMapPose();
    info.opp_shield = InitMapPose();

    // PostStamped information update
    info.ally = InitMapPose();
    info.ally_first_enemy = InitMapPose();
    info.ally_second_enemy = InitMapPose();
    info.last_enemy = InitMapPose();
   

    // initalize threshold----------------------------------------------------------------------
    threshold.near_dist = 0.5;  // m
    threshold.near_angle =0.2;  // rad ~ 10 degree
    threshold.detect_dist = 5.0;  // m
    threshold.heat_upper_bound = 110; // limit

    // initialize enemy
    in_dodge = false;
    _front_first_enemy = false;
    _front_second_enemy = false;
    my_shoot_1_cnt = 0;
    my_shoot_2_cnt = 0;
    ally_shoot_1_cnt = 0;
    ally_shoot_2_cnt = 0;

    // dodge config
    _dodge_in_reload = decision_config.dodge_in_reload();

    // load all shield reload points
    if (info.team_blue){     
        info.start_position = info.is_master? Point2PoseStamped(decision_config.blue().master_bot().start_position()):Point2PoseStamped(decision_config.blue().wing_bot().start_position());
        info.first_enemy  = Point2PoseStamped(decision_config.red().master_bot().start_position());
        info.second_enemy =  Point2PoseStamped(decision_config.red().wing_bot().start_position());
    } 
    else{
        info.start_position = info.is_master? Point2PoseStamped(decision_config.red().master_bot().start_position()): Point2PoseStamped(decision_config.red().wing_bot().start_position());
        info.first_enemy  = Point2PoseStamped(decision_config.blue().master_bot().start_position());
        info.second_enemy =  Point2PoseStamped(decision_config.blue().wing_bot().start_position());
    }


    // use wifi
    if (decision_config.usewifi()){
        connect_wifi = true;
        if (info.is_master){
            std::string ip = decision_config.master_ip();
            std::string port = "";
            int length = ip.length();
            for (int i=length-1; i>=0; i--){
                if (ip[i] != ':')
                    port = ip[i] + port;
                else
                    break; 
            }
            port = "tcp://*:"  + port;
            masterSocket_.bind(port);
            masterThread = std::thread(&Blackboard::CommunicateMaster, this);
        }
        else{
            clientSocket_.connect(decision_config.master_ip());
            clientThread = std::thread(&Blackboard::CommunicateClient, this);
        }
        guardSocket_.connect(decision_config.guard_ip());
        guardThread = std::thread(&Blackboard::CommunicateGuard,this);
        guardSocket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }
  }

    ~Blackboard() = default;


    // supply request
    void SupplyRequest(int number){
        info.is_supplying = true;
    }
 

    void Shoot(int hz){
        if (!is_in_shoot_state && info.can_shoot){
            is_in_shoot_state = true;
            shoot_thread = std::thread(&Blackboard::_ShootThread, this, hz);
        }
    }

    void StartFirWhl(){
        armor_detection_goal_.command = 5;
        armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
    }

    void StopShoot(){
        if (is_in_shoot_state){
            is_in_shoot_state = false;
            shoot_thread.join();
            armor_detection_goal_.command = 5 ;
            armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
        }  
    }


  void StartDodge(){
      if (info.can_dodge){
          roborts_msgs::DodgeMode dg;
          dg.is_dodge = true;
          dodge_pub_.publish(dg);
          in_dodge = true;
      }
  }

  void StopDodge(){
      roborts_msgs::DodgeMode dg;
      dg.is_dodge = false;
      dodge_pub_.publish(dg);
      in_dodge = false;
  }
  
  // Front Camera Armor Detection
  void FrontCameraArmorCallback(const roborts_msgs::ArmorsPos::ConstPtr &armors){
    //   printf("Front Camera!\n");
      if (armors->num_armor == 0){
          info.valid_front_camera_armor = false;
          _front_first_enemy = false;
          _front_second_enemy = false;
          use_camera_pose = false;
      }
      else{
            geometry_msgs::PoseStamped enemy_pose , global_pose_msg;
            tf::Stamped<tf::Pose> tf_pose ,global_tf_pose;
            enemy_pose.header.frame_id = gimbal_frameID;
            geometry_msgs::PoseStamped cur_pose = GetRobotMapPose();
            // Set up valid set
            std::set<int> valid_id_set; 
            bool valid = false;
            for (int i=0; i<armors->num_armor; i++){
                int id = armors->id[i];
                int state = armors->state[i];
                valid_id_set.insert(id); // ?????????????????!!!!!!!!!!!!!!!   is ID 0 should be all dealth points???
                if (armors->pose_A.size()>0){
                        double dx,dy;
                        // mm -> m 
                        dx = armors->pose_A[i]/1000;
                        dy = armors->pose_B[i]/1000;
                        if (dx<=0 || dx>=8 || dy<=0 || dy>=5)
                            continue;
                        if (id == 1)  _front_first_enemy = true;
                        else  _front_second_enemy = true;
                        if(dx < 1.5 || !connect_wifi){      
                            use_camera_pose = true;
                            enemy_pose.pose.position.x = dx;
                            enemy_pose.pose.position.y = dy;
                            poseStampedMsgToTF(enemy_pose, tf_pose);
                            tf_pose.stamp_ = ros::Time(0);
                            try
                            {
                                tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
                                if(_front_first_enemy){
                                    tf::poseStampedTFToMsg(global_tf_pose,info.first_enemy);
                                }else{
                                    tf::poseStampedTFToMsg(global_tf_pose,info.second_enemy);
                                }
                            }
                            catch (tf::TransformException &ex) {
                                    ROS_ERROR("tf error when transform enemy pose from camera to map");
                            }
                        }else {
                            use_camera_pose = false;
                        }
                        valid = true;
                }
            }
            // all 0.
            if (!valid) {  info.valid_front_camera_armor = false;  }
            else   { info.valid_front_camera_armor = true; }
            // Clear invalid set
            for (int id=1; id<3; id++){
                if (valid_id_set.find(id) == valid_id_set.end()){
                    if (id == 1) _front_first_enemy = false;
                    else if (id == 2) _front_second_enemy = false;
                }
            }
            valid_id_set.clear();
      }
        info.valid_camera_armor = info.valid_front_camera_armor;
        info.has_first_enemy = _front_first_enemy;
        info.has_second_enemy = _front_second_enemy;
        info.has_my_enemy = info.has_first_enemy || info.has_second_enemy;
  }

  
    // vel call back
    void VelAccCallback(const roborts_msgs::TwistAccel::ConstPtr &msg){
        my_vel_x = msg->twist.linear.x;
        my_vel_y = msg->twist.linear.y;
    }
  
    // Game Status call back
    void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr &msg){
        info.remaining_time = msg->remaining_time;
        info.is_begin = bool(msg->game_status == 4);
        info.game_status = msg->game_status;
    }

    void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr& damage) {
            // info.is_hitted = true;
        }

    // Cmd Gimbal call back
    void CmdGimbalCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg){
            bool yaw_mode = msg->yaw_mode;
            bool pitch_mode = msg->pitch_mode;
            float yaw_angle = msg->yaw_angle;
            float pitch_angle = msg->pitch_angle;
            if (yaw_mode && std::abs(yaw_angle) <= 0.2 && info.valid_front_camera_armor){
                _gimbal_can = true;
            }
            else{
                _gimbal_can = false;
            }
            // gimbal pitch
            // printf("yaw_mode:%d yaw_angle:%f, pitch_mode:%d, pitch_angle:%f\n", yaw_mode, yaw_angle, pitch_mode, pitch_angle);
        
    }

    // Robot status call back
    void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &rb_status){
        info.remain_hp = rb_status->remain_hp;
        remain_hp = info.remain_hp;
        //   info.team_blue = bool(rb_status->id == 13 || rb_status->id==14);
    }
  
    // Robot Heat call back
    void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr &rh){
        info.heat = rh->shooter_heat;
    }

    // Robot Shoot call back
    void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr &msg){
        info.frequency = msg->frequency;
        info.speed = msg->speed;
    }
    // buff zone x y call back 
    void GameZoneCallback(const roborts_msgs::GameZoneArrayConstPtr& zone) {
        geometry_msgs::PoseStamped blue_bullet_point , blue_shield_point, red_bullet_point, red_shield_point;
        bool blue_bullet_buff_active,blue_shield_buff_active,red_bullet_buff_active,red_shield_buff_active;
        auto blue_bullet = roborts_msgs::GameZone::BLUE_BULLET_SUPPLY;
        auto red_bullet = roborts_msgs::GameZone::RED_BULLET_SUPPLY;
        auto blue_shield = roborts_msgs::GameZone::BLUE_HP_RECOVERY;
        auto red_shield = roborts_msgs::GameZone::RED_HP_RECOVERY;
        // F1 ~ F6
        float buff_point[12] = {7.58, 2.16, 6.18, 3.3, 4.45, 0.9, 4.5, 4.48, 2.47, 2.07, 1.08, 3.34};
        
        for (int m = 0; m < 6; m++) {
        if (zone->zone[m].type == blue_bullet) {
            blue_bullet_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2]);
            blue_bullet_buff_active  = zone->zone[m].active;
        }
        if (zone->zone[m].type == blue_shield) {
            blue_shield_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2]);
            blue_shield_buff_active  = zone->zone[m].active;
        }
        if (zone->zone[m].type == red_bullet) {
            red_bullet_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2]);
            red_bullet_buff_active  = zone->zone[m].active;
        }
        if (zone->zone[m].type == red_shield) {
            red_shield_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2]);
            red_shield_buff_active  = zone->zone[m].active;
        }
    }
    if(info.team_blue){
    info.my_reload = blue_bullet_point;
    info.my_shield = blue_shield_point;
    info.opp_reload = red_bullet_point;
    info.opp_shield = red_shield_point;
    info.bullet_buff_active = blue_bullet_buff_active;
    info.shield_buff_active = blue_shield_buff_active;

    }
    else{
    info.my_reload = red_bullet_point;
    info.my_shield = red_shield_point;
    info.opp_reload = blue_bullet_point;
    info.opp_shield = blue_shield_point;
    info.bullet_buff_active = red_bullet_buff_active;
    info.shield_buff_active = red_shield_buff_active;
    }
  }


  void EnemyHpCallback(const roborts_msgs::GameRobotHPConstPtr& hp){
    if(info.team_blue){
      info.emeny_first_hp = hp-> red1;
      info.emeny_second_hp = hp-> red2;
    }
    else{
      info.emeny_first_hp = hp-> blue1;
      info.emeny_second_hp = hp-> blue2;
    }
  }

  void EnemyBulletCallback(const roborts_msgs::GameRobotBulletConstPtr& bullet){
    if(info.team_blue){
        info.emeny_first_bullet = bullet-> red1;
        info.emeny_second_bullet = bullet-> red2;
        if(info.is_master){
                info.remain_bullet = bullet->blue1;
        }
        else{
                info.remain_bullet = bullet-> blue2;
        }
    }
    else{
        info.emeny_first_bullet = bullet-> blue1;
        info.emeny_second_bullet = bullet-> blue2;
        if(info.is_master){
                info.remain_bullet = bullet-> red1;
        }else{
                info.remain_bullet = bullet-> red2;
        }
    }
  }

  // Enemy  // messages are always zero for enemy.
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
     
    if (feedback->detected){
      info.valid_front_camera_armor = true;
      enemy_detected_ = true;
    } else{
      info.valid_front_camera_armor = false;
      enemy_detected_ = false;
    }
  }

// 逻辑判断选择击打目标敌人
geometry_msgs::PoseStamped GetEnemy() {  // const: Can not introduce New Arguments
    geometry_msgs::PoseStamped cur_pose = GetRobotMapPose();
    geometry_msgs::PoseStamped enemyPose;
    unsigned int shoot_1_cnt = my_shoot_1_cnt + ally_shoot_1_cnt;
    unsigned int shoot_2_cnt = my_shoot_2_cnt + ally_shoot_2_cnt;
    info.first_enemy_dist = GetDistance(cur_pose,info.first_enemy);
    info.second_enemy_dist = GetDistance(cur_pose,info.second_enemy);
    // 2. fixed  1 enemy and attack
    // The different mark
    if (info.has_first_enemy && info.has_second_enemy){
        if(info.first_enemy_dist > threshold.detect_dist || info.second_enemy_dist > threshold.detect_dist)
        {   
            //有一个超范围则选择最近距离
            enemyPose = info.first_enemy_dist <= info.second_enemy_dist ? info.first_enemy : info.second_enemy;
        }
        else{
            if(info.emeny_first_hp < 400 || info.emeny_second_hp < 400  ){
                // 都不超范围则选择若有一个低血量则选择最低血量
                enemyPose = info.emeny_first_hp <= info.emeny_second_hp ? info.first_enemy : info.second_enemy;
            }
            else{
                if(info.emeny_first_bullet < 0 || info.emeny_second_bullet < 0){
                // 都不超范围则选择且无低血量若有无子弹的车则选择该车
                    enemyPose = info.emeny_first_bullet <= info.emeny_first_bullet ? info.first_enemy : info.second_enemy;
                }
                else{
                    // 都不超范围且无血量最低的车且都有子弹则选择较低血量的车
                    enemyPose = info.emeny_first_hp <= info.emeny_second_hp ? info.first_enemy : info.second_enemy;
                }
            }      
        }
        info.last_enemy = enemyPose;
        if(CanShoot()){
            if(enemyPose == info.first_enemy) my_shoot_1_cnt++; 
            else my_shoot_2_cnt++;
        }
    }
    else if (info.has_first_enemy && info.has_ally_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++; 
        }
        else{
            enemyPose = info.ally_second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    else if (info.has_second_enemy  && info.has_ally_first_enemy ){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.ally_first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++;
        }
        else{
            enemyPose = info.second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    else if (info.has_ally_first_enemy && info.has_ally_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.ally_first_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_1_cnt++;
        }
        else{
            enemyPose = info.ally_second_enemy;
            info.last_enemy = enemyPose;
            if (CanShoot()) my_shoot_2_cnt++;
        }
    }
    // Then, my first enemy
    else if (info.has_first_enemy){
        enemyPose = info.first_enemy;
        info.last_enemy = enemyPose;
        if (CanShoot()) my_shoot_1_cnt++;
    }
    else if (info.has_second_enemy){
        enemyPose = info.second_enemy;
        info.last_enemy = enemyPose;
        if (CanShoot()) my_shoot_2_cnt++;
    }
    else if (info.has_ally_first_enemy){
        enemyPose = info.ally_first_enemy;
        info.last_enemy = enemyPose;
        if (CanShoot()) my_shoot_1_cnt++;
    }
    else if (info.has_ally_second_enemy){
        enemyPose = info.ally_second_enemy;
        info.last_enemy = enemyPose;
        if (CanShoot()) my_shoot_2_cnt++;
    }
    else {
        enemyPose = info.last_enemy;
    } 
    enemyPose.pose.orientation = GetRelativeQuaternion(enemyPose, cur_pose);
    info.last_enemy = enemyPose;
    info.got_last_enemy = true;
    if(enemyPose == info.first_enemy){
        target_id_.id = 1;
        target_id_pub_.publish(target_id_);
    }
    else if(enemyPose == info.second_enemy){
        target_id_.id = 2;
        target_id_pub_.publish(target_id_);
    }
    return enemyPose;
  }

  bool CanShoot(){
       if (_gimbal_can                    // gimbal yaw can
           && info.heat <threshold.heat_upper_bound   // heater bound
        //    && ( (my_vel_x==0 && my_vel_y ==0) || in_dodge )  // velocity or dodge
           && info.valid_front_camera_armor)   // valid front camera
           { 
                return true;
           }
       else{
           StopShoot();
           return false;
       }
  }

  bool CanDodge(){
      // whether dodge in my reload!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      geometry_msgs::PoseStamped myPose = GetRobotMapPose();
      // relative yaw angle
      //云台不可移动，车辆没有被攻击，无子弹且无供给，不在装载区
      if (!_gimbal_can 
         || !info.is_hitted
         || (GetDistance(info.my_reload, myPose) <=0.65 && !_dodge_in_reload)
         || (info.remain_bullet <0 && info.bullet_buff_active)
      )
      {
          StopDodge();
          return false;
      }

      geometry_msgs::PoseStamped enemyPose = GetEnemy();
      
      // absolute distance
      double distance = GetDistance(myPose, enemyPose);
      if (distance >2.0 && distance<0.5 && info.remain_bullet >= 1000){
          StopDodge();
          return false;
      }

    // // Stop Dodge Condition in Costmap.
    //   const double radius = 0.125;
    //   const double sqrt2 = 1.414;
    //   double x,y;
    //   double cur_x, cur_y;
    //   double cost;
    //   double int_x[] = {-radius, -radius/sqrt2, 0, radius/sqrt2, radius, radius/sqrt2, 0, -radius/sqrt2};
    //   double int_y[] = {0, -radius/sqrt2, -radius, radius/sqrt2, 0, radius/sqrt2, radius, -radius/sqrt2};
    //   int u_x, u_y;
    //   x = myPose.pose.position.x;
    //   y = myPose.pose.position.y;
    //   for (int i=0; i<=7; i++){
    //         cur_x = x + int_x[i];
    //         cur_y = y + int_y[i];
    //         GetCostMap2D()->World2MapWithBoundary(cur_x, cur_y, u_x, u_y);
    //         cost = GetCostMap2D()->GetCost(u_x, u_y);
    //         if (cost >=253){
    //             StopDodge();
    //             return false;
    //         }
                      
          
    //   }
      return true;

  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // fake enemy Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }


  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

   geometry_msgs::PoseStamped GetMyGoal() const {
    return info.my_goal;
  }
  
  void SetMyGoal(geometry_msgs::PoseStamped goal){
      info.my_goal = goal;
  }
  
  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }


  bool IsBombAllyGoal(const geometry_msgs::PoseStamped goal){
      auto distance = GetDistance(goal, info.ally_goal);
      return distance < threshold.near_dist && !info.is_master;
  }

  bool hasOtherUnitsInThisArea(const geometry_msgs::PoseStamped target){
          //   是否有单位存在 该目标点中
        std::vector<geometry_msgs::PoseStamped>  others;
        double dist;
        others.push_back(info.ally);
        if (info.has_first_enemy)
                others.push_back(info.first_enemy);
        if (info.has_second_enemy)
                others.push_back(info.second_enemy);
        if (info.has_ally_first_enemy)
                others.push_back(info.ally_first_enemy);
        if (info.has_ally_second_enemy)
                others.push_back(info.ally_second_enemy);
        for (int i=0; i<others.size(); i++){
                dist = GetDistance(target, others[i]);
                if (dist <= threshold.near_dist)
                return true;
        }
        return false;

  }

 

  bool IsBombAllyGoal(const float x, const float y){
      auto distance = GetEulerDistance(x, y, info.ally_goal.pose.position.x, info.ally_goal.pose.position.y);
      return distance < threshold.near_dist;
  }

  bool IsInStuckArea(){
      geometry_msgs::PoseStamped curPose = GetRobotMapPose();
      double yaw = tf::getYaw(curPose.pose.orientation);
      float cur_x = curPose.pose.position.x, cur_y = curPose.pose.position.y;
      float x[4], y[4];
      float deg_30 = 30 / 180 * 3.1415926;
      float d = 0.3;
      x[0] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw + deg_30)));
      y[0] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw + deg_30)));
      x[1] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw - deg_30)));
      y[1] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw - deg_30)));
      x[2] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw + deg_30)));
      y[2] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw + deg_30)));
      x[3] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw - deg_30)));
      y[3] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw - deg_30)));
      unsigned int mx, my;
      for (int i=0; i<4; i++){
          GetCostMap2D()->World2Map(x[i], y[i], mx, my);
          if (GetCostMap2D()->GetCost(mx, my) <253)
            return false;
      }

      // In stuck area
      const double sqrt2 = 1.414;
      double cost;
      double c_x, c_y;
      double int_x[] = {-d, -d/sqrt2, 0, d/sqrt2, d, d/sqrt2, 0, -d/sqrt2};
      double int_y[] = {0, -d/sqrt2, -d, d/sqrt2, 0, d/sqrt2, d, -d/sqrt2};
      int u_x, u_y;
      double acc_angle = 0.0;
      double cur_angle = 0.0;
      double count = 0;
      for (int i=0; i<=7; i++){
            c_x = cur_x + int_x[i];
            c_y = cur_y + int_y[i];
            GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
            cost = GetCostMap2D()->GetCost(u_x, u_y);
            if (cost >= 253){
                cur_angle = std::atan2(int_y[i], int_x[i] + 0.00001);
                cur_angle = cur_angle >0? cur_angle : cur_angle + 2 * 3.1415926;
                acc_angle += cur_angle;
                count++;
            }   
      }
      acc_angle = acc_angle / count;
      acc_angle = acc_angle - 3.1415926;
      acc_angle = acc_angle < -3.1415926 ? acc_angle + 2 * 3.1415926 : acc_angle;

      // in case of rotate in a roll.
      c_x = cur_x + d * std::cos(acc_angle);
      c_y = cur_y + d * std::sin(acc_angle);
      GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
      cost = GetCostMap2D()->GetCost(u_x, u_y);
      if (cost >=253){
          acc_angle = acc_angle >0 ? acc_angle - 3.1415926 : acc_angle + 3.1415926;
      }
    //   printf("acc_angle: %f\n", acc_angle * 180 / 3.14);

      // get yaw
      acc_angle = acc_angle - yaw;
      double vx, vy;
      vx = cos(acc_angle);
      vy = sin(acc_angle);
      geometry_msgs::Twist tw;
      tw.linear.x = vx;
      tw.linear.y = vy;
      tw.linear.z = 0;
      // has sent cmd_vel
      cmd_vel_pub_.publish(tw);
      return true;
  }


    /******************** tool *******************/
    geometry_msgs::Quaternion GetRelativeQuaternion(const geometry_msgs::PoseStamped to,const geometry_msgs::PoseStamped from){
        double dy, dx;
        dy = to.pose.position.y - from.pose.position.y;
        dx = to.pose.position.x - from.pose.position.x;
        double yaw = std::atan2(dy, dx);    
        return tf::createQuaternionMsgFromYaw(yaw);
    }

    geometry_msgs::Quaternion GetRelativeQuaternion(const double to_x, const double to_y, const geometry_msgs::PoseStamped from){
        double dx, dy;
        dy = to_y - from.pose.position.y;
        dx = to_x - from.pose.position.x;
        double yaw = std::atan2(dy, dx);
        return tf::createQuaternionMsgFromYaw(yaw);
    }

    double GetDistance(const geometry_msgs::PoseStamped &pose1,
        const geometry_msgs::PoseStamped &pose2) {
        const geometry_msgs::Point point1 = pose1.pose.position;
        const geometry_msgs::Point point2 = pose2.pose.position;
        const double dx = point1.x - point2.x;
        const double dy = point1.y - point2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double GetEulerDistance(const float x1, const float y1, const float x2, const float y2){
        return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    }

    double GetAngle(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2) {
        const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
        const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(quaternion1, rot1);
        tf::quaternionMsgToTF(quaternion2, rot2);
        return rot1.angleShortestPath(rot2);
    }

    geometry_msgs::PoseStamped Point2PoseStamped(Point point){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.pose.position.x = point.x();
        ps.pose.position.y = point.y();
        ps.pose.position.z = point.z();
        tf::Quaternion q = tf::createQuaternionFromRPY(point.roll(),
                                                    point.pitch(),
                                                    point.yaw());
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        return ps;
    }

    geometry_msgs::PoseStamped Point2PoseStamped(float x , float y){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = ros::Time::now();
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        return ps;
    }

    const geometry_msgs::PoseStamped GetRobotMapPose() {
        UpdateRobotPose();
        return self_pose_;
    }

    const geometry_msgs::PoseStamped GetRobotGimbalMapPose(){
        return UpdateRobotGimbalPose("map");
    }

    const geometry_msgs::PoseStamped GetRobotGimbalBasePose(){
        return UpdateRobotGimbalPose(pose_frameID);
    }

    const std::shared_ptr<CostMap> GetCostMap(){
        return costmap_ptr_;
    }

    const CostMap2D* GetCostMap2D() {
        return costmap_2d_;
    }

    const unsigned char* GetCharMap() {
        return charmap_;
    }


    DecisionInfoPool info;
    Threshold threshold;

 private:
    ComInfo setComInfo(){
        ComInfo ci;
        ci.hp = info.remain_hp;
        ci.shoot_1 = my_shoot_1_cnt;
        ci.shoot_2 = my_shoot_2_cnt;
        ci.has_enemy = info.has_my_enemy;
        ci.bullet = info.remain_bullet;
        ci.pose_x = self_pose_.pose.position.x;
        ci.pose_y = self_pose_.pose.position.y;
        ci.yaw = tf::getYaw(self_pose_.pose.orientation);

        //是否看见第一个敌人
        ci.first_valid = info.has_first_enemy;
        ci.first_enemy_x = info.first_enemy.pose.position.x;
        ci.first_enemy_y = info.first_enemy.pose.position.y;
        //是否看见第二个敌人
        ci.second_valid = info.has_second_enemy;
        ci.second_enemy_x = info.second_enemy.pose.position.x;
        ci.second_enemy_y = info.second_enemy.pose.position.y;
        
        ci.goal_x = info.my_goal.pose.position.x;
        ci.goal_y = info.my_goal.pose.position.y;

        // fusion info
        ci.fusion.num_id_target = num_id_target;
        ci.fusion.num_no_id_target = num_no_id_target;
        unsigned int bound = 2 <num_id_target?2:num_id_target;
        for (int i=0; i<bound; i++) ci.fusion.id_targets[i] = id_targets[i];
        bound = 4 < num_no_id_target? 4:num_no_id_target;
        for (int i=0; i<bound; i++) ci.fusion.no_id_targets[i] = no_id_targets[i];
        return ci;
    }

    void getComInfo(ComInfo ci){  
        info.ally_remain_hp = ci.hp;
        info.ally_remain_bullet = ci.bullet;
        info.ally.header.stamp = ros::Time::now();
        info.ally.pose.position.x = ci.pose_x;
        info.ally.pose.position.y = ci.pose_y;
        info.ally.pose.position.z = 0;
        info.ally.pose.orientation = tf::createQuaternionMsgFromYaw(ci.yaw);
        info.has_ally_first_enemy = ci.first_valid;
        if (ci.first_valid){ 
            info.ally_first_enemy.pose.position.x = ci.first_enemy_x;
            info.ally_first_enemy.pose.position.y = ci.first_enemy_y;
        }
        info.has_ally_second_enemy = ci.second_valid;
        if (ci.second_valid) {
            info.ally_second_enemy.pose.position.x = ci.second_enemy_x;
            info.ally_second_enemy.pose.position.y = ci.second_enemy_y;
        }
        info.ally_goal.pose.position.x = ci.goal_x;
        info.ally_goal.pose.position.y = ci.goal_y;
        info.has_ally_enemy = ci.has_enemy;

        // fusion info
        ally_num_id_target = ci.fusion.num_id_target;
        ally_num_no_id_target = ci.fusion.num_no_id_target;
        unsigned int bound = 2 <ally_num_id_target? 2:ally_num_id_target;
        for (int i=0; i<bound; i++)  ally_id_targets[i] = ci.fusion.id_targets[i];
        bound = 4 < ally_num_no_id_target? 4 :ally_num_no_id_target;
        for (int i=0; i<bound; i++)  ally_no_id_targets[i] = ci.fusion.no_id_targets[i];

        // prepare to publish fusion data
        //   FS.id_targets.clear();
        //   FS.no_id_targets.clear();
        //   FS.num_id_target = ally_num_id_target;
        //   FS.num_no_id_target = ally_num_no_id_target;
        //   for (int i=0; i< ally_num_id_target; i++) FS.id_targets.push_back(ally_id_targets[i]);
        //   for (int i=0; i< ally_num_no_id_target; i++) FS.no_id_targets.push_back(ally_no_id_targets[i]);

        // ally shoot cnt
        ally_shoot_1_cnt = ci.shoot_1;
        ally_shoot_2_cnt = ci.shoot_2;

    }

    void getGuardInfo( CarPositionSend gi){
        // cm -> m
        if (!use_camera_pose){
            if(!info.team_blue){
                if (gi.blue1.x > 0 && gi.blue1.y > 0){
                info.first_enemy.pose.position.x = gi.blue1.y/100;
                info.first_enemy.pose.position.y = gi.blue1.x/100;
                }
                if (gi.blue2.x > 0 && gi.blue2.y  > 0){
                info.second_enemy.pose.position.x = gi.blue2.y/100;
                info.second_enemy.pose.position.y = gi.blue2.x/100;
                }
            }
            else{
                if (gi.red1.x > 0 && gi.red1.y > 0){
                info.first_enemy.pose.position.x = gi.red1.y/100;
                info.first_enemy.pose.position.y = gi.red1.x/100;
                }
                if (gi.red2.x > 0 && gi.red2.y  > 0){
                info.second_enemy.pose.position.x = gi.red2.y/100;
                info.second_enemy.pose.position.y= gi.red2.x/100;
                }
            }
        }
        
  }

  // timeit shoot bullet control
    void _ShootThread(int hz){
        ros::Rate loop(hz);
        while (ros::ok()){
            // shoot once command
            armor_detection_goal_.command = 8;
            armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
            if (! is_in_shoot_state)
                break;
            loop.sleep();   
        }
  }



  // master and client communication function
    void CommunicateMaster(){
        ComInfo ci;
        ros::Rate loop(50);
        while (ros::ok()){
            zmq::message_t rec_message;
            masterSocket_.recv(&rec_message);
            memcpy(&ci, rec_message.data(), sizeof(ci));
            getComInfo(ci);   
            //---------------------------------------------------------------------------------------------------------------------------------------
            ci = setComInfo();
            zmq::message_t send_message(sizeof(ci));
            memcpy(send_message.data(), &ci, sizeof(ci));
            
            masterSocket_.send(send_message);

            ally_pub_.publish(info.ally);
            // fusion_pub_.publish(FS);
            info.has_ally = true;
        
            loop.sleep();
        }
    }

  void CommunicateClient(){
    ComInfo ci;
    ros::Rate loop(50);
    while (ros::ok()){
        ci = setComInfo();
        zmq::message_t send_message(sizeof(ci));
        memcpy(send_message.data(), &ci, sizeof(ci));    
        clientSocket_.send(send_message);
        //----------------------------------------------------------------------------------------------------------------------------------------
        zmq::message_t rec_message;
        clientSocket_.recv(&rec_message);
        memcpy(&ci, rec_message.data(), sizeof(ci));
        getComInfo(ci);    
        geometry_msgs::PoseStamped ap;
        ally_pub_.publish(info.ally);
        // fusion_pub_.publish(FS);
        info.has_ally = true;
        loop.sleep();       
    }
  }

    // communicate guard function
    void CommunicateGuard(){
        CarPositionSend gi;
        ros::Rate loop(50);
        while (ros::ok()){
            zmq::message_t rec_message(sizeof(CarPositionSend));
            guardSocket_.recv(&rec_message);
            memcpy(&gi, rec_message.data(), sizeof(gi));
            getGuardInfo(gi);
            loop.sleep();       
        }
    }



  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();
     
    robot_tf_pose.frame_id_ = pose_frameID;
    robot_tf_pose.stamp_ = ros::Time(0);
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, self_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }


  geometry_msgs::PoseStamped UpdateRobotGimbalPose(std::string frame){
        tf::Stamped<tf::Pose> gimbal_tf_pose;
        gimbal_tf_pose.setIdentity();
        gimbal_tf_pose.frame_id_ = gimbal_frameID;
        gimbal_tf_pose.stamp_ = ros::Time(0);
        geometry_msgs::PoseStamped gimbal_pose;
        geometry_msgs::PoseStamped target_pose;
        try{
            tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
            tf_ptr_->transformPose(frame, gimbal_pose, target_pose);
        }
        catch (tf::LookupException &ex){
            ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
        }
        return target_pose;
  }

  geometry_msgs::PoseStamped InitMapPose(){
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = ros::Time::now();
      p.pose.position.x = -99;
      p.pose.position.y = -99;
      return p;
  }

  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! fake Enenmy  sub
  ros::Subscriber enemy_sub_;
  // Info  sub
  ros::Subscriber armor_sub_, camera_armor_sub_, back_camera_sub_, info_sub_, fusion_target_sub_, cmd_gimbal_sub_;
  // referee sub
  ros::Subscriber robot_status_sub_, robot_shoot_sub_, robot_heat_sub_, game_status_sub_, supply_sub_, buff_sub_, 
                                vel_acc_sub_,robot_damage_sub_ ,game_zone_sub_ , emeny_hp_sub_,emeny_bullet_sub_;
  //info  publisher
  ros::Publisher shoot_pub_, ally_pub_, fusion_pub_, dodge_pub_, supply_pub_, cmd_vel_pub_ , target_id_pub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped self_pose_, enemy_;
  //! robot gimbal pose
  geometry_msgs::PoseStamped gimbal_pose_;

  //target id
  roborts_msgs::Aimtargeid target_id_;
  // point
  geometry_msgs::Point camera_enemy_;

  //remain hp
  int remain_hp;

  // zmq
  zmq::context_t context_;
  zmq::context_t guardcontext_;
  zmq::socket_t masterSocket_, clientSocket_;
  zmq::socket_t guardSocket_;
  std::thread masterThread, clientThread;
  std::thread guardThread;
  std::thread shoot_thread;

  bool is_in_shoot_state;
  // frame id
  std::string pose_frameID, gimbal_frameID;
  bool use_camera;
  bool connect_wifi;
  bool use_camera_pose;
  // fusion
  unsigned int num_id_target, ally_num_id_target;
  unsigned int num_no_id_target, ally_num_no_id_target;
  unsigned int my_shoot_1_cnt, my_shoot_2_cnt, ally_shoot_1_cnt, ally_shoot_2_cnt;
  roborts_msgs::Target id_targets[2], ally_id_targets[2];
  roborts_msgs::Target no_id_targets[4], ally_no_id_targets[4];
  double my_vel_x{0}, my_vel_y{0};

  bool _gimbal_can, in_dodge;
  bool _front_first_enemy, _front_second_enemy;
  bool _dodge_in_reload;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
