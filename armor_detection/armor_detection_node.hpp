
#include "mv_video_capture.hpp"
#include "opencv2/opencv.hpp"
#include "basic_armor.hpp"
#include "basic_pnp.hpp"

#include <condition_variable>
#include <thread>
#include <boost/thread.hpp>
#include "io/io.h"
#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/ArmorsPos.h"
#include "state/node_state.h"
#include "state/error_code.h"

using roborts_common::NodeState;
using roborts_common::ErrorInfo;

class ArmorDetectionNode
{
public:
    explicit ArmorDetectionNode();
    void StartThread();
    void PauseThread();
    void StopThread();
    void ExecuteLoop();
    void PublishGimbalMsgs();
    void PublishInfoMsgs();
    void Clear();
    void ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data);
    ~ArmorDetectionNode();
private:
    NodeState node_state_;
    ErrorInfo error_info_;
    bool initialized_;
    bool running_;

    mindvision::VideoCapture* mv_capture_;
    basic_armor::Num num_;
    basic_armor::Detector basic_armor_;
    basic_pnp::PnP pnp_;
    uart::SerialPort serial_;
    cv::Mat img_;

    std::thread armor_detection_thread_;
    std::mutex mutex_;
    std::condition_variable condition_var_;

    ros::NodeHandle nh_;
    ros::NodeHandle enemy_nh_;

    // 发布两个机器人对应的坐标和 id 
    ros::Publisher camera_armor_pub_;
    //控制云台
    ros::Publisher enemy_info_pub_;
    actionlib::SimpleActionServer<roborts_msgs::ArmorDetectionAction> as_;
    roborts_msgs::GimbalAngle gimbal_angle_;
    roborts_msgs::ArmorsPos fornt_camera_robot_pos_;

    //控制云台
    double pitch_;
    double yaw_;

    // 击打目标敌人坐标
    double x_;
    double y_;
    double z_;

    bool detected_enemy_;
    unsigned long demensions_;
    unsigned int undetected_count_;
    unsigned int undetected_armor_delay_;

    //检测到的装甲板数量    0 ， 1， 2
    int num_armor_;
    //检测到的装甲板 id          1号机器人 , 2号机器人  
    // id_无顺序但要对应于 state_ 中的顺序
    std::vector< int > armor_id_;

    //检测到的第一个机器人的相对坐标
    double pose_A_[2];
    //检测到的第二个机器人的相对坐标
    double pose_B_[2];



};


