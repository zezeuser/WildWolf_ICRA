
#include "mv_video_capture.hpp"
#include "opencv2/opencv.hpp"
#include "basic_armor.hpp"
#include "basic_pnp.hpp"

#include <condition_variable>
#include <thread>
#include <boost/thread.hpp>
#include "io/io.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "actionlib/server/simple_action_server.h"
#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/Aimtargeid.h"
#include "roborts_msgs/ArmorsPos.h"
#include "roborts_msgs/GimbalInfo.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/RobotShoot.h"
#include "state/node_state.h"
#include "state/error_code.h"
#include "kalman.h"
#include  <nadjieb/mjpeg_streamer.hpp>

using roborts_common::NodeState;
using roborts_common::ErrorInfo;
using MJPEGStreamer = nadjieb::MJPEGStreamer;
constexpr int S = 2;

class ArmorDetectionNode
{
public:
    explicit ArmorDetectionNode();
    void StartThread();
    void PauseThread();
    void StopThread();
    void ExecuteLoop();
    void StartFricWhl();
    void StopFricWhl();
    void ShootOnce();
    void ShootContinuous();
    void ShootNone();
    void ArrangeMsg(int _armor_id , float _yaw_angle , float _depth ) ;
    void Clear();
    void ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data);
    void TargeIdCallBack(const roborts_msgs::Aimtargeid::ConstPtr& id);
    void UpdateGimbalDataCallBack(const roborts_msgs::GimbalInfo::ConstPtr& data);
    void UpdateBulletVelocityCallBcak(const roborts_msgs::RobotShoot::ConstPtr& velocity);
    
    
    ~ArmorDetectionNode();

    private:
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
    NodeState node_state_;
    ErrorInfo error_info_;
    bool initialized_;
    bool running_;

    mindvision::VideoCapture* mv_capture_;
    basic_armor::Num num_;
    basic_armor::Detector basic_armor_;
    basic_pnp::PnP pnp_;
    MJPEGStreamer streamer;
    uart::Receive_Data data_;
    cv::Mat img_;
    using _Kalman = Kalman<1, S>;
    _Kalman kalman;

    std::thread armor_detection_thread_;
    std::mutex mutex_;
    std::condition_variable condition_var_;

    ros::NodeHandle nh_;
    ros::NodeHandle enemy_nh_;

    // 发布两个机器人对应的坐标和 id  pub
    ros::Publisher camera_armor_pub_;
    // 控制云台 pub
    ros::Publisher enemy_info_pub_;
    // 目标敌人 id sub
    ros::Subscriber target_id_sub_;
    // gimbal info sub
    ros::Subscriber gimbal_info_sub_;
    std::shared_ptr<tf::TransformListener> tf_ptr_;
    actionlib::SimpleActionServer<roborts_msgs::ArmorDetectionAction> as_;
    ros::ServiceClient fricwhl_client_;
    ros::ServiceClient shoot_client_;
    roborts_msgs::FricWhl fricwhl_;
    roborts_msgs::ShootCmd shootcmd_;
    roborts_msgs::GimbalAngle gimbal_angle_;
    roborts_msgs::ArmorsPos front_camera_robot_pos;
    geometry_msgs::PoseStamped gimbal_pose_;
    bool fricwhl_open_;
    bool shooting_;
    // 击打目标 id
    int target_id_;

    // 击打目标敌人坐标
    double x_;
    double y_;
    // 是否探测到装甲板
    bool detected_enemy_;
    unsigned long demensions_;
    unsigned int undetected_count_;
    unsigned int undetected_armor_delay_;

    float   acceleration_{0};
    


};


