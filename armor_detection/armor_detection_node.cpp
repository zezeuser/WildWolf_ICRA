#include  "armor_detection_node.hpp"

ArmorDetectionNode::ArmorDetectionNode():
node_state_(roborts_common::IDLE),
initialized_(false),
detected_enemy_(false),
undetected_armor_delay_(0),
as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false),
basic_armor_("/home/zeze/catkin_ws/src/armor_detection/configs/armor/basic_armor_config.xml"),
pnp_("/home/zeze/catkin_ws/src/armor_detection/configs/camera/mv_camera_config_337.xml",
                                         "/home/zeze/catkin_ws/src/armor_detection/configs/angle_solve/basic_pnp_config.xml"),
serial_("/home/zeze/catkin_ws/src/armor_detection/configs/serial/uart_serial_config.xml"),
num_(8)
{            
    mv_capture_ = new mindvision::VideoCapture(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));
    enemy_nh_ = ros::NodeHandle();
    enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    camera_armor_pub_ = enemy_nh_.advertise<roborts_msgs::ArmorsPos>("front_camera_robot_pos", 10);
    initialized_ = true;
    as_.start();
}

ArmorDetectionNode::~ArmorDetectionNode()
{
    StopThread();
}

void ArmorDetectionNode::ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) {
  roborts_msgs::ArmorDetectionFeedback feedback;
  roborts_msgs::ArmorDetectionResult result;
  bool undetected_msg_published = false;

  if(!initialized_){
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_INFO("Initialization Failed, Failed to execute action!");
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }
  ros::Rate rate(25);
  while(ros::ok()) {

    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }
    std::lock_guard<std::mutex> guard(mutex_);
    {
      if (undetected_count_ != 0) {
        feedback.detected = true;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp    = ros::Time::now();
        //相对坐标
        feedback.enemy_pos.pose.position.x = x_;
        feedback.enemy_pos.pose.position.y = y_;
        feedback.enemy_pos.pose.position.z = 0 ;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = false;
      } else if(!undetected_msg_published) {
        feedback.detected = false;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp    = ros::Time::now();

        feedback.enemy_pos.pose.position.x = 0;
        feedback.enemy_pos.pose.position.y = 0;
        feedback.enemy_pos.pose.position.z = 0;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = true;
      }
    }
    rate.sleep();
  }

}
void ArmorDetectionNode::ExecuteLoop() {
  undetected_count_ = undetected_armor_delay_;

  while(running_) {
    if (node_state_ == NodeState::RUNNING) {
        if (mv_capture_->isindustryimgInput()) {
            img_ = mv_capture_->image();
        }
        bool img_empty = img_.empty();
        if (!img_empty) {
            serial_.updateReceiveInformation();
            if (basic_armor_.runBasicArmor(img_, serial_.returnReceive())) {
                pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
                serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
                fornt_camera_robot_pos_.id .push_back(num_.detectionNum(basic_armor_.returnArmorNumImg(0, img_), basic_armor_.returnFinalArmorDistinguish(0)));
            }
            detected_enemy_ = basic_armor_.returnSuccessArmor();
            std::cout << "是否探测到"<<detected_enemy_ << std::endl;
            // std::lock_guard<std::mutex> guard(mutex_);
            cv::imshow("SJUT", img_);
            if(cv::waitKey(1) == 'q') {
                break;
            }
        }
        else if (img_empty){
          detected_enemy_ = false;
        }
        basic_armor_.freeMemory();
        mv_capture_->cameraReleasebuff();
    }
    if(detected_enemy_) {
      // 控制云台
      pitch_ = pnp_.returnPitchAngle();
      yaw_ =  pnp_.returnYawAngle() ;
    //目标敌方的相对位置
      x_ =  pnp_.returnDepth() * cos(yaw_ / 180 * 3.1415926);
      y_  = pnp_.returnDepth() * sin(yaw_/ 180 * 3.1415926);
    //TODO
      // num_armor_ = basic_armor_.returnArmorNum(); 
      num_armor_ = 1;
      pose_A_[0]  = x_;
      pose_A_[1] = y_;
      fornt_camera_robot_pos_.num_armor = num_armor_ ;  
      for(int i = 0 ; i< num_armor_ ; ++i)
      {
        fornt_camera_robot_pos_.state.push_back(1);
        if(i == 0){
          fornt_camera_robot_pos_.pose_A.push_back(pose_A_[0]);
          fornt_camera_robot_pos_.pose_A.push_back(pose_A_[1]);
        }
        if(i == 1){
            fornt_camera_robot_pos_.pose_B.push_back(pose_B_[0]);
            fornt_camera_robot_pos_.pose_B.push_back(pose_B_[1]);
        }
      }
      PublishInfoMsgs();
    }       
    else if (!detected_enemy_){
      fornt_camera_robot_pos_.num_armor = 0 ; 
      gimbal_angle_.yaw_mode = true;
      gimbal_angle_.pitch_mode = false;
      gimbal_angle_.yaw_angle = 0;
      gimbal_angle_.pitch_angle = 0;
      undetected_count_--;
      PublishGimbalMsgs();
      PublishInfoMsgs();
    }
    else if (node_state_ == NodeState::PAUSE) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_var_.wait(lock);
    }
    }
  }


void ArmorDetectionNode::StartThread() {
  ROS_INFO("Armor detection node started!");
  running_ = true;
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }
  std::cout << "start armor detection thread" << std::endl;
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
//   armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

void ArmorDetectionNode::PublishGimbalMsgs() {
  enemy_info_pub_.publish(gimbal_angle_);
}

void ArmorDetectionNode::PublishInfoMsgs() {
  camera_armor_pub_.publish(fornt_camera_robot_pos_);
  Clear();
}
void ArmorDetectionNode::Clear(){
  fornt_camera_robot_pos_.num_armor = 0;
    fornt_camera_robot_pos_.id.clear();
    fornt_camera_robot_pos_.state.clear();
    fornt_camera_robot_pos_.pose_A.clear();
    fornt_camera_robot_pos_.pose_B.clear();

}

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM,SignalHandler);
    ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);
    ArmorDetectionNode armor_detection;
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    ros::waitForShutdown();
    armor_detection.StopThread();
    return 0;
}