#include  "armor_detection_node.hpp"

auto down_ =  std::chrono::system_clock::now();
ArmorDetectionNode::ArmorDetectionNode():
node_state_(roborts_common::IDLE),
initialized_(false),
detected_enemy_(false),
undetected_armor_delay_(100),
target_id_(1),
shooting_(false),
fricwhl_open_(false),
as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false),
basic_armor_("/home/wildwolf-nuc02/catkin_ws/src/armor_detection/configs/armor/basic_armor_config.xml"),
pnp_("/home/wildwolf-nuc02/catkin_ws/src/armor_detection/configs/camera/mv_camera_config_337.xml",
                                         "/home/wildwolf-nuc02/catkin_ws/src/armor_detection/configs/angle_solve/basic_pnp_config.xml"),
num_(2)
{            
    mv_capture_ = new mindvision::VideoCapture(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));
    data_.my_color = uart::ALL;
    data_.my_robot_id = uart::SUP_SHOOT;
    data_.now_run_mode = uart::INFANTRY;
    data_.bullet_velocity = 15;
    data_.Receive_Pitch_Angle_Info.pitch_angle = 0.f;
    data_.Receive_Yaw_Angle_Info.yaw_angle = 0.f;
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    enemy_nh_ = ros::NodeHandle();
    enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    camera_armor_pub_ = enemy_nh_.advertise<roborts_msgs::ArmorsPos>("front_camera_robot_pos", 30);
    target_id_sub_ = enemy_nh_.subscribe<roborts_msgs::Aimtargeid>("emeny_target_id",30,&ArmorDetectionNode::TargeIdCallBack,this);
    gimbal_info_sub_ = enemy_nh_.subscribe<roborts_msgs::GimbalInfo>("gimbal_info",100,&ArmorDetectionNode::UpdateGimbalDataCallBack,this);
    fricwhl_client_ = nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    shoot_client_ = nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
    initialized_ = true;

    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity();
    _Kalman::Matrix_zxd H;
    H(0, 0) = 1;
    _Kalman::Matrix_xxd R;
    R(0, 0) = 10;
    for (int i = 1; i < S; i++) {
        R(i, i) = 100;
    }
    _Kalman::Matrix_zzd Q{20};
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);
    streamer.start(8080);
    as_.start();
    ROS_INFO("armor_detection server success start ! !");
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
    case 5:
      ShootNone();
      break;
    case 6:
      StopFricWhl();
      break;
    case 7:
      ShootContinuous();
      break;
    case 8:
      ShootOnce();
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
  cv::Point3f target_pnp = cv::Point3f();
  std::vector<uchar>::iterator id_iterator;
  float x_last = 0;
  float y_last = 0;
  auto start = std::chrono::system_clock::now();
  double c_speed = 0.0, last_speed = 0.0;
  float compensate_w           = 0;
  float last_compensate_w      = 0;
  float last_last_compensate_w = 0;
  while (running_)
  {
          if (node_state_ == NodeState::RUNNING)
          {
            if (mv_capture_->isindustryimgInput()){
                    img_ = mv_capture_->image();
            }
                if (!img_.empty()){
                    double m_yaw = data_.Receive_Yaw_Angle_Info.yaw_angle;
                    // cv::putText(img_, std::to_string(m_yaw), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 255, 0));
                    auto end = std::chrono::system_clock::now();
                    static double last_yaw = 0, last_speed = 0.0;
                    // cv::putText(img_, std::to_string(std::fabs(last_yaw - m_yaw)), cv::Point(50, 200), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 255, 0));
                    if (std::fabs(last_yaw - m_yaw) > (25 / 180. * M_PI)) {
                        kalman.reset(m_yaw, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 0.001);
                        last_yaw = m_yaw;
                        std::cout << "reset" << std::endl;
                    } else {
                        last_yaw = m_yaw;
                        Eigen::Matrix<double, 1, 1> z_k{m_yaw};
                        _Kalman::Matrix_x1d         state = kalman.update(z_k, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 0.001);
                        c_speed                           = state(1, 0) * 1.75;
                        c_speed                           = (c_speed + last_speed) * 0.5;
                        // cv::putText(img_, std::to_string(c_speed), cv::Point(50, 150), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 255, 0));

                        last_speed                        = c_speed;
                    }
                    if (basic_armor_.runBasicArmor(img_, data_)){
                        for (int i = 0; i < basic_armor_.returnArmorNum(); i++) {
                            cv::Mat num_img = basic_armor_.returnArmorNumImg(i, img_);
                            int armor_id = num_.detectionNum(num_img, basic_armor_.returnFinalArmorDistinguish(i));
                            basic_armor_.setArmorId(armor_id, i);
                            pnp_.solvePnP(data_.bullet_velocity, basic_armor_.returnFinalArmorDistinguish(i), basic_armor_.returnFinalArmorRotatedRect(i));
                            ArrangeMsg(armor_id , pnp_.returnYawAngle(),pnp_.returnDepth());
                            basic_armor_.setArmorPnp(cv::Point3f(pnp_.returnYawAngle(),
                                                                  pnp_.returnPitchAngle(),
                                                                  pnp_.returnDepth()),
                                                                  i);
                            // std::cout<< pnp_.returnYawAngle() << '      ' << pnp_.returnPitchAngle() << std::endl;
                          }
                        //  查看目标 id 是否在视野内 ， 不在视野内则选择最优装甲板
                        id_iterator = find(front_camera_robot_pos.id.begin(), front_camera_robot_pos.id.end(), target_id_);
                        if(id_iterator == front_camera_robot_pos.id.end()){
                                target_pnp = basic_armor_.returnArmorPnp(0);
                        }else{
                            target_pnp = basic_armor_.returnArmorPnp(std::distance(front_camera_robot_pos.id.begin(),id_iterator));
                        }
                        double predict_time    = (target_pnp.z * 0.001 / data_.bullet_velocity);
                        double s_yaw           = atan2(predict_time * c_speed * target_pnp.z * 0.001, 1);
                        compensate_w           = 8 * tan(s_yaw);
                        compensate_w           = (last_last_compensate_w + last_compensate_w + compensate_w) * 0.333;
                        last_last_compensate_w = last_compensate_w;
                        last_compensate_w      = compensate_w;
                        // cv::putText(img_, std::to_string(compensate_w), cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 255, 0));
                        static cv::Point2f ss = cv::Point2f(0, 0);
                        ss                    = cv::Point2f(-compensate_w, 0);
                        std::vector<cv::Point2f> traget_2d = basic_armor_.returnFinalArmor4Point(0);
                        traget_2d[0] += ss;
                        traget_2d[1] += ss;
                        traget_2d[2] += ss;
                        traget_2d[3] += ss;
                        for (size_t l = 0; l != 4; ++l) {
                            cv::line(img_, traget_2d[l], traget_2d[(l + 1) % 4], cv::Scalar(0, 255, 255), 3, 8);
                        }
                     //   pnp_.solvePnP(data_.bullet_velocity, 1, traget_2d);
                      //  target_pnp.x = pnp_.returnYawAngle();
                        front_camera_robot_pos.num_armor = basic_armor_.returnArmorNum();
                    }
                    detected_enemy_ = basic_armor_.returnSuccessArmor();
                    // cv::imshow("SJUT", img_);
                    // if (cv::waitKey(1) == 'q'){
                    //         break;
                    // }
                }
                else{
                    detected_enemy_ = false;
                }
                std::vector<uchar> buff_img_;
                cv::imencode(".jpg", img_, buff_img_, params);
                streamer.publish("/bgr", std::string(buff_img_.begin(), buff_img_.end()));
                basic_armor_.freeMemory();
                mv_capture_->cameraReleasebuff();
          }
        //   if (fabs(target_pnp.x) < 0.2) {
        //       target_pnp.x = 0;
        //   }
        //   if (fabs(target_pnp.y) < 0.2) {
        //       target_pnp.y = 0;
        //   }
        //   target_pnp.x += x_last;
        //   x_last *= 0.5;
          target_pnp.y += y_last;
          y_last *= 0.5;
          if (detected_enemy_){
              gimbal_angle_.yaw_mode = true;
              gimbal_angle_.pitch_mode = true;
              gimbal_angle_.yaw_angle = -target_pnp.x/180*CV_PI;
              gimbal_angle_.pitch_angle = target_pnp.y/180*CV_PI;
              x_last = gimbal_angle_.yaw_angle;
              enemy_info_pub_.publish(gimbal_angle_);
              camera_armor_pub_.publish(front_camera_robot_pos);
              undetected_count_ = 0;
              Clear();
          }else{
              undetected_count_ ++;
              if(undetected_count_ >= undetected_armor_delay_){
                // 哨兵模式不需要复位云台
                // gimbal_angle_.yaw_mode = false;
                // gimbal_angle_.pitch_mode = false;
                // gimbal_angle_.yaw_angle = 0;
                // gimbal_angle_.pitch_angle = 0.08;
                // enemy_info_pub_.publish(gimbal_angle_);
                camera_armor_pub_.publish(front_camera_robot_pos);
              }
          }
          if (node_state_ == NodeState::PAUSE){
              std::unique_lock<std::mutex> lock(mutex_);
              condition_var_.wait(lock);
          }
        }
        streamer.stop();
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

void ArmorDetectionNode::StartFricWhl(){
    // if(detected_enemy_){
    fricwhl_.request.open = true;
    fricwhl_client_.call(fricwhl_);
        // fricwhl_open_ = fricwhl_.response.received;
    // }
    fricwhl_open_ = true;
}

void ArmorDetectionNode::StopFricWhl(){
    fricwhl_.request.open = false;
    fricwhl_client_.call(fricwhl_);
    // fricwhl_open_ = fricwhl_.response.received;
    fricwhl_open_ = false;
}

void ArmorDetectionNode::ShootOnce(){
    if(!fricwhl_open_){
        StartFricWhl();
    }
    if(fricwhl_open_ ){
        shootcmd_.request.mode = shootcmd_.request.ONCE ;  // once  mode
        shootcmd_.request.number = 1 ;
        shoot_client_.call(shootcmd_);
        shooting_ = shootcmd_.response.received;
    }
}

void ArmorDetectionNode::ShootContinuous(){
    if(!fricwhl_open_){
        StartFricWhl();
    }
    shootcmd_.request.mode = shootcmd_.request.CONTINUOUS ;   // contunyius mode 
    shootcmd_.request.number = 5 ;
    shoot_client_.call(shootcmd_);
    shooting_ = shootcmd_.response.received;
}

void ArmorDetectionNode::ShootNone(){
    if(!fricwhl_open_){
        StartFricWhl();
    }
    shootcmd_.request.mode = shootcmd_.request.STOP ;  
    shoot_client_.call(shootcmd_);
    shooting_ = false;
}

void ArmorDetectionNode::ArrangeMsg(int _armor_id , float _yaw_angle , float _depth ) {
        front_camera_robot_pos.id.push_back(_armor_id);
        front_camera_robot_pos.state.push_back(1);
        // x 
        front_camera_robot_pos.pose_A.push_back(_depth * cos(_yaw_angle / 57.2957805));
        // y
        front_camera_robot_pos.pose_B.push_back(_depth* sin(_yaw_angle / 57.2957805));
}

void ArmorDetectionNode::Clear(){
    front_camera_robot_pos.num_armor = 0;
    front_camera_robot_pos.id.clear();
    front_camera_robot_pos.state.clear();
    front_camera_robot_pos.pose_A.clear();
    front_camera_robot_pos.pose_B.clear();
}

void ArmorDetectionNode::TargeIdCallBack(const roborts_msgs::Aimtargeid::ConstPtr& targe_id){
    target_id_ =  targe_id->id;
}

void ArmorDetectionNode::UpdateGimbalDataCallBack(const roborts_msgs::GimbalInfo::ConstPtr& data){
    auto start  = std::chrono::system_clock::now();
    // std::cout << "time : " << std::chrono::duration_cast<std::chrono::milliseconds>(down_ - start).count() * 0.001 << std::endl;
   //  std::cout << data_.Receive_Yaw_Angle_Info.yaw_angle - (data->gyro_yaw/10 +180) << std::endl;
    data_.Receive_Yaw_Angle_Info.yaw_angle = data->gyro_yaw/10 + 180 ;
    data_.Receive_Pitch_Angle_Info.pitch_angle = data->gyro_pitch/10;
    down_  =  std::chrono::system_clock::now();
}


void ArmorDetectionNode::UpdateBulletVelocityCallBcak(const roborts_msgs::RobotShoot::ConstPtr& velocity){
    data_.bullet_velocity = (int)velocity->speed+0.5;  //四舍五入
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
