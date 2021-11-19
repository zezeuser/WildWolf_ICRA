project(roborts_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    nav_msgs
    geometry_msgs
    actionlib
    message_generation
    actionlib_msgs
    )

add_action_files(
    DIRECTORY action
    FILES
    LocalPlanner.action
    GlobalPlanner.action
    ArmorDetection.action
 
)

add_message_files(
        DIRECTORY msg/referee_system
        FILES
        GameStatus.msg
        GameResult.msg
        GameRobotHP.msg
        GameRobotBullet.msg
        GameZone.msg
        GameZoneArray.msg
        RobotStatus.msg
        RobotHeat.msg
        RobotDamage.msg
        RobotShoot.msg
)

add_message_files(
    DIRECTORY msg
    FILES
    PunishInfo.msg
    RobotInfo.msg
    TreeStatus.msg
    TwistAccel.msg
    GimbalAngle.msg
    GimbalRate.msg
    ObstacleMsg.msg
    ShootInfo.msg
    ShootState.msg
    ShooterCmd.msg
    SentryInfo.msg
    ArmorPos.msg
    ArmorsPos.msg
    AllyPose.msg
    Distance.msg
    GimbalPID.msg
    GimbalControl.msg
    TargetInfo.msg
    FVector.msg
    FusionTarget.msg
    LaserTarget.msg
    Target.msg
    DodgeMode.msg
    PyArmorInfo.msg
    GimbalInfo.msg
    BallCollision.msg
    GimbalActionlib.msg
    Speech.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    SwingDefend.srv
    GimbalMode.srv
    FricWhl.srv
    ShootCmd.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs
)

catkin_package()
