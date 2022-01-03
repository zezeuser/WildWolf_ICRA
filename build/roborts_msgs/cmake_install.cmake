# Install script for directory: /home/zeze/catkin_ws/src/roborts_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/action" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/roborts_msgs/action/LocalPlanner.action"
    "/home/zeze/catkin_ws/src/roborts_msgs/action/GlobalPlanner.action"
    "/home/zeze/catkin_ws/src/roborts_msgs/action/ArmorDetection.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerAction.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerActionGoal.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerActionResult.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerActionFeedback.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerGoal.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerResult.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/LocalPlannerFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerAction.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerActionGoal.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerActionResult.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerActionFeedback.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerGoal.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerResult.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/GlobalPlannerFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionAction.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionActionGoal.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionActionResult.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionActionFeedback.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionGoal.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionResult.msg"
    "/home/zeze/catkin_ws/src/build/devel/share/roborts_msgs/msg/ArmorDetectionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg/referee_system" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/GameStatus.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/GameResult.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/GameRobotHP.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/GameRobotBullet.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/GameZone.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/GameZoneArray.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/RobotStatus.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/RobotHeat.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/RobotDamage.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/referee_system/RobotShoot.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/Aimtargeid.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/RobotInfo.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/TwistAccel.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/GimbalAngle.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/GimbalRate.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/SwingMode.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/ArmorPos.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/ArmorsPos.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/AllyPose.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/GimbalControl.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/Target.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/DodgeMode.msg"
    "/home/zeze/catkin_ws/src/roborts_msgs/msg/GimbalInfo.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/srv" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/roborts_msgs/srv/SwingDefend.srv"
    "/home/zeze/catkin_ws/src/roborts_msgs/srv/GimbalMode.srv"
    "/home/zeze/catkin_ws/src/roborts_msgs/srv/FricWhl.srv"
    "/home/zeze/catkin_ws/src/roborts_msgs/srv/ShootCmd.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/cmake" TYPE FILE FILES "/home/zeze/catkin_ws/src/build/roborts_msgs/catkin_generated/installspace/roborts_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zeze/catkin_ws/src/build/devel/include/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zeze/catkin_ws/src/build/devel/share/roseus/ros/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zeze/catkin_ws/src/build/devel/share/common-lisp/ros/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zeze/catkin_ws/src/build/devel/share/gennodejs/ros/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/zeze/catkin_ws/src/build/devel/lib/python2.7/dist-packages/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zeze/catkin_ws/src/build/devel/lib/python2.7/dist-packages/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zeze/catkin_ws/src/build/roborts_msgs/catkin_generated/installspace/roborts_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/cmake" TYPE FILE FILES "/home/zeze/catkin_ws/src/build/roborts_msgs/catkin_generated/installspace/roborts_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/cmake" TYPE FILE FILES
    "/home/zeze/catkin_ws/src/build/roborts_msgs/catkin_generated/installspace/roborts_msgsConfig.cmake"
    "/home/zeze/catkin_ws/src/build/roborts_msgs/catkin_generated/installspace/roborts_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs" TYPE FILE FILES "/home/zeze/catkin_ws/src/roborts_msgs/package.xml")
endif()

