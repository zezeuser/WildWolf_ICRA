cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(zmq_test CXX)

# decision_node
add_executable(zmq_test 
 ZMQ_server.cpp
  )

target_link_libraries(zmq_test    -lzmq)


