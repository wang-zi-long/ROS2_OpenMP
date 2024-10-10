// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "node_common.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  dummy_load_calibration();

  // 开启openmp的节点个数，从0开始，最大为18
  int count = std::stoi(argv[1]);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 17, 1000ms, (count > 0) ? true : false);
  executor.add_node(Timer1);
  if(count > 0){
    count -- ;
  }
  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 2, 1959, (count > 0) ? true : false);
  executor.add_node(Sub1);
  if(count > 0){
    count -- ;
  }

  auto Timer2 = std::make_shared<Sensor>("Timer2", "CHAIN2_TIMER_OUT", 2, 1, 23, 80ms, (count > 0) ? true : false);
  executor.add_node(Timer2);
  if(count > 0){
    count -- ;
  }
  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN2_TIMER_OUT", 2, 2, 161, (count > 0) ? true : false);
  executor.add_node(Sub2);
  if(count > 0){
    count -- ;
  }

  auto Transfer3_1 = std::make_shared<Transfer>("Transfer3_1", "CHAIN2_TIMER_OUT", "CHAIN2_Transfer_OUT", 3, 2, 22, (count > 0) ? true : false);
  executor.add_node(Transfer3_1);
  if(count > 0){
    count -- ;
  }
  auto Transfer3_2 = std::make_shared<Transfer>("Transfer3_2", "CHAIN2_Transfer_OUT", "CHAIN2_SUB_OUT", 3, 3, 184, (count > 0) ? true : false);
  executor.add_node(Transfer3_2);
  if(count > 0){
    count -- ;
  }
  auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN2_SUB_OUT", 3, 4, 91, (count > 0) ? true : false);
  executor.add_node(Sub3);
  if(count > 0){
    count -- ;
  }

  auto Timer4 = std::make_shared<Sensor>("Timer4", "CHAIN4_TIMER_OUT", 4, 1, 231, 100ms, (count > 0) ? true : false);
  executor.add_node(Timer4);
  if(count > 0){
    count -- ;
  }
  auto Transfer4_1 = std::make_shared<Transfer>("Transfer4_1", "CHAIN4_TIMER_OUT", "CHAIN4_Transfer_OUT", 4, 2, 79, (count > 0) ? true : false);
  executor.add_node(Transfer4_1);
  if(count > 0){
    count -- ;
  }
  auto Transfer4_2 = std::make_shared<Transfer>("Transfer4_2", "CHAIN4_Transfer_OUT", "CHAIN4_SUB_OUT", 4, 3, 142, (count > 0) ? true : false);
  executor.add_node(Transfer4_2);
  if(count > 0){
    count -- ;
  }
  auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN4_SUB_OUT", 4, 4, 179, (count > 0) ? true : false);
  executor.add_node(Sub4);
  if(count > 0){
    count -- ;
  }

  auto Timer5 = std::make_shared<Sensor>("Timer5", "CHAIN5_TIMER_OUT", 5, 1, 206, 100ms, (count > 0) ? true : false);
  executor.add_node(Timer5);
  if(count > 0){
    count -- ;
  }
  auto Transfer5 = std::make_shared<Transfer>("Transfer5", "CHAIN5_TIMER_OUT", "CHAIN5_SUB_OUT", 5, 2, 179, (count > 0) ? true : false);
  executor.add_node(Transfer5);
  if(count > 0){
    count -- ;
  }
  auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN5_SUB_OUT", 5, 3, 66, (count > 0) ? true : false);
  executor.add_node(Sub5);
  if(count > 0){
    count -- ;
  }

  auto Timer6 = std::make_shared<Sensor>("Timer6", "CHAIN6_TIMER_OUT", 6, 1, 170, 160ms, (count > 0) ? true : false);
  executor.add_node(Timer6);
  if(count > 0){
    count -- ;
  }
  auto Transfer6_1 = std::make_shared<Transfer>("Transfer6_1", "CHAIN6_TIMER_OUT", "CHAIN6_Transfer_OUT", 6, 2, 110, (count > 0) ? true : false);
  executor.add_node(Transfer6_1);
  if(count > 0){
    count -- ;
  }
  auto Transfer6_2 = std::make_shared<Transfer>("Transfer6_2", "CHAIN6_Transfer_OUT", "CHAIN6_SUB_OUT", 6, 3, 66, (count > 0) ? true : false);
  executor.add_node(Transfer6_2);
  if(count > 0){
    count -- ;
  }
  auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN6_SUB_OUT", 6, 4, 79, (count > 0) ? true : false);
  executor.add_node(Sub6);
  if(count > 0){
    count -- ;
  }

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
