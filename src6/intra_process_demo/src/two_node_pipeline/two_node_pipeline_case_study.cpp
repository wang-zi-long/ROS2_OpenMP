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
  //是否启用OpenMP，0表示不启用，1表示启用openmp1，表示启用openmp2
  int is_openmp = std::stoi(argv[2]);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, (is_openmp == 2 && count > 0) ? (17 / OPENMP_THREAD_NUM) : 17, 1000ms, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Timer1);
  if(count > 0){
    count -- ;
  }
  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 2, (is_openmp == 2 && count > 0)  ? (1959 / OPENMP_THREAD_NUM) : 1959, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Sub1);
  if(count > 0){
    count -- ;
  }

  auto Timer2 = std::make_shared<Sensor>("Timer2", "CHAIN2_TIMER_OUT", 2, 1, (is_openmp == 2 && count > 0)  ? (23 / OPENMP_THREAD_NUM) : 23, 80ms, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Timer2);
  if(count > 0){
    count -- ;
  }
  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN2_TIMER_OUT", 2, 2, (is_openmp == 2 && count > 0)  ? (161 / OPENMP_THREAD_NUM) : 161,(is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Sub2);
   if(count > 0){
    count -- ;
  }
  auto Transfer3_1 = std::make_shared<Transfer>("Transfer3_1", "CHAIN2_TIMER_OUT", "CHAIN2_Transfer_OUT", 3, 2, (is_openmp == 2 && count > 0)  ? (22 / OPENMP_THREAD_NUM) : 22, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer3_1);
   if(count > 0){
    count -- ;
  }
  auto Transfer3_2 = std::make_shared<Transfer>("Transfer3_2", "CHAIN2_Transfer_OUT", "CHAIN2_SUB_OUT", 3, 3, (is_openmp == 2 && count > 0)  ? (184 / OPENMP_THREAD_NUM) : 184,(is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer3_2);
  if(count > 0){
    count -- ;
  }
  auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN2_SUB_OUT", 3, 4, (is_openmp == 2 && count > 0)  ? (91 / OPENMP_THREAD_NUM) : 91, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Sub3);
  if(count > 0){
    count -- ;
  }
  auto Timer4 = std::make_shared<Sensor>("Timer4", "CHAIN4_TIMER_OUT", 4, 1, (is_openmp == 2 && count > 0)  ? (231 / OPENMP_THREAD_NUM) : 231, 100ms, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Timer4);
  if(count > 0){
    count -- ;
  }
  auto Transfer4_1 = std::make_shared<Transfer>("Transfer4_1", "CHAIN4_TIMER_OUT", "CHAIN4_Transfer_OUT", 4, 2, (is_openmp == 2 && count > 0)  ? (79 / OPENMP_THREAD_NUM) : 79,  (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer4_1);
  if(count > 0){
    count -- ;
  }
  auto Transfer4_2 = std::make_shared<Transfer>("Transfer4_2", "CHAIN4_Transfer_OUT", "CHAIN4_SUB_OUT", 4, 3, (is_openmp == 2 && count > 0)  ? (142 / OPENMP_THREAD_NUM) : 142, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer4_2);
  if(count > 0){
    count -- ;
  }
  auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN4_SUB_OUT", 4, 4, (is_openmp == 2 && count > 0)  ? (179 / OPENMP_THREAD_NUM) : 179,  (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Sub4);
if(count > 0){
    count -- ;
  }
  auto Timer5 = std::make_shared<Sensor>("Timer5", "CHAIN5_TIMER_OUT", 5, 1, (is_openmp == 2 && count > 0)  ? (206 / OPENMP_THREAD_NUM) : 206, 100ms, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Timer5);
  if(count > 0){
    count -- ;
  }
  auto Transfer5 = std::make_shared<Transfer>("Transfer5", "CHAIN5_TIMER_OUT", "CHAIN5_SUB_OUT", 5, 2, (is_openmp == 2 && count > 0)  ? (179 / OPENMP_THREAD_NUM) : 179,  (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer5);
  if(count > 0){
    count -- ;
 }
  auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN5_SUB_OUT", 5, 3, (is_openmp == 2 && count > 0)  ? (66 / OPENMP_THREAD_NUM) : 66, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Sub5);
  if(count > 0){
    count -- ;
  }

  auto Timer6 = std::make_shared<Sensor>("Timer6", "CHAIN6_TIMER_OUT", 6, 1, (is_openmp == 2 && count > 0)  ? (170 / OPENMP_THREAD_NUM) : 170, 160ms, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Timer6);
  if(count > 0){
    count -- ;
  }
  auto Transfer6_1 = std::make_shared<Transfer>("Transfer6_1", "CHAIN6_TIMER_OUT", "CHAIN6_Transfer_OUT", 6, 2, (is_openmp == 2 && count > 0)  ? (110 / OPENMP_THREAD_NUM) : 110,(is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer6_1);
  if(count > 0){
    count -- ;
  }
  auto Transfer6_2 = std::make_shared<Transfer>("Transfer6_2", "CHAIN6_Transfer_OUT", "CHAIN6_SUB_OUT", 6, 3, (is_openmp == 2 && count > 0)  ? (66 / OPENMP_THREAD_NUM) : 66, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Transfer6_2);
  if(count > 0){
    count -- ;
  }
  auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN6_SUB_OUT", 6, 4, (is_openmp == 2 && count > 0)  ? (79 / OPENMP_THREAD_NUM) : 79, (is_openmp != 0 && count > 0) ? true : false, executor);
  executor.add_node(Sub6);
  if(count > 0){
    count -- ;
  }

  if(is_openmp == 0){
    set_strategy(0);
  }else if(is_openmp == 1){
    set_strategy(1);
  }else if(is_openmp == 2){
    set_executor_num(EXECUTOR_THREAD_NUM);
    set_strategy(2);
    omp_queue_init();
  }

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
