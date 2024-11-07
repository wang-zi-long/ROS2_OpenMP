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
#include "/home/neu/Desktop/OMP/src/intra_process_demo/include/conf.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 2, 404, true, executor); // 1
  executor.add_node(Sub1);

  auto Transfer2_1 = std::make_shared<Transfer>("Transfer2_1", "CHAIN1_TIMER_OUT", "CHAIN2_Transfer_OUT", 2, 2, 29, false, executor);
  executor.add_node(Transfer2_1);

  auto Transfer2_2 = std::make_shared<Transfer>("Transfer2_2", "CHAIN2_Transfer_OUT", "CHAIN2_SUB_OUT", 2, 3, 328, true, executor); // 2
  executor.add_node(Transfer2_2);

  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN2_SUB_OUT", 2, 4, 32, false, executor);
  executor.add_node(Sub2);

  auto Transfer3_1 = std::make_shared<Transfer>("Transfer3_1", "CHAIN3_TIMER_OUT", "CHAIN3_Transfer_OUT", 3, 2, 26,false, executor);
  executor.add_node(Transfer3_1);

  auto Transfer3_2 = std::make_shared<Transfer>("Transfer3_2", "CHAIN3_Transfer_OUT", "CHAIN3_SUB_OUT", 3, 3, 24,false, executor);
  executor.add_node(Transfer3_2);

  auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN3_SUB_OUT", 3, 4, 248, true, executor); // 4
  executor.add_node(Sub3);

  auto Transfer4_1 = std::make_shared<Transfer>("Transfer4_1", "CHAIN4_TIMER_OUT", "CHAIN4_SUB_OUT", 4, 2, 24,false, executor);
  executor.add_node(Transfer4_1);

  auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN4_SUB_OUT", 4, 3, 31,false, executor);
  executor.add_node(Sub4);

  auto Transfer5 = std::make_shared<Transfer>("Transfer5", "CHAIN5_TIMER_OUT", "CHAIN5_SUB_OUT", 5, 2, 304,false, executor); // 6
  executor.add_node(Transfer5);

  auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN5_SUB_OUT", 5, 3, 66,false, executor);
  executor.add_node(Sub5);

  auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN6_TIMER_OUT", 6, 2, 37,false, executor);
  executor.add_node(Sub6);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 30, 80ms, false, executor);
  executor.add_node(Timer1);

  auto Timer3 = std::make_shared<Sensor>("Timer3", "CHAIN3_TIMER_OUT", 3, 1, 284, 100ms, true, executor); // 3
  executor.add_node(Timer3);

  auto Timer4 = std::make_shared<Sensor>("Timer4", "CHAIN4_TIMER_OUT", 4, 1, 364, 100ms, false, executor); // 5
  executor.add_node(Timer4);

  auto Timer5 = std::make_shared<Sensor>("Timer5", "CHAIN5_TIMER_OUT", 5, 1, 17, 160ms, false, executor);
  executor.add_node(Timer5);

  auto Timer6 = std::make_shared<Sensor>("Timer6", "CHAIN6_TIMER_OUT", 6, 1, 21, 200ms, false, executor);
  executor.add_node(Timer6);

  // set_executor_num(EXECUTOR_THREAD_NUM);
  // set_strategy(2);
  // omp_queue_init();
  set_strategy(1);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
