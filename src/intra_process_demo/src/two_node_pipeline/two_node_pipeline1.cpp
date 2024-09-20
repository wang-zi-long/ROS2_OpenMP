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

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 1000, 230ms, true);
  executor.add_node(Timer1);
  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 2, 200, false);
  executor.add_node(Sub1);

  auto Timer2 = std::make_shared<Sensor>("Timer2", "CHAIN2_TIMER_OUT", 2, 1, 200, 250ms, false);
  executor.add_node(Timer2);
  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN2_TIMER_OUT", 2, 2, 1000, true);
  executor.add_node(Sub2);

  // auto Transfer3_1 = std::make_shared<Transfer>("Transfer3_1", "CHAIN2_TIMER_OUT", "CHAIN2_Transfer_OUT", 3, 2, 22, false);
  // executor.add_node(Transfer3_1);
  // auto Transfer3_2 = std::make_shared<Transfer>("Transfer3_2", "CHAIN2_Transfer_OUT", "CHAIN2_SUB_OUT", 3, 3, 184, true);
  // executor.add_node(Transfer3_2);
  // auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN2_SUB_OUT", 3, 4, 91, true);
  // executor.add_node(Sub3);

  // auto Timer4 = std::make_shared<Sensor>("Timer4", "CHAIN4_TIMER_OUT", 4, 1, 231, 100ms, true);
  // executor.add_node(Timer4);
  // auto Transfer4_1 = std::make_shared<Transfer>("Transfer4_1", "CHAIN4_TIMER_OUT", "CHAIN4_Transfer_OUT", 4, 2, 79, false);
  // executor.add_node(Transfer4_1);
  // auto Transfer4_2 = std::make_shared<Transfer>("Transfer4_2", "CHAIN4_Transfer_OUT", "CHAIN4_SUB_OUT", 4, 3, 142, false);
  // executor.add_node(Transfer4_2);
  // auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN4_SUB_OUT", 4, 4, 179, true);
  // executor.add_node(Sub4);

  // auto Timer5 = std::make_shared<Sensor>("Timer5", "CHAIN5_TIMER_OUT", 5, 1, 206, 100ms, true);
  // executor.add_node(Timer5);
  // auto Transfer5 = std::make_shared<Transfer>("Transfer5", "CHAIN5_TIMER_OUT", "CHAIN5_SUB_OUT", 5, 2, 179, true);
  // executor.add_node(Transfer5);
  // auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN5_SUB_OUT", 5, 3, 66, false);
  // executor.add_node(Sub5);

  // auto Timer6 = std::make_shared<Sensor>("Timer6", "CHAIN6_TIMER_OUT", 6, 1, 170, 160ms, true);
  // executor.add_node(Timer6);
  // auto Transfer6_1 = std::make_shared<Transfer>("Transfer6_1", "CHAIN6_TIMER_OUT", "CHAIN6_Transfer_OUT", 6, 2, 110, true);
  // executor.add_node(Transfer6_1);
  // auto Transfer6_2 = std::make_shared<Transfer>("Transfer6_2", "CHAIN6_Transfer_OUT", "CHAIN6_SUB_OUT", 6, 3, 66, false);
  // executor.add_node(Transfer6_2);
  // auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN6_SUB_OUT", 6, 4, 79, false);
  // executor.add_node(Sub6);

#if(LOCAL_SCREEN_PRINT == 1)
#if(EXECUTE_TYPE == DEFAULT)
  FILE *fp = freopen("/home/neu/Desktop/OpenMP/results/DEFAULT.log", "w", stdout);
#elif(EXECUTE_TYPE == OPENMP)
  FILE *fp = freopen("/home/neu/Desktop/OpenMP/results/OPENMP.log", "w", stdout);
#endif
#endif

  executor.spin();

  rclcpp::shutdown();

#if (LOCAL_SCREEN_PRINT == 1)
  fclose(fp);
#endif

  return 0;
}
