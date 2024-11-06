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
  // FILE *fp = freopen("/home/neu/Desktop/Openmp/results/10.23/1_to_N/OMP_DEFAULT/1_to_4_FIFO.log", "w", stdout);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  dummy_load_calibration();

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 0, 1000ms, false, executor);
  executor.add_node(Timer1);
  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 1, 2000, true, executor);
  executor.add_node(Sub1);
  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN1_TIMER_OUT", 1, 2, 2000, true, executor);
  executor.add_node(Sub2);
  // auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN1_TIMER_OUT", 1, 3, 2000, true, executor);
  // executor.add_node(Sub3);
  // auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN1_TIMER_OUT", 1, 4, 2000, true, executor);
  // executor.add_node(Sub4);

  set_strategy(1);

  executor.spin();

  // fclose(fp);

  rclcpp::shutdown();

  return 0;
}
