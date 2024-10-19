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
#include <cstring>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "node_common.hpp"
#include "conf.hpp"
#include <vector>
#include <thread>
#include "node_common.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  dummy_load_calibration();
  rclcpp::init(argc, argv);

  // 创建一个单线程的 Executor
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  auto Timer = std::make_shared<Sensor>("Timer", "CHAIN_TIMER_OUT", 1, 1, 5000, 1000ms, false, 10, executor);
  executor.add_node(Timer);

  set_strategy(0);

  // 进入 Executor 的 spin 循环
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

