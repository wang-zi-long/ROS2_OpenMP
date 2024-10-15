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

using namespace std::chrono_literals;

void external_event_trigger(rclcpp::GuardCondition::SharedPtr guard_condition) {
  printf("hello world!!!\n");  // 模拟外部事件，延迟5秒触发
  RCLCPP_INFO(rclcpp::get_logger("GuardCondition Example"), "Triggering GuardCondition from external thread");
  guard_condition->trigger();  // 触发 GuardCondition
}

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    // 在构造函数中初始化 GuardCondition
    guard_condition_ = std::make_shared<rclcpp::GuardCondition>();

    // 创建一个线程模拟外部事件
    external_thread_ = std::thread(external_event_trigger, guard_condition_);
  }

  // 覆盖自定义的回调函数
  void handle_guard_condition() {
    RCLCPP_INFO(this->get_logger(), "Handling GuardCondition event");
  }

  rclcpp::GuardCondition::SharedPtr get_guard_condition() const {
    return guard_condition_;
  }

  ~MyNode() {
    if (external_thread_.joinable()) {
      external_thread_.join();
    }
  }

private:
  rclcpp::GuardCondition::SharedPtr guard_condition_;
  std::thread external_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNode>();

  // 创建一个单线程的 Executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // 注册节点
  executor.add_node(node);

  // 使用 lambda 函数来处理 GuardCondition 的触发，注意这里我们添加了 size_t 参数
  auto guard_callback = [&](size_t) {
    node->handle_guard_condition();
  };

    // 注册回调，当 GuardCondition 被触发时调用
  node->get_guard_condition()->set_on_trigger_callback(guard_callback);

  // 进入 Executor 的 spin 循环
  executor.spin();

  rclcpp::shutdown();
  return 0;
}


