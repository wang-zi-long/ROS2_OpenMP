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
#include <iostream>
#include <vector>
#include <cstdlib> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "/home/neu/Desktop/OMP/src/intra_process_demo/include/conf.hpp"
#include "/usr/local/gcc/lib/gcc/aarch64-unknown-linux-gnu/15.0.0/include/omp.h"

#define DUMMY_LOAD_ITER 100
using namespace std::chrono_literals;
const int SIZE = 100;
uint64_t dummy_load_calib = 1;

uint64_t get_clocktime() { 
    long int        ns; 
    uint64_t        all; 
    time_t          sec; 
    struct timespec spec; 

    clock_gettime(CLOCK_REALTIME, &spec);

    sec   = spec.tv_sec; 
    ns    = spec.tv_nsec; 
    all   = (uint64_t) sec * 1000000000UL + (uint64_t) ns; 
    return all;  
}
std::vector<std::vector<int>> A(SIZE, std::vector<int>(SIZE));
std::vector<std::vector<int>> B(SIZE, std::vector<int>(SIZE));
std::vector<std::vector<int>> C(SIZE, std::vector<int>(SIZE)); 

void multiplyMatrices(const std::vector<std::vector<int>>& A, 
                      const std::vector<std::vector<int>>& B, 
                      std::vector<std::vector<int>>& C) {
    // 矩阵乘法
    for (int i = 0; i < SIZE; ++i) {
        for (int j = 0; j < SIZE; ++j) {
            for (int k = 0; k < SIZE; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}
void dummy_load_ms(int load_ms) {
  volatile uint64_t i, j;
  for (j = 0; j < dummy_load_calib * load_ms; j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

void dummy_load_100us(int load_100us) {
  auto now = std::chrono::steady_clock::now();
  auto end_time = now + std::chrono::microseconds(load_100us * 100);
  for (;std::chrono::steady_clock::now() < end_time;){
  }
}

void dummy_load_100us_omp(int load_100us, rclcpp::executors::MultiThreadedExecutor &executor) {
  for(int i = 0;i < OMP_NUM;++i){
    #pragma omp parallel num_threads(OPENMP_THREAD_NUM)
    {
      auto now = std::chrono::steady_clock::now();
      auto end_time = now + std::chrono::microseconds((int) ( (float) load_100us / OMP_NUM / OPENMP_THREAD_NUM * 100 ) );
      while(std::chrono::steady_clock::now() < end_time){
      }
    }
    if(get_strategy() == 2){
      if(get_wait_for_work_flag() == true){
        executor.trigger_interrupt_guard();
      }
      int pri = get_priority();
      omp_Node* temp = dequeue2(pri);
      // if(temp == NULL && chain_idx != 1){
      //   usleep(10);
      // }
      while (temp != NULL)
      {
        temp->fn(temp->data);
        dequeue3(temp, pri);
        if(get_wait_for_work_flag() == true){
          executor.trigger_interrupt_guard();
        }
        temp = dequeue2(pri);
      }
      pri_count_handle1(pri);
    }
  }
}

void dummy_load_calibration() {
  volatile uint64_t ts_start, ts_end;
  while(1) {
    ts_start = get_clocktime(); // in ns
    dummy_load_ms(100);         // 100ms
    ts_end = get_clocktime();   // in ns
    int duration_ns = ts_end - ts_start;
    if (abs(duration_ns - 100*1000*1000) < 1000000) {// error margin: 1ms
      break;
    }
    dummy_load_calib = 100*1000*1000*dummy_load_calib / duration_ns;
    if (dummy_load_calib <= 0) {
      dummy_load_calib = 1;
    }
  }
  ts_start = get_clocktime(); // in ns
  dummy_load_ms(10);          // 10ms
  ts_end = get_clocktime();   // in ns
  printf("|CALIBRATION TEST|[Setting: 10ms]->@time-measure: %lu. \r\n", ts_end-ts_start);
}
int cnter_val[5] = {0, 0, 0, 0, 0};

struct Sensor : public rclcpp::Node {
  Sensor(const std::string & node_name, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, std::chrono::duration<int,std::milli> period_ms, bool use_openmp, rclcpp::executors::MultiThreadedExecutor &executor)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    auto callback = [this, captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp, &executor]() -> void {
        long int tid = syscall(SYS_gettid);
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        // std::thread::id thread_id = std::this_thread::get_id();
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = cnter_val[chain_idx-1]++;

        volatile uint64_t ts_start = get_clocktime();
        dummy_load_100us(exe_time_100us);
        volatile uint64_t ts_end = get_clocktime();
        int cpu_core = sched_getcpu();

        printf("|TID:%ld|-->[Chain%u-Sensor%u.exe:%u(100us).on:%d]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Published msg: %d]. \r\n\n", 
                tid,
                chain_idx,
                node_idx,
                exe_time_100us,
                cpu_core,
                ts_start,
                ts_end,
                ts_end-ts_start,
                msg->data
              );

        pub_ptr->publish(std::move(msg));
      };
    timer_ = this->create_wall_timer(period_ms, callback);
  }
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

struct Command : public rclcpp::Node {
  Command(const std::string & node_name, const std::string & input_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp, rclcpp::executors::MultiThreadedExecutor &executor)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [this, node_idx, chain_idx, exe_time_100us, use_openmp,  &executor](std_msgs::msg::Int32::UniquePtr msg) {

          long int tid = syscall(SYS_gettid);

          volatile uint64_t ts_start = get_clocktime();
          if(use_openmp){
            dummy_load_100us_omp(exe_time_100us, executor);
          }else{
            dummy_load_100us(exe_time_100us);
          }
          
          volatile uint64_t ts_end = get_clocktime();
          int cpu_core = sched_getcpu();

          printf("|TID:%ld|-->[Chain%u-Command%u.exe:%u(100us).on:%d]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d]. \r\n\n", 
                  tid,
                  chain_idx,
                  node_idx,
                  exe_time_100us,
                  cpu_core,
                  ts_start,
                  ts_end,
                  ts_end-ts_start,
                  msg->data
                );
      }
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  // dummy_load_calibration();
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 1, 240, true, executor);
  executor.add_node(Sub1);
  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN1_TIMER_OUT", 1, 2, 240, true, executor);
  executor.add_node(Sub2);
  auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN1_TIMER_OUT", 1, 3, 240, true, executor);
  executor.add_node(Sub3);
  auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN1_TIMER_OUT", 1, 4, 240, true, executor);
  executor.add_node(Sub4);
  // auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN1_TIMER_OUT", 1, 5, 240, true, executor);
  // executor.add_node(Sub5);
  // auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN1_TIMER_OUT", 1, 6, 240, true, executor);
  // executor.add_node(Sub6);
  // auto Sub7 = std::make_shared<Command>("Sub7", "CHAIN1_TIMER_OUT", 1, 7, 240, true, executor);
  // executor.add_node(Sub7);
  // auto Sub8 = std::make_shared<Command>("Sub8", "CHAIN1_TIMER_OUT", 1, 8, 240, false, executor);
  // executor.add_node(Sub8);
  // auto Sub9 = std::make_shared<Command>("Sub9", "CHAIN1_TIMER_OUT", 1, 9, 240, false, executor);
  // executor.add_node(Sub9);
  // auto Sub10 = std::make_shared<Command>("Sub10", "CHAIN1_TIMER_OUT", 1, 10, 240, false, executor);
  // executor.add_node(Sub10);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 0, 100ms, false, executor);
  executor.add_node(Timer1);

  // set_executor_num(EXECUTOR_THREAD_NUM);
  // set_strategy(2);
  // omp_queue_init();
  set_strategy(1);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
