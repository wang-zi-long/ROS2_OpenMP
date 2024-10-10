#ifndef DEMOS__NODE_COMMON_H
#define DEMOS__NODE_COMMON_H

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include "std_msgs/msg/string.hpp"
// #include "/usr/local/lib/gcc/x86_64-pc-linux-gnu/15.0.0/include/omp.h"
#include "conf.hpp"

#define DUMMY_LOAD_ITER	      100

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

void dummy_load_ms(int load_ms) {
  volatile uint64_t i, j;
  for (j = 0; j < dummy_load_calib * load_ms; j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

void dummy_load_100us(int load_100us, bool use_openmp) {
  volatile uint64_t i, j;
  if(use_openmp){
    #pragma omp parallel private(i) num_threads(OPENMP_THREAD_NUM)
    {
      volatile uint64_t start = get_clocktime();
      long int tid = syscall(SYS_gettid);

      #pragma omp for
      for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
        for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
            __asm__ volatile ("nop");

      volatile uint64_t end = get_clocktime();
      printf("|TID:%ld|-->[Start@%lu-End@%lu-Cost:%lu(ns)]\n", tid, start, end, end - start);
    }
  }else{
    for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
  }
}

void dummy_load_100us_new(int load_100us, bool use_openmp, int chain_idx, int node_idx, uint32_t data) {
  if(use_openmp){
    uint64_t i,j;
    #pragma omp parallel for num_threads(OPENMP_THREAD_NUM)
    for (j = 0; j < (dummy_load_calib * load_100us / 10); j++){
        for (i = 0; i < DUMMY_LOAD_ITER; i++) {
            __asm__ volatile ("nop");
        }
    }
  }else{
    uint64_t i,j;
    for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
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
  Sensor(const std::string & node_name, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, std::chrono::duration<int,std::milli> period_ms, bool use_openmp)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    auto callback = [captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp]() -> void {

        long int tid = syscall(SYS_gettid);

        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }

        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = cnter_val[chain_idx-1]++;

        volatile uint64_t ts_start = get_clocktime();
        dummy_load_100us_new(exe_time_100us, use_openmp, chain_idx, node_idx, msg->data);
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

/// MARK: Transfer Node structure definition.
struct Transfer : public rclcpp::Node {
  Transfer(const std::string & node_name, const std::string & input_topic, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp](std_msgs::msg::Int32::UniquePtr msg) {
          
          long int tid = syscall(SYS_gettid);

          volatile uint64_t ts_start = get_clocktime();
          auto pub_ptr = captured_pub.lock();
          if (!pub_ptr) {
            return;
          }

          dummy_load_100us_new(exe_time_100us, use_openmp, chain_idx, node_idx, msg->data);
          volatile uint64_t ts_end = get_clocktime();
          int cpu_core = sched_getcpu();

          printf("|TID:%ld|-->[Chain%u-Transfer%u.exe:%u(100us).on:%d]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Route msg: %d]. \r\n\n", 
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

      }
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

/// MARK: Command Node structure definition.
struct Command : public rclcpp::Node {
  Command(const std::string & node_name, const std::string & input_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [node_idx, chain_idx, exe_time_100us, use_openmp](std_msgs::msg::Int32::UniquePtr msg) {

          long int tid = syscall(SYS_gettid);

          volatile uint64_t ts_start = get_clocktime();
          // std::thread::id thread_id = std::this_thread::get_id();
          // dummy_load_100us(exe_time_100us, use_openmp);
          dummy_load_100us_new(exe_time_100us, use_openmp, chain_idx, node_idx, msg->data);
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

#endif