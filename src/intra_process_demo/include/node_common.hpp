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
#include "/usr/local/lib/gcc/x86_64-pc-linux-gnu/15.0.0/include/omp.h"
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

void dummy_load_100us(int load_100us) {
  // volatile uint64_t i, j;
  // for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
  //   for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
  //       __asm__ volatile ("nop");
  auto now = std::chrono::steady_clock::now();
  auto end_time = now + std::chrono::microseconds(load_100us * 100);
  while(std::chrono::steady_clock::now() < end_time){
  }
}

void dummy_load_100us_omp(int load_100us, int a, int b) {
  // for(int k = 0;k < OMP_NUM; ++k){
    #pragma omp parallel for num_threads(OPENMP_THREAD_NUM)
    for (uint64_t j = 0; j < (dummy_load_calib * (load_100us) / 10); j++){
        // if(j == 0){
        //   long int tid = syscall(SYS_gettid);
        //   int cpu_core = sched_getcpu();
        //   printf("|Tid:%ld|-->|%d|%d|On:%d|111\n", tid, a, b, cpu_core);
        // }
        for (uint64_t i = 0; i < DUMMY_LOAD_ITER; i++) {
            __asm__ volatile ("nop");
        }
        // if(j == (dummy_load_calib * (load_100us) / 10) - 1){
        //   long int tid = syscall(SYS_gettid);
        //   int cpu_core = sched_getcpu();
        //   printf("|Tid:%ld|-->|%d|%d|On:%d|222\n", tid, a, b, cpu_core);
        // }
    }
  // }
}

void dummy_load_100us_new(int load_100us, bool use_openmp, int chain_idx, int node_idx, uint32_t data, rclcpp::executors::MultiThreadedExecutor &executor) {
  if(use_openmp){
    int strategy = get_strategy();
    for(int k = 0;k < OMP_NUM; ++k){
      #pragma omp parallel num_threads(OPENMP_THREAD_NUM)
      {
        // long int tid = syscall(SYS_gettid);
        // printf("|Tid:%ld|-->|%d|%d|\n", tid, chain_idx, node_idx);

        auto now = std::chrono::steady_clock::now();
        auto end_time = now + std::chrono::microseconds(load_100us / OPENMP_THREAD_NUM * 100);
        while(std::chrono::steady_clock::now() < end_time){
        }
      }
      if(strategy == 2){
        // long int tid = syscall(SYS_gettid);
        // uint64_t cur = get_clocktime();
        // printf("|Tid:%ld|11111-->|%ld|\n",tid, cur);

        if(get_wait_for_work_flag() == true){
          // cur = get_clocktime();
          // printf("|Tid:%ld|Wake-->|%ld|\n",tid, cur);
          executor.trigger_interrupt_guard();
        }
        int pri = get_priority();
        omp_Node* temp = dequeue2(pri);
        if(temp == NULL && chain_idx != 1){
          usleep(10);
        }
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
  Sensor(const std::string & node_name, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, std::chrono::duration<int,std::milli> period_ms, bool use_openmp, rclcpp::executors::MultiThreadedExecutor &executor)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    auto callback = [this, captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp, &executor]() -> void {
        long int tid = syscall(SYS_gettid);
        // if(priority != 0){
        //   // 修改执行器线程以及OpenMP线程的优先级
        //   if(use_openmp){
        //     #pragma omp parallel num_threads(OPENMP_THREAD_NUM)
        //     {
        //       std::string command = "sudo chrt -f -p " + std::to_string(priority) + " " + std::to_string(tid);
        //       system(command.c_str());
        //     }
        //   }else{
        //     std::string command = "sudo chrt -f -p " + std::to_string(priority) + " " + std::to_string(tid);
        //     system(command.c_str());
        //   }
        // }
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        // std::thread::id thread_id = std::this_thread::get_id();
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = cnter_val[chain_idx-1]++;

        volatile uint64_t ts_start = get_clocktime();
        dummy_load_100us(exe_time_100us);
        // dummy_load_100us_new(exe_time_100us, true, chain_idx, node_idx, msg->data, executor);
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

        // if(priority != 0){
        //   // 恢复执行器线程的优先级
        //   std::string command = "sudo chrt -f -p 50 " + std::to_string(tid);
        //   system(command.c_str());
        // }

      };
    timer_ = this->create_wall_timer(period_ms, callback);
  }
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// MARK: Transfer Node structure definition.
struct Transfer : public rclcpp::Node {
  Transfer(const std::string & node_name, const std::string & input_topic, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp,
   rclcpp::executors::MultiThreadedExecutor &executor)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [this, captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp, &executor](std_msgs::msg::Int32::UniquePtr msg) {
          
          long int tid = syscall(SYS_gettid);

          // if(priority != 0){
          //   // 修改执行器线程以及OpenMP线程的优先级
          //   if(use_openmp){
          //     #pragma omp parallel num_threads(OPENMP_THREAD_NUM)
          //     {
          //       std::string command = "sudo chrt -f -p " + std::to_string(priority) + " " + std::to_string(tid);
          //       system(command.c_str());
          //     }
          //   }else{
          //     std::string command = "sudo chrt -f -p " + std::to_string(priority) + " " + std::to_string(tid);
          //     system(command.c_str());
          //   }
          // }

          volatile uint64_t ts_start = get_clocktime();
          auto pub_ptr = captured_pub.lock();
          if (!pub_ptr) {
            return;
          }

          dummy_load_100us_new(exe_time_100us, use_openmp, chain_idx, node_idx, msg->data, executor);
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

          // if(priority != 0){
          //   std::string command = "sudo chrt -f -p 50 " + std::to_string(tid);
          //   system(command.c_str());
          // }

      }
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

/// MARK: Command Node structure definition.
struct Command : public rclcpp::Node {
  Command(const std::string & node_name, const std::string & input_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp, rclcpp::executors::MultiThreadedExecutor &executor)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [this, node_idx, chain_idx, exe_time_100us, use_openmp,  &executor](std_msgs::msg::Int32::UniquePtr msg) {

          long int tid = syscall(SYS_gettid);

          // dummy_load_100us(10);
          volatile uint64_t ts_start = get_clocktime();
          // dummy_load_100us_omp(exe_time_100us, chain_idx, node_idx);
          dummy_load_100us_new(exe_time_100us, use_openmp, chain_idx, node_idx, msg->data, executor);
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

          // if(priority != 0){
          //   std::string command = "sudo chrt -f -p 50 " + std::to_string(tid);
          //   system(command.c_str());
          // }
      }
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

#endif