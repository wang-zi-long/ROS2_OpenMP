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

#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "rcpputils/scope_exit.hpp"
#include "rclcpp/utilities.hpp"
#include "/home/neu/Desktop/OMP/src/intra_process_demo/include/conf.hpp"
#include "/usr/local/gcc/lib/gcc/aarch64-unknown-linux-gnu/15.0.0/include/omp.h"

using rclcpp::executors::MultiThreadedExecutor;

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: rclcpp::Executor(options),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout)
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard wait_lock{wait_mutex_};
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  run(thread_id);
  for (auto & thread : threads) {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

void openmp_init(size_t this_thread_number){
  #pragma omp parallel num_threads(OPENMP_THREAD_NUM)
	{
		pthread_t current_thread = pthread_self();
  
		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(4, &cpuset);
    CPU_SET(5, &cpuset);
    CPU_SET(6, &cpuset);
    CPU_SET(7, &cpuset);
    // CPU_SET(8, &cpuset);
    // CPU_SET(9, &cpuset);
    // CPU_SET(10, &cpuset);
    // CPU_SET(11, &cpuset);
		int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
		if (result != 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}
		CPU_ZERO(&cpuset);
		if (pthread_getaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
			perror("pthread_getaffinity_np");
			exit(EXIT_FAILURE);
		}
		long int tid = syscall(SYS_gettid);
		std::string command = "sudo chrt -f -p 50 " + std::to_string(tid);
		system(command.c_str());

    printf("|%ld|%ld|\n", this_thread_number, tid);
	}
}

std::atomic<uint64_t> priority(0);
void
MultiThreadedExecutor::run(size_t this_thread_number)
{
  (void)this_thread_number;
  
  int strategy = get_strategy();
  
  if(strategy == 1){
    // GOMP
    openmp_init(this_thread_number);
  }else if(strategy == 2){
    // ROSOMP
    pthread_t current_thread = pthread_self();
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    // @Noted：暂时为每个执行器线程绑定单独的CPU核心
    CPU_SET(4, &cpuset);
    CPU_SET(5, &cpuset);
    CPU_SET(6, &cpuset);
    CPU_SET(7, &cpuset);
    // CPU_SET(8, &cpuset);
    // CPU_SET(9, &cpuset);
    // CPU_SET(10, &cpuset);
    // CPU_SET(11, &cpuset);
    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
      perror("pthread_setaffinity_np");
      exit(EXIT_FAILURE);
    }
    CPU_ZERO(&cpuset);
    if (pthread_getaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
      perror("pthread_getaffinity_np");
      exit(EXIT_FAILURE);
    }
    long int tid = syscall(SYS_gettid);
    std::string command = "sudo chrt -f -p 50 " + std::to_string(tid);
    system(command.c_str());
    set_tid_map(tid, this_thread_number);
    printf("|%ld|%ld|\n", this_thread_number, tid);
  }

  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      // std::lock_guard wait_lock{wait_mutex_};
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    if(strategy == 2){
      omp_Node *temp = dequeue();
      while(temp != NULL){
        temp->fn(temp->data);
        dequeue1(temp);
        temp = dequeue();
      }
      long int value = priority.load();
      ++ priority;
      set_priority(this_thread_number, value);
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}
