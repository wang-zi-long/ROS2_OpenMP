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

using namespace std;

int main(int argc, char * argv[])
{
#if (LOCAL_SCREEN_PRINT == 1)
  FILE *fp = freopen("/home/neu/Desktop/templog/log.log", "w", stdout);
#endif

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  dummy_load_calibration();

  //传入随机参数，分别是任务链个数（2～7）、每条任务链的周期（50ms~500ms，负载强度固定为30%的情况下，不同的周期对应不同的总执行时间）
  int chain_num = stoi(argv[1]);
  int period    = stoi(argv[2]);
  //任务链执行周期的比例（0表示：1：1：1，1表示1：2：3，2表示3：2：1，3表示1：3：1）
  int exe_ratio = stoi(argv[3]);
  //是否启用OpenMP，0表示不启用，1表示启用openmp1，表示启用openmp2
  int is_openmp = stoi(argv[4]);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  vector<std::shared_ptr<Sensor>>   Timer;
  vector<std::shared_ptr<Transfer>> Trans;
  vector<std::shared_ptr<Command>>  Sub;
  for(int i = 1;i <= chain_num; ++i){
    std::chrono::duration<int, std::milli> period_ms(period + (i - 1) * 100);
    // std::chrono::duration<int, std::milli> period_ms(period);
    //通过任务链周期计算每条任务链的总执行时间，单位为ms
    int exe_time_sum = (period + (i - 1) * 100) * CHAIN_WORKFLOW;
    // int exe_time_sum = (period) * CHAIN_WORKFLOW;
    int exe1, exe2, exe3;
    switch (exe_ratio)
    {
    case 0/* 1:1:1 */:
      exe1 = exe_time_sum / 3;
      exe2 = exe_time_sum / 3;
      exe3 = exe_time_sum / 3;
      break;
    case 1/* 1:2:3 */:
      exe1 = exe_time_sum / 6 * 1;
      exe2 = exe_time_sum / 6 * 2;
      exe3 = exe_time_sum / 6 * 3;
      break;
    case 2/* 3:2:1 */:
      exe1 = exe_time_sum / 6 * 3;
      exe2 = exe_time_sum / 6 * 2;
      exe3 = exe_time_sum / 6 * 1;
      break;
    case 3/* 1:3:1 */:
      exe1 = exe_time_sum / 5 * 1;
      exe2 = exe_time_sum / 5 * 3;
      exe3 = exe_time_sum / 5 * 1;
      break;
    }

    printf("Chain %d : %dms | %d\t%d\t%d\n", i, ((period + (i - 1) * 100)), exe1, exe2, exe3);
    // printf("Chain %d : %dms | %d\t%d\t%d\n", i, (period), exe1, exe2, exe3);

    Timer.push_back(make_shared<Sensor>("Timer" + to_string(i), "CHAIN" + to_string(i) + "_TIMER_OUT", i, 1, is_openmp == 2 ? (exe1 * 10 / OPENMP_THREAD_NUM) : (exe1 * 10), period_ms, 
                    is_openmp == 0 ? false : true, is_openmp == 2 ? (1 + (10 - i) * 10) : 0));
    executor.add_node(Timer.back());
    Trans.push_back(make_shared<Transfer>("Transfer" + to_string(i), "CHAIN" + to_string(i) + "_TIMER_OUT", "CHAIN" + to_string(i) + "_SUB_OUT", i, 2, is_openmp == 2 ? (exe2 * 10 / OPENMP_THREAD_NUM) : (exe2 * 10), 
                    is_openmp == 0 ? false : true, is_openmp == 2 ? (2 + (10 - i) * 10) : 0));
    executor.add_node(Trans.back());
    Sub.push_back(make_shared<Command>("Sub" + to_string(i), "CHAIN" + to_string(i) + "_SUB_OUT", i, 3, is_openmp == 2 ? (exe3 * 10 / OPENMP_THREAD_NUM) : (exe3 * 10), 
                    is_openmp == 0 ? false : true, is_openmp == 2 ? (3 + (10 - i) * 10) : 0));
    executor.add_node(Sub.back());
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

#if (LOCAL_SCREEN_PRINT == 1)
  fclose(fp);
#endif

  rclcpp::shutdown();

  return 0;
}
