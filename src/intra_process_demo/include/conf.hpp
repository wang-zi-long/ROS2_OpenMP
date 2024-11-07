#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/syscall.h>
#include "stdio.h"
#include "string.h"

#define LOCAL_SCREEN_PRINT  0

#define DEFAULT         0
#define OPENMP1         1
#define OPENMP2         2
#define EXECUTE_TYPE    OPENMP1

#define OPENMP_THREAD_NUM      4
#define EXECUTOR_THREAD_NUM    4

// 开启Timer错过的打印输出
#define TIMER_MISSED    0
#define CHAIN_WORKFLOW  0.3

// OMP的段数
#define OMP_NUM     6 // 1,2,3,4,5,6,7,8