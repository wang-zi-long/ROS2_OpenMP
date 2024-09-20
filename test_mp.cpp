#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "omp.h"
#include <sched.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <string>
#include <cstdlib>

void func(){
	int th_id, nthreads;

	printf("Before parallel region, total threads: %d\n", omp_get_num_threads());

	#pragma omp parallel private(th_id) num_threads(2)
	{
		pid_t tid = syscall(SYS_gettid);  // 获取系统线程号
		th_id = omp_get_thread_num();
		printf("Hello World from thread %d\t%d\t", th_id, tid);

		// printf("In parallel region, total threads: %d\n", omp_get_num_threads());

		#pragma omp barrier
		if (th_id == 1) {
			nthreads = omp_get_num_threads();
			printf("There are %d threads\n", nthreads);
		}	
	}
	
	printf("After parallel region, total threads: %d\n", omp_get_num_threads());
}

void func1(){
	#pragma omp parallel num_threads(2)
	{
		pthread_t current_thread = pthread_self();
  
		// 创建 CPU 集合并设置 CPU 0 和 1
		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);       // 清空 CPU 集合
		CPU_SET(0, &cpuset);     // 添加 CPU 0
		CPU_SET(1, &cpuset);     // 添加 CPU 1

		// 应用 CPU 亲和性设置
		int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
		if (result != 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}

		// 检查设置是否成功
		CPU_ZERO(&cpuset);
		if (pthread_getaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
			perror("pthread_getaffinity_np");
			exit(EXIT_FAILURE);
		}

		pid_t tid = syscall(SYS_gettid);

		printf("%d\n", tid);

		std::string command = "sudo chrt -f -p 50 " + std::to_string(tid);
		system(command.c_str());
	}
}

int main(void) 
{
    pid_t tid = syscall(SYS_gettid);  // 获取系统线程号
	printf("%d\n", tid);
	int i;
	int j = 0;

	func1();

	#pragma omp parallel for num_threads(2) private(j)
	for (i = 0; i <40; i++) {
		if(omp_get_thread_num() == 0)
		{
			int cpu = sched_getcpu();
			printf("Thread %d processing iteration %d | %d\n", omp_get_thread_num(), i, cpu);
			while (1)
			{
				
			}
			
		}
		sleep(1);
		int cpu = sched_getcpu();
		printf("Thread %d processing iteration %d | %d\n", omp_get_thread_num(), i, cpu);
	}

	return 0;
}

