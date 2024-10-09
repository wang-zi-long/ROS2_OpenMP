/* Copyright (C) 2005-2024 Free Software Foundation, Inc.
   Contributed by Richard Henderson <rth@redhat.com>.

   This file is part of the GNU Offloading and Multi Processing Library
   (libgomp).

   Libgomp is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.

   Libgomp is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
   FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   Under Section 7 of GPL version 3, you are granted additional
   permissions described in the GCC Runtime Library Exception, version
   3.1, as published by the Free Software Foundation.

   You should have received a copy of the GNU General Public License and
   a copy of the GCC Runtime Library Exception along with this program;
   see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
   <http://www.gnu.org/licenses/>.  */

/* This file handles the (bare) PARALLEL construct.  */

#include "libgomp.h"
#include <limits.h>
#include "stdio.h"
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/syscall.h>
#include <pthread.h>
#include <stdatomic.h>

/* Determine the number of threads to be launched for a PARALLEL construct.
   This algorithm is explicitly described in OpenMP 3.0 section 2.4.1.
   SPECIFIED is a combination of the NUM_THREADS clause and the IF clause.
   If the IF clause is false, SPECIFIED is forced to 1.  When NUM_THREADS
   is not present, SPECIFIED is 0.  */

unsigned
gomp_resolve_num_threads (unsigned specified, unsigned count)
{
  struct gomp_thread *thr = gomp_thread ();
  struct gomp_task_icv *icv;
  unsigned threads_requested, max_num_threads, num_threads;
  unsigned long busy;
  struct gomp_thread_pool *pool;

  icv = gomp_icv (false);

  if (specified == 1)
    return 1;

  if (thr->ts.active_level >= 1
  /* Accelerators with fixed thread counts require this to return 1 for
     nested parallel regions.  */
#if !defined(__AMDGCN__) && !defined(__nvptx__)
      && icv->max_active_levels_var <= 1
#endif
      )
    return 1;
  else if (thr->ts.active_level >= icv->max_active_levels_var)
    return 1;

  /* If NUM_THREADS not specified, use nthreads_var.  */
  if (specified == 0)
    threads_requested = icv->nthreads_var;
  else
    threads_requested = specified;

  max_num_threads = threads_requested;

  /* If dynamic threads are enabled, bound the number of threads
     that we launch.  */
  if (icv->dyn_var)
    {
      unsigned dyn = gomp_dynamic_max_threads ();
      if (dyn < max_num_threads)
	max_num_threads = dyn;

      /* Optimization for parallel sections.  */
      if (count && count < max_num_threads)
	max_num_threads = count;
    }

  /* UINT_MAX stands for infinity.  */
  if (__builtin_expect (icv->thread_limit_var == UINT_MAX, 1)
      || max_num_threads == 1)
    return max_num_threads;

  /* The threads_busy counter lives in thread_pool, if there
     isn't a thread_pool yet, there must be just one thread
     in the contention group.  If thr->team is NULL, this isn't
     nested parallel, so there is just one thread in the
     contention group as well, no need to handle it atomically.  */
  pool = thr->thread_pool;
  if (thr->ts.team == NULL || pool == NULL)
    {
      num_threads = max_num_threads;
      if (num_threads > icv->thread_limit_var)
	num_threads = icv->thread_limit_var;
      if (pool)
	pool->threads_busy = num_threads;
      return num_threads;
    }

#ifdef HAVE_SYNC_BUILTINS
  do
    {
      busy = pool->threads_busy;
      num_threads = max_num_threads;
      if (icv->thread_limit_var - busy + 1 < num_threads)
	num_threads = icv->thread_limit_var - busy + 1;
    }
  while (__sync_val_compare_and_swap (&pool->threads_busy,
				      busy, busy + num_threads - 1)
	 != busy);
#else
  gomp_mutex_lock (&gomp_managed_threads_lock);
  num_threads = max_num_threads;
  busy = pool->threads_busy;
  if (icv->thread_limit_var - busy + 1 < num_threads)
    num_threads = icv->thread_limit_var - busy + 1;
  pool->threads_busy += num_threads - 1;
  gomp_mutex_unlock (&gomp_managed_threads_lock);
#endif

  return num_threads;
}

void
GOMP_parallel_start (void (*fn) (void *), void *data, unsigned num_threads)
{
  num_threads = gomp_resolve_num_threads (num_threads, 0);
  gomp_team_start (fn, data, num_threads, 0, gomp_new_team (num_threads),
		   NULL);
}

void
GOMP_parallel_end (void)
{
  struct gomp_task_icv *icv = gomp_icv (false);
  if (__builtin_expect (icv->thread_limit_var != UINT_MAX, 0))
    {
      struct gomp_thread *thr = gomp_thread ();
      struct gomp_team *team = thr->ts.team;
      unsigned int nthreads = team ? team->nthreads : 1;
      gomp_team_end ();
      if (nthreads > 1)
      {
        /* If not nested, there is just one thread in the
          contention group left, no need for atomicity.  */
        if (thr->ts.team == NULL){
          thr->thread_pool->threads_busy = 1;
        }
        else
          {
    #ifdef HAVE_SYNC_BUILTINS
            __sync_fetch_and_add (&thr->thread_pool->threads_busy,
                1UL - nthreads);
    #else
            gomp_mutex_lock (&gomp_managed_threads_lock);
            thr->thread_pool->threads_busy -= nthreads - 1;
            gomp_mutex_unlock (&gomp_managed_threads_lock);
    #endif
          }
	}
    }
  else{
    gomp_team_end ();
  }
}
ialias (GOMP_parallel_end)

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

int getcpu_via_syscall(void) {
    unsigned cpu, node;
    if (syscall(SYS_getcpu, &cpu, &node, NULL) == -1) {
        perror("syscall SYS_getcpu failed");
        return -1; // 返回 -1 表示失败
    }
    return (int)cpu;
}

bool help_flag;
bool helpflag(){
  return help_flag;
}

int strategy = -1;
void set_strategy(int temp){
  strategy = temp;
}
int get_strategy(){
  return strategy;
}

int executor_num = -1;
void set_executor_num(int temp){
  executor_num = temp;
}

void
GOMP_parallel (void (*fn) (void *), void *data, unsigned num_threads,
	       unsigned int flags)
{
if(strategy == -1){
  printf("Not set OMP_strategy!!!\n");
}else if(strategy == 1){
  num_threads = gomp_resolve_num_threads (num_threads, 0);
  gomp_team_start (fn, data, num_threads, flags, gomp_new_team (num_threads),
		   NULL);
  fn (data);
  ialias_call (GOMP_parallel_end) ();
}else if(strategy == 2){
  if(executor_num == -1){
    printf("Not set executor_num!!!\n");
  }
  // long int Tid = syscall(SYS_gettid);
  // // int cpu_core = getcpu_via_syscall();
  // uint64_t cur_time = get_clocktime();
  // printf("|TID:%ld|-->[GOMP_parallel:%lu]\n", Tid, cur_time);

  // @Mark：修改代码逻辑：在入队之前，首先将help_flag置成true
  // omp从线程在wait_for_work之前，即使未出队成功，但是如果看到help_flag为true，omp从线程会循环查询omp队列
  help_flag = true;
  // 主线程将任务加入队列中，唤醒因队空而堵塞的从线程
  omp_Node *temp = enqueue(fn, data);
  help_flag = false;

  // cur_time = get_clocktime();
  // printf("|TID:%ld|-->[After enqueue:%lu]\n", Tid, cur_time);

  // 主线程执行任务，@Mark：此处存在问题，整体任务并没有按照线程数平分成若干个任务，后续需要修改
  fn(data);

  // omp_Node *temp = omp_queue->head->next;
  // if(temp != NULL){
  //   cur_time = get_clocktime();
  //   printf("|TID:%ld|-->[Before cowait:%lu]-->[temp_ptr:%p]\n", Tid, cur_time, (void*)temp);
  //   pthread_cond_wait(&temp->cond, &temp->lock);
  // }
  pthread_mutex_lock(&temp->lock);
  // 添加逻辑，omp主线程查看任务节点是否从omp队列中（被omp从线程）取出
  // @Mark：当执行器线程的个数大于2时，后续的代码逻辑需要进行修改
  if(temp->count == 1){
    // 如果任务节点未被取出，说明另一个执行器未进入omp从线程的状态，因此该任务节点仍由omp主线程执行
    // cur_time = get_clocktime();
    // printf("|TID:%ld|-->[Before    deq:%lu]\n", Tid, cur_time);
    // @Mark：当任务节点加上优先级后，此处的代码逻辑后续需要修改
    // @Note：omp主线程执行到此处时，另一个执行器线程可能也处于omp主线程状态，因此，此时不能直接执行dequeue()函数出队
    temp->count--;
    if(temp->count == 0){
      // 此处需要加锁，因为此时修改的是omp队列，即将任务节点从omp队列中取出
      pthread_mutex_lock(&omp_queue->lock);
      temp->pre->next = temp->next;
      if(temp->next != NULL){
        temp->next->pre = temp->pre;
      }
      pthread_mutex_unlock(&omp_queue->lock);
      temp->pre = temp->next = NULL;
    }else{
      // @mark：当执行器线程个数大于2，此处的代码逻辑需要进一步思考
    }
    pthread_mutex_unlock(&temp->lock);
    temp->fn(temp->data);
    dequeue1(temp);
  }else if(temp->count != 1 && !temp->reclaim_flag){
    // @Note：omp主线程有可能比omp从线程更晚执行结束
    // 经过测试，有可能发生这种情况，主线程和从线程的执行结束时间仅差一点，即主线程略晚于从线程执行结束，此时主线程不需要任何操作（从线程将任务节点回收）
    // cur_time = get_clocktime();
    // printf("|TID:%ld|-->[Before cowait:%lu]-->[temp_ptr:%p]\n", Tid, cur_time, (void*)temp);
    // 用来标记omp主线程进入堵塞状态，任务节点的回收实际上还是omp线程来干
    // 经过测试，发现这么一个现象：在主线程检查reclaim_flag之后、堵塞之前，从线程将reclaim_flag置成true，因此在此多一层判断
    temp->reclaim_flag = true;
    pthread_mutex_unlock(&temp->lock);
    // 如果任务节点已经被omp从线程取出，omp主线程等待omp从线程执行完成
    pthread_cond_wait(&temp->cond, &temp->lock);
  }
  // cur_time = get_clocktime();
  // printf("|TID:%ld|-->[Ret  parallel:%lu]\n", Tid, cur_time);
}
}

Queue* omp_queue;
// 通过锁从node_pool里互斥申请和回收任务节点
pthread_mutex_t pool_lock;
omp_Node* node_pool;
#define node_pool_num 5

void omp_queue_init(){
  // @Mark：后续更改成静态分配空间
  omp_queue = (Queue *)malloc(sizeof(Queue));
  omp_queue->head = (omp_Node*)malloc(sizeof(omp_Node));
  omp_queue->head->pre = NULL;
  omp_queue->head->next = NULL;
  pthread_mutex_init(&omp_queue->lock, NULL);
  pthread_cond_init(&omp_queue->cond, NULL); 
  node_pool = (omp_Node*)malloc(sizeof(omp_Node));
  node_pool->pre = NULL;
  node_pool->next = NULL;
  for(int i = 0;i < node_pool_num;++i){
    omp_Node* temp = (omp_Node*)malloc(sizeof(omp_Node));
    temp->reclaim_flag = true;
    if(i == 0){
      node_pool->next = temp;
      node_pool->pre = temp;
      temp->pre = node_pool;
      temp->next = node_pool;
    }else{
      temp->next = node_pool->next;
      temp->pre = node_pool;
      node_pool->next = temp;
      temp->next->pre = temp;
    }
    pthread_mutex_init(&temp->lock, NULL);
    pthread_cond_init(&temp->cond, NULL);
  }
  help_flag = false;
}
ialias (omp_queue_init)

omp_Node* enqueue(void (*fn) (void *), void *data) {
  // 每次从node_pool的末尾申请节点，将回收的节点加入node_pool的首部，以保证上一次和本次入队时申请的节点不同
  // @Note：此处可能存在问题，后续需要检查和修改
  pthread_mutex_lock(&pool_lock);
  omp_Node* temp;
  if(node_pool->pre == node_pool->next && node_pool->pre == NULL){
    // 此时node_pool为空，输出一下，暂时不做处理
    printf("Node_pool is empty!!!\n");
    return NULL;
  }else if(node_pool->pre == node_pool->next && node_pool->pre != NULL){
    // 此时node_pool里面只剩下一个任务节点
    temp = node_pool->pre;
    node_pool->pre = node_pool->next = NULL;
  }else{
    temp = node_pool->pre;
    node_pool->pre = temp->pre;
    temp->pre->next = node_pool;
  }
  temp->pre = temp->next = NULL;
  temp->fn = fn;
  temp->data = data;
  temp->reclaim_flag = false;
  // @Mark：omp_get_num_threads()的值持续等于1，预留API接口
  // @Note：count的初始值为1，意味着omp从线程从omp队列中取出任务节点即可执行
  // @Mark：任务节点的初始值与执行器线程的个数相关，当执行器线程的个数大于2时，后续代码逻辑需要进一步思考
  temp->count = executor_num - 1;
  pthread_mutex_unlock(&pool_lock);

  pthread_mutex_lock(&omp_queue->lock);
  if(omp_queue->head->pre == omp_queue->head->next && omp_queue->head->pre == NULL){
    omp_queue->head->pre = temp;
    omp_queue->head->next = temp;
    temp->pre = omp_queue->head;
    temp->next = omp_queue->head;
  }else{
    omp_Node* pre = omp_queue->head;
    while(pre->next != NULL && pre->next->count <= temp->count){
      pre = pre->next;
    }
    temp->next = omp_queue->head->next;
    temp->pre = omp_queue->head;
    temp->next->pre = temp;
    omp_queue->head->next = temp;
  }
  // pthread_cond_signal(&omp_queue->cond);
  pthread_mutex_unlock(&omp_queue->lock);
  return temp;
}
ialias (enqueue)

// 此函数实际上用来检查omp队列是否为空，如果不为空，返回omp队列中的第一个任务节点
omp_Node* dequeue() {
  // long int Tid = syscall(SYS_gettid);
  // int cpu_core = getcpu_via_syscall();
  // uint64_t current_time;
  while (omp_queue->head->pre == omp_queue->head->next && omp_queue->head->next == NULL) {
      return NULL;
      // pthread_cond_wait(&omp_queue->cond, &omp_queue->lock);
      // current_time = get_clocktime();
      // printf("|TID:%ld|-->[CPU Core On:%d]-->[dequeue waiting222:%lu]\n", Tid, cpu_core, current_time);
  }
  pthread_mutex_lock(&omp_queue->lock);
  // 后进先出策略
  // @Note：后续任务节点添加优先级后，按照优先级出队的逻辑在此处修改添加
  omp_Node *temp = omp_queue->head->next;
  // omp从线程才会执行此函数，而且只要omp从线程执行此函数，意味着omp从线程即将执行任务节点的内容
  
  // 需要判断，只有当任务节点执行次数为0时，才能将其从omp队列中移除
  // @Note：只是将任务节点从omp队列中移除，并没有回收任务节点
  if(temp->count == 0){
    // @mark：当执行器线程个数等于2时，omp从线程执行此函数，不可能执行下一分支（任务节点的初始值是1）
    temp->pre->next = temp->next;
    if(temp->next != NULL){
      temp->next->pre = temp->pre;
    }
    temp->pre = temp->next = NULL;
  }else{
    // @mark：当执行器线程个数大于2，此处的代码逻辑需要进一步思考
  }
  pthread_mutex_unlock(&omp_queue->lock);
  temp->count--;
  return temp;
}
ialias (dequeue)

// dequeue函数与dequeue1函数无法合并，因为只有当任务节点的内容执行完毕后，才能回收任务节点
void dequeue1(omp_Node *temp){
  // long int Tid = syscall(SYS_gettid);
  // 访问任务节点时上锁的主要目的：使得omp主线程在GOMP_parallel()函数里访问任务节点和omp从线程执行此函数时互斥
  pthread_mutex_lock(&temp->lock);
  // pthread_mutex_lock(&omp_queue->lock);
  // @Note：如果执行器线程的个数大于2时，此处的逻辑可能不正确，需要修改temp->count ！= 0分支的代码逻辑
  if(temp->count == 0){
    // 从线程执行此函数，主要目的是回收任务节点，不可能执行下一分支
    // pthread_mutex_unlock(&omp_queue->lock);
    // 此时任务节点的内容执行完毕，需要唤醒因任务节点同步而等待的omp主线程
    if(temp->reclaim_flag == true){
      // 说明omp主线程进入堵塞状态，因此需要唤醒omp主线程
      pthread_cond_signal(&temp->cond);
      // uint64_t cur_time = get_clocktime();
      // printf("|TID:%ld|-->[After  signal:%lu]-->[temp_ptr:%p]\n", Tid, cur_time, (void*)temp);
    }else{
      // 说明omp主线程未进入堵塞状态，说明从线程比主线程先执行完毕，只需要回收任务节点即可
      temp->reclaim_flag = true;
    }
    pthread_mutex_unlock(&temp->lock);
    pthread_mutex_lock(&pool_lock);
    // 回收任务节点，@Mark：node_pool的访问和修改可能需要加锁
    temp->pre = node_pool;
    temp->next = node_pool->next;
    if(node_pool->next != NULL){
      node_pool->next->pre = temp;
    }
    node_pool->next = temp;
    pthread_mutex_unlock(&pool_lock);
    // printf("|TID:%ld|-->[return dequeue1]\n", Tid);
  }else{

  }
}
ialias (dequeue1)

unsigned
GOMP_parallel_reductions (void (*fn) (void *), void *data,
			  unsigned num_threads, unsigned int flags)
{
  struct gomp_taskgroup *taskgroup;
  num_threads = gomp_resolve_num_threads (num_threads, 0);
  uintptr_t *rdata = *(uintptr_t **)data;
  taskgroup = gomp_parallel_reduction_register (rdata, num_threads);
  gomp_team_start (fn, data, num_threads, flags, gomp_new_team (num_threads),
		   taskgroup);
  fn (data);
  ialias_call (GOMP_parallel_end) ();
  gomp_sem_destroy (&taskgroup->taskgroup_sem);
  free (taskgroup);
  return num_threads;
}

bool
GOMP_cancellation_point (int which)
{
  if (!gomp_cancel_var)
    return false;

  struct gomp_thread *thr = gomp_thread ();
  struct gomp_team *team = thr->ts.team;
  if (which & (GOMP_CANCEL_LOOP | GOMP_CANCEL_SECTIONS))
    {
      if (team == NULL)
	return false;
      return team->work_share_cancelled != 0;
    }
  else if (which & GOMP_CANCEL_TASKGROUP)
    {
      if (thr->task->taskgroup)
	{
	  if (thr->task->taskgroup->cancelled)
	    return true;
	  if (thr->task->taskgroup->workshare
	      && thr->task->taskgroup->prev
	      && thr->task->taskgroup->prev->cancelled)
	    return true;
	}
      /* FALLTHRU into the GOMP_CANCEL_PARALLEL case,
	 as #pragma omp cancel parallel also cancels all explicit
	 tasks.  */
    }
  if (team)
    return gomp_team_barrier_cancelled (&team->barrier);
  return false;
}
ialias (GOMP_cancellation_point)

bool
GOMP_cancel (int which, bool do_cancel)
{
  if (!gomp_cancel_var)
    return false;

  if (!do_cancel)
    return ialias_call (GOMP_cancellation_point) (which);

  struct gomp_thread *thr = gomp_thread ();
  struct gomp_team *team = thr->ts.team;
  if (which & (GOMP_CANCEL_LOOP | GOMP_CANCEL_SECTIONS))
    {
      /* In orphaned worksharing region, all we want to cancel
	 is current thread.  */
      if (team != NULL)
	team->work_share_cancelled = 1;
      return true;
    }
  else if (which & GOMP_CANCEL_TASKGROUP)
    {
      if (thr->task->taskgroup)
	{
	  struct gomp_taskgroup *taskgroup = thr->task->taskgroup;
	  if (taskgroup->workshare && taskgroup->prev)
	    taskgroup = taskgroup->prev;
	  if (!taskgroup->cancelled)
	    {
	      gomp_mutex_lock (&team->task_lock);
	      taskgroup->cancelled = true;
	      gomp_mutex_unlock (&team->task_lock);
	    }
	}
      return true;
    }
  team->team_cancelled = 1;
  gomp_team_barrier_cancel (team);
  return true;
}

/* The public OpenMP API for thread and team related inquiries.  */

int
omp_get_num_threads (void)
{
  struct gomp_team *team = gomp_thread ()->ts.team;
  return team ? team->nthreads : 1;
}

int
omp_get_thread_num (void)
{
  return gomp_thread ()->ts.team_id;
}

/* This wasn't right for OpenMP 2.5.  Active region used to be non-zero
   when the IF clause doesn't evaluate to false, starting with OpenMP 3.0
   it is non-zero with more than one thread in the team.  */

int
omp_in_parallel (void)
{
  return gomp_thread ()->ts.active_level > 0;
}

int
omp_get_level (void)
{
  return gomp_thread ()->ts.level;
}

int
omp_get_ancestor_thread_num (int level)
{
  struct gomp_team_state *ts = &gomp_thread ()->ts;
  if (level < 0 || level > ts->level)
    return -1;
  for (level = ts->level - level; level > 0; --level)
    ts = &ts->team->prev_ts;
  return ts->team_id;
}

int
omp_get_team_size (int level)
{
  struct gomp_team_state *ts = &gomp_thread ()->ts;
  if (level < 0 || level > ts->level)
    return -1;
  for (level = ts->level - level; level > 0; --level)
    ts = &ts->team->prev_ts;
  if (ts->team == NULL)
    return 1;
  else
    return ts->team->nthreads;
}

int
omp_get_active_level (void)
{
  return gomp_thread ()->ts.active_level;
}

ialias (omp_get_num_threads)
ialias (omp_get_thread_num)
ialias (omp_in_parallel)
ialias (omp_get_level)
ialias (omp_get_ancestor_thread_num)
ialias (omp_get_team_size)
ialias (omp_get_active_level)
