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
ialias(helpflag)

int strategy = -1;
void set_strategy(int temp){
  strategy = temp;
}
ialias(set_strategy)
int get_strategy(){
  return strategy;
}
ialias(get_strategy)

int executor_num = -1;
void set_executor_num(int temp){
  executor_num = temp;
}
ialias(set_executor_num)

#define max_executor_num  10
int priority[max_executor_num];
void set_priority(int executor_id, uint64_t pri){
  priority[executor_id] = pri;
}
ialias(set_priority)

typedef struct Entry {
    long int key;
    int value;
    struct Entry* next;  // 用于处理哈希冲突的链表
} Entry;
Entry* Table;
void insert(long int key, int value) {
    Entry* newEntry = (Entry*)malloc(sizeof(Entry));
    newEntry->key = key;
    newEntry->value = value;
    newEntry->next = Table->next;
    Table->next = newEntry;
}
int find(long int key) {
    Entry* entry = Table->next;
    while (entry != NULL) {
        if (entry->key == key) {
            return entry->value;
        }
        entry = entry->next;
    }
    return -1;
}
void set_tid_map(long int tid, int executor_id){
  insert(tid, executor_id);
}
ialias(set_tid_map)

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
    long int Tid = syscall(SYS_gettid);
    int executor_id = find(Tid);
    if(executor_id == -1){
      printf("Find executor_id failed!!!\n");
    }
    int pri = priority[executor_id];

    help_flag = true;
    enqueue(fn, data, pri, executor_num);
    help_flag = false;

    omp_Node * temp = dequeue2(pri);
    while (temp != NULL)
    {
      temp->fn(temp->data);
      dequeue1(temp);
      temp = dequeue2(pri);
    }
  }
}

pthread_mutex_t queue_lock;
omp_Node *queue;
// 通过锁从node_pool里互斥申请和回收任务节点
#define node_pool_num 20
pthread_mutex_t pool_lock;
omp_Node* node_pool;

void omp_queue_init(){
  queue = (omp_Node*)malloc(sizeof(omp_Node));
  queue->pre = NULL;
  queue->next = NULL;
  pthread_mutex_init(&queue_lock, NULL);
  node_pool = (omp_Node*)malloc(sizeof(omp_Node));
  node_pool->pre = NULL;
  node_pool->next = NULL;
  for(int i = 0;i < node_pool_num;++i){
    omp_Node* temp = (omp_Node*)malloc(sizeof(omp_Node));
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
  Table = (Entry*)malloc(sizeof(Entry));
  Table->next = NULL;
}
ialias (omp_queue_init)

void enqueue(void (*fn) (void *), void *data, int priority, int executor_num) {
  for(int i = 0;i < executor_num;++i){
    pthread_mutex_lock(&pool_lock);
    omp_Node* temp;
    if(node_pool->pre == node_pool->next && node_pool->pre == NULL){
      // 此时node_pool为空，输出一下，暂时不做处理
      printf("Node_pool is empty!!!\n");
      return;
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
    temp->priority = priority;
    pthread_mutex_unlock(&pool_lock);

    pthread_mutex_lock(&queue_lock);
    if(queue->pre == queue->next && queue->pre == NULL){
      queue->pre = temp;
      queue->next = temp;
      temp->pre = queue;
      temp->next = queue;
    }else{
      omp_Node* pre = queue;
      while(pre->next != NULL && pre->next->priority <= temp->priority){
        pre = pre->next;
      }
      temp->next = queue->next;
      temp->pre = queue;
      temp->next->pre = temp;
      queue->next = temp;
    }
    pthread_mutex_unlock(&queue_lock);
  }
}
ialias (enqueue)

omp_Node* dequeue() {
  pthread_mutex_lock(&queue_lock);
  while (queue->pre == queue->next && queue->pre == NULL) {
    pthread_mutex_unlock(&queue_lock);
    return NULL;
  }
  // priority的值越小，任务节点的优先级越大
  omp_Node *temp = queue->next;
  if(queue->pre == queue->next && queue->pre != NULL){
    queue->pre = queue->next = NULL;
  }else{
    temp->pre->next = temp->next;
    temp->next->pre = temp->pre;
  }
  temp->pre = temp->next = NULL;
  pthread_mutex_unlock(&queue_lock);
  return temp;
}
ialias (dequeue)

omp_Node* dequeue2(int priority) {
  pthread_mutex_lock(&queue_lock);
  while (queue->pre == queue->next && queue->pre == NULL) {
    pthread_mutex_unlock(&queue_lock);
    return NULL;
  }
  omp_Node *temp = queue->next;
  // priority的值越小，任务节点的优先级越大
  if(temp->priority > priority){
    pthread_mutex_unlock(&queue_lock);
    return NULL;
  }
  if(queue->pre == queue->next && queue->pre != NULL){
    queue->pre = queue->next = NULL;
  }else{
    temp->pre->next = temp->next;
    temp->next->pre = temp->pre;
  }
  temp->pre = temp->next = NULL;
  pthread_mutex_unlock(&queue_lock);
  return temp;
}
ialias (dequeue2)

void dequeue1(omp_Node *temp){
  pthread_mutex_lock(&pool_lock);
  if(node_pool->pre == node_pool->next && node_pool->pre == NULL){
    node_pool->pre = node_pool->next = temp;
  }else{
    temp->pre = node_pool;
    temp->next = node_pool->next;
    node_pool->next->pre = temp;
    node_pool->next = temp;
  }
  pthread_mutex_unlock(&pool_lock);
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
