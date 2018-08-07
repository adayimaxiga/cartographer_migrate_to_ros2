/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/thread_pool.h"

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/task.h"
#include "glog/logging.h"


/*  2018.8.6    LD
 *
 *  cartographer 源码阅读
 *
 *  本文件功能：
 *
 *  线程池，初始化一个线程数目固定的线程池
 *
 *  */
namespace cartographer {
namespace common {

void ThreadPoolInterface::Execute(Task* task) { task->Execute(); }

void ThreadPoolInterface::SetThreadPool(Task* task) {
  task->SetThreadPool(this);
}

ThreadPool::ThreadPool(int num_threads) {
  MutexLocker locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });//每个线程初始化执行DoWork，函数在下面
  }
}

//线程池为0时才能析构，否则check过不去
ThreadPool::~ThreadPool() {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
  }
  for (std::thread& thread : pool_) {
    thread.join();
  }
}

void ThreadPool::NotifyDependenciesCompleted(Task* task) {
  MutexLocker locker(&mutex_);
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());
  task_queue_.push_back(it->second);
  tasks_not_ready_.erase(it);
}
/*
 * 让线程池执行 task函数。
 * */
std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
  std::shared_ptr<Task> shared_task;
  {
    MutexLocker locker(&mutex_);
    auto insert_result =
        tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
    CHECK(insert_result.second) << "Schedule called twice";
    shared_task = insert_result.first->second;
  }
  SetThreadPool(shared_task.get());
  return shared_task;
}
//每个线程执行这个函数
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  for (;;) {
    std::shared_ptr<Task> task;
    {
      //加锁，同一时刻只能一个线程领取任务。
      MutexLocker locker(&mutex_);
      locker.Await([this]() REQUIRES(mutex_) {
        return !task_queue_.empty() || !running_;
        //队列不为空或者没有运行。
      });
      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());  //领取任务
        task_queue_.pop_front();                //删除。
      } else if (!running_) {
        return;
      }
      //结束解锁。
    }
    CHECK(task);
    CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);
    Execute(task.get());        //执行任务
  }
}

}  // namespace common
}  // namespace cartographer
