/******************************************************************************
 * Copyright (c) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of Texas Instruments Incorporated nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "thread_pool.h"

using namespace std;
using namespace tidl;

void ThFunc(int th_id, ThPool* pool)
{
  while (true)
  {
    // wait on th_id
    pool->WaitForWork(th_id);

    // check stop condition
    if (pool->Stop())  return;

    // Run user func
    pool->RunUserFunc(th_id);

    // notify completition
    pool->NotifyCompletion(th_id);
  }
}

ThPool::ThPool(int num_threads, UserFunc user_func) :
        num_threads_m(num_threads),
        user_func_m(user_func),
        stop_m(false),
        pool_m(num_threads),
        pool_state_m((1ULL << num_threads) - 1),
        v_mutex_th_m(num_threads),
        v_cv_th_work_m(num_threads),
        v_cv_th_completion_m(num_threads),
        v_user_data_m(num_threads, nullptr),
        v_completion_data_m(num_threads, nullptr)
{
  for (int i = 0; i < num_threads_m; i++)
  {
    pool_m[i] = thread(ThFunc, i, this);
  }
}

ThPool::~ThPool()
{
  stop_m = true;
  for (auto& data : v_user_data_m)  data = &stop_m;
  for (auto& cv : v_cv_th_work_m)   cv.notify_all();
  for (auto& th : pool_m)           th.join();
}

int ThPool::RunAsync(void *user_data)
{
  int th_id = -1;
  {
    std::unique_lock<std::mutex> lock(mutex_pool_m);
    cv_pool_m.wait(lock, [this]{ return this->pool_state_m != 0; });
    // find first 1 bit
    for (int i = 0; i < num_threads_m; i++)
      if (pool_state_m & (1 << i))
      {
        th_id = i;
        break;
      }
    pool_state_m &= (~ (1 << th_id));
  }

  {
    std::unique_lock<std::mutex> lock(v_mutex_th_m[th_id]);
    v_user_data_m[th_id] = user_data;
  }
  v_cv_th_work_m[th_id].notify_all();
  return th_id;
}

void* ThPool::Wait(int th_id)
{
  void *user_data = nullptr;

  {
    std::unique_lock<std::mutex> lock(v_mutex_th_m[th_id]);
    v_cv_th_completion_m[th_id].wait(lock, [this, th_id]{
                       return this->v_completion_data_m[th_id] != nullptr; });
    user_data = v_completion_data_m[th_id];
    v_completion_data_m[th_id] = nullptr;
  }

  {
    std::unique_lock<std::mutex> lock(mutex_pool_m);
    pool_state_m |= (1 << th_id);
  }
  cv_pool_m.notify_all();

  return user_data;
}


void ThPool::RunUserFunc(int th_id)
{
  user_func_m(v_user_data_m[th_id]);
}

void ThPool::WaitForWork(int th_id)
{
    std::unique_lock<std::mutex> lock(v_mutex_th_m[th_id]);
    v_cv_th_work_m[th_id].wait(lock, [this, th_id]{
                              return this->v_user_data_m[th_id] != nullptr; });
}

void ThPool::NotifyCompletion(int th_id)
{
  {
    std::unique_lock<std::mutex> lock(v_mutex_th_m[th_id]);
    v_completion_data_m[th_id] = v_user_data_m[th_id];
    v_user_data_m[th_id] = nullptr;
  }
  v_cv_th_completion_m[th_id].notify_all();
}
