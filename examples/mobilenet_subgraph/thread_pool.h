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

#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

using namespace std;

namespace tidl {

#define TIDL_MAX_NUM_THREADS  32

typedef void(*UserFunc)(void *user_data);

class ThPool {
  public:
    ThPool(int num_threads, UserFunc user_func);
    ~ThPool();
    // returns th_id that can be used for Wait()
    int RunAsync(void* user_data);
    void* Wait(int th_id);

    // Run by threaded function
    bool Stop()  { return stop_m; }
    void RunUserFunc(int th_id);
    void WaitForWork(int th_id);
    void NotifyCompletion(int th_id);

  private:

    int                num_threads_m;
    UserFunc           user_func_m;
    bool               stop_m;
    vector<thread>     pool_m;
    mutex              mutex_pool_m;
    condition_variable cv_pool_m;
    // bit vector for availability, up to 32 threads, 1: avail, 0: not avail
    int32_t            pool_state_m;

    vector<mutex>              v_mutex_th_m;
    vector<condition_variable> v_cv_th_work_m;
    vector<condition_variable> v_cv_th_completion_m;

    vector<void *>             v_user_data_m;
    vector<void *>             v_completion_data_m;
};

}  // namespace tidl
