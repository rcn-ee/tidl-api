/******************************************************************************
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of Texas Instruments Incorporated nor the
 *         names of its contributors may be used to endorse or promote products
 *         derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *   THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <vector>
#include <chrono>

#define MAX_WINDOW_SIZE 64
#define INIT_FRAME_TIME 0.001

// Compute average FPS across a sliding window of frames
class AvgFPSWindow
{
  public:
    AvgFPSWindow(uint32_t window_size) :
        window_size_m(window_size), circ_idx_m(0), total_time_m(0.0)
    {
        if (window_size_m == 0 || window_size_m > MAX_WINDOW_SIZE)
            window_size_m = MAX_WINDOW_SIZE;
        history_times_m.assign(window_size_m, INIT_FRAME_TIME);
        frame_time_m = INIT_FRAME_TIME;
        total_time_m = window_size_m * INIT_FRAME_TIME;
        t0_m = std::chrono::steady_clock::now();
    }

    // Invoked per loop iteration to capture frame time
    void Tick()
    {
        t1_m = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t1_m - t0_m;
        frame_time_m = elapsed.count();  // in seconds
        t0_m = t1_m;
    }

    // Update the frame_time_m into circular array for history timing
    // Reading will only become valid after window_size_m frames
    // Return updated average FPS
    double UpdateAvgFPS()
    {
        total_time_m += frame_time_m - history_times_m[circ_idx_m];
        history_times_m[circ_idx_m] = frame_time_m;
        circ_idx_m = (circ_idx_m + 1) % window_size_m;
        return (1.0 * window_size_m) / total_time_m;
    }

    // Return average FPS
    double GetAvgFPS()
    {
        return (1.0 * window_size_m) / total_time_m;
    }

    AvgFPSWindow() =delete;
    AvgFPSWindow(const AvgFPSWindow&) =delete;
    AvgFPSWindow& operator=(const AvgFPSWindow&) =delete;

  private:
    uint32_t window_size_m;
    uint32_t circ_idx_m;
    double total_time_m;
    std::vector<double> history_times_m;
    std::chrono::time_point<std::chrono::steady_clock> t0_m, t1_m;
    double frame_time_m;
};
