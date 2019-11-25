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

//! @file subgraph_runtime_impl.h

#pragma once
#include <vector>
#include <mutex>
#include <condition_variable>
#include "execution_object_pipeline.h"
#include "subgraph_data_conv.h"


namespace tidl {

// Singleton ResM   .h file
// Resource manager for available EVE and DSP devices,
//   - Allocates EVEs and DSPs
//   - Constructs Executors (tidl_setup) and ExecutionObjects (tid_init)
//   - Creates set of ExecutionPipelines (with or without DSP)
//   - Allocating EOP on demand (acquire and free semantics)
//   - Allocates input/output buffers
class ResM {
  public:
    ResM();
    ~ResM();
    static ResM& Instance(uint32_t total_num_subgraphs = 1);

    // how to get resources for subgraph_id
    void                     InitSubgraph(uint32_t subgraph_id);
    void                     FreeSubgraph(uint32_t subgraph_id);
    uint32_t                 GetNumEOPs(uint32_t subgraph_id);
    ExecutionObjectPipeline* GetEOP(uint32_t subgraph_id);
    void                     FreeEOP(uint32_t subgraph_id,
                                     ExecutionObjectPipeline* eop);
    Configuration&           GetConfiguration(uint32_t subgraph_id);
    const SubgraphDataConv&  GetInConv(uint32_t subgraph_id);
    const SubgraphDataConv&  GetOutConv(uint32_t subgraph_id);
    uint32_t                 GetNumEs() { return num_es_per_subgraph_m; }


  private:
    void Init(uint32_t num_subgraphs);

    bool     enable_trace_m;
    uint32_t num_subgraphs_m;
    uint32_t num_es_per_subgraph_m;
    uint32_t num_eves_m;
    uint32_t num_dsps_m;
    uint32_t num_lg2_dsps_used_m;  // in partitioned execution case
    std::mutex mutex_init_m;

    // indexed by subgraph_id for resources
    struct ResEOP {
      ResEOP() : free_eop_index(0), is_used(), eops(nullptr) {}

      uint32_t free_eop_index;
      std::mutex mutex_eops;
      std::condition_variable cv_eops;
      std::vector<bool> is_used;
      std::vector<ExecutionObjectPipeline*>* eops;
    };
    std::vector<Configuration> cs_m;
    std::vector<Executor*> es_m;
    std::vector<Executor*> e2s_m;
    std::vector<ResEOP> *eops_m;
    std::vector<SubgraphDataConv*> in_conv_m;
    std::vector<SubgraphDataConv*> out_conv_m;
};

} // namespace tidl
