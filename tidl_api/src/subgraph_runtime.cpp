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

#include <pthread.h>
#define LOKI_PTHREAD_H
#include <loki/Singleton.h>

#include "util.h"
#include "subgraph_runtime.h"
#include "subgraph_runtime_impl.h"


#if 0
// Auto-generated code from Relay/TVM compilation step after
// partitioning and lowering to backend implementation

void TVM_TidlFunction(int total_subgraphs, int subgraph_id,
                     int num_input_tensors, int num_output_tensors,
                     PackedArgs args)
{
  float** in_data  = new float*[num_inputs_per_inference * batch_size];
  float** out_data = new float*[num_outputs_per_inference * batch_size];

  for (in j = 0; j < batch_size; j++)
  {
    for (int i = 0; i < num_inputs_per_inference + num_outputs_per_inference;
         i++)
      if (i < num_inputs_per_inference)
        in_data[j * num_inputs_per_inference + i] = args.data[i][j];
      else
        out_data[j * num_outpus_per_inference + i - num_inputs_per_inference]
                                                  = args.data[i][j];
  }

  // call into this function in libtidl.so
  // dlopen("libtidl_api.so")
  // TidlFunc = dlsym("TidlRunSubgraph");
  (*TidlFunc)(total_subgraphs, subgraph_id, batch_size
              num_inputs_per_inference, num_outputs_per_inference,
              in_data, out_data);

  delete [] in_data;
  delete [] out_data;
}
#endif


// Singleton ResM .cpp
using namespace tidl;

int TidlGetPreferredBatchSize(int total_subgraphs)
{
  ResM& res = ResM::Instance(total_subgraphs);
  return res.GetNumEs();
}

void TidlInitSubgraph(int total_subgraphs, int subgraph_id)
{
  ResM& res = ResM::Instance(total_subgraphs);
  res.InitSubgraph(subgraph_id);
}

void TidlFreeSubgraph(int total_subgraphs, int subgraph_id)
{
  ResM& res = ResM::Instance(total_subgraphs);
  res.FreeSubgraph(subgraph_id);
}

void TidlRunSubgraph(int total_subgraphs,
                     int subgraph_id,
                     int batch_size,
                     int num_inputs_per_inference,
                     int num_outputs_per_inference,
                     float **input_tensors,
                     float **output_tensors
                    )
{
  ResM& res = ResM::Instance(total_subgraphs);
  res.InitSubgraph(subgraph_id);
  int num_eops = res.GetNumEOPs(subgraph_id);
  if (num_eops > batch_size)  num_eops = batch_size;
  std::vector<ExecutionObjectPipeline*> eops(num_eops);
  for (int i = 0; i < num_eops; i++)
    eops[i] = res.GetEOP(subgraph_id);
  const SubgraphDataConv& in_conv  = res.GetInConv(subgraph_id);
  const SubgraphDataConv& out_conv = res.GetOutConv(subgraph_id);

  std::vector<std::vector<float *>> in_data_v(batch_size),
                                    out_data_v(batch_size);
  for (int frame_idx = 0; frame_idx < batch_size; frame_idx++)
  {
    for (int i = 0; i < num_inputs_per_inference; i++)
      in_data_v[frame_idx].emplace_back(input_tensors[
                                    frame_idx * num_inputs_per_inference + i]);
    for (int i = 0; i < num_outputs_per_inference; i++)
      out_data_v[frame_idx].emplace_back(output_tensors[
                                    frame_idx * num_inputs_per_inference + i]);
  }

  // Process batch_size frames with available eops in pipelined manner
  // additional num_eops iterations to flush the pipeline (epilogue)
  for (int frame_idx = 0; frame_idx < batch_size + num_eops; frame_idx++)
  {
    ExecutionObjectPipeline *eop = eops[frame_idx % num_eops];

    if (eop->ProcessFrameWait())
    {
      const uint8_t *out_data = (const uint8_t*) eop->GetOutputBufferPtr();
      out_conv.ScaleDequant(out_data, out_data_v[frame_idx - num_eops]);
    }

    if (frame_idx < batch_size)
    {
       uint8_t *in_data = (uint8_t *) eop->GetInputBufferPtr();
      in_conv.ScaleQuant(in_data_v[frame_idx], in_data);
      eop->ProcessFrameStartAsync();
    }
  }

  for (int i = 0; i < num_eops; i++)
      res.FreeEOP(subgraph_id, eops[i]);
}


typedef Loki::SingletonHolder <tidl::ResM, Loki::CreateUsingNew,
Loki::DefaultLifetime, Loki::ClassLevelLockable> tidlSingleResM;

ResM::ResM() : enable_trace_m(false), num_subgraphs_m(0),
               num_lg2_dsps_used_m(0), eops_m(nullptr)
{
}

ResM::~ResM()
{
  for (uint32_t i = 0; i < num_subgraphs_m; i++)
    FreeSubgraph(i);

  delete eops_m;
  eops_m = nullptr;
}

void ResM::FreeSubgraph(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);

  if (eops_m != nullptr)
  {
    ResEOP& res_eop = (*eops_m)[subgraph_id];
    if (res_eop.eops != nullptr)
    {
      for (const ExecutionObjectPipeline* eop : *(res_eop.eops))
      {
        free(eop->GetInputBufferPtr());
        free(eop->GetOutputBufferPtr());
        delete eop;
      }
      delete res_eop.eops;
      res_eop.eops = nullptr;
    }
  }

  delete es_m[subgraph_id];
  es_m[subgraph_id] = nullptr;

  delete e2s_m[subgraph_id];
  e2s_m[subgraph_id] = nullptr;

  delete in_conv_m[subgraph_id];
  in_conv_m[subgraph_id] = nullptr;

  delete out_conv_m[subgraph_id];
  out_conv_m[subgraph_id] = nullptr;
}

ResM& ResM::Instance(uint32_t total_num_subgraphs)
{
  ResM& res = tidlSingleResM::Instance();
  res.Init(total_num_subgraphs);
  return res;
}

void ResM::Init(uint32_t num_subgraphs)
{
  std::lock_guard<std::mutex> lock(mutex_init_m);

  if (num_subgraphs_m == 0)
  {
    num_subgraphs_m = num_subgraphs;

    if (getenv("TIDL_SUBGRAPH_TRACE") != nullptr)  enable_trace_m = true;

    // Allocating resources
    num_eves_m = Executor::GetNumDevices(DeviceType::EVE);
    num_dsps_m = Executor::GetNumDevices(DeviceType::DSP);

    assert(num_eves_m > 0 || num_dsps_m > 0);
    assert(num_subgraphs_m <= num_eves_m || num_subgraphs_m <= num_dsps_m);
    num_es_per_subgraph_m = num_eves_m / num_subgraphs_m;
    if (num_eves_m == 0)
      num_es_per_subgraph_m = num_dsps_m / num_subgraphs_m;

    cs_m.resize(num_subgraphs_m);
    es_m.resize(num_subgraphs_m, nullptr);
    e2s_m.resize(num_subgraphs_m, nullptr);
    eops_m = new std::vector<ResEOP>(num_subgraphs_m);
    in_conv_m.resize(num_subgraphs_m, nullptr);
    out_conv_m.resize(num_subgraphs_m, nullptr);
  }
}


void ResM::InitSubgraph(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);
  ResEOP& res_eop = (*eops_m)[subgraph_id];

  std::unique_lock<std::mutex> lock(res_eop.mutex_eops);

  // Constructing EOPs if not already constructed
  if (res_eop.eops == nullptr)
  {
    if (enable_trace_m)
      printf("Subgraph %d: initialing E/EOPs with %d cores\n",
             subgraph_id, num_es_per_subgraph_m);

    // Read config file
    std::string cfg_file = "subgraph" + std::to_string(subgraph_id) + ".cfg";
    bool status = cs_m[subgraph_id].ReadFromFile(cfg_file);
    assert(status);

    // Read the network
    sTIDL_Network_t *net = new sTIDL_Network_t;
    status = ReadNetworkBinary(cs_m[subgraph_id].netBinFile,
                               reinterpret_cast<char *>(net));
    assert(status);

    // Get data conversion info from configuration
    // Get input/output tensors dimensions from network
    // Construct data converters at the subgraph boundaries
    std::vector<int> inDims, outDims;
    for (int32_t layer = 0; layer < net->numLayers; layer++)
    {
      if (net->TIDLLayers[layer].layerType != (int32_t) TIDL_DataLayer)
        continue;
      if (net->TIDLLayers[layer].numInBufs <= 0)
      {
        for (int d = 0; d < 4; d++)
          inDims.push_back(net->TIDLLayers[layer].outData[0].dimValues[d]);
      }
      if (net->TIDLLayers[layer].numOutBufs <= 0)
      {
        for (int d = 0; d < 4; d++)
          outDims.push_back(net->TIDLLayers[layer].inData[0].dimValues[d]);
      }
    }
    assert(cs_m[subgraph_id].inIsNCHW.size() * 4 == inDims.size());
    assert(cs_m[subgraph_id].outIsNCHW.size() * 4 == outDims.size());
    std::vector<bool> inIsSigned, outIsSigned, inIsNCHW, outIsNCHW;
    for (int v : cs_m[subgraph_id].inIsSigned)  inIsSigned.push_back(v != 0);
    for (int v : cs_m[subgraph_id].inIsNCHW)    inIsNCHW.push_back(v != 0);
    for (int v : cs_m[subgraph_id].outIsSigned) outIsSigned.push_back(v != 0);
    for (int v : cs_m[subgraph_id].outIsNCHW)   outIsNCHW.push_back(v != 0);
    in_conv_m[subgraph_id] = new SubgraphDataConv(
                                               cs_m[subgraph_id].inConvType,
                                               inIsSigned,
                                               cs_m[subgraph_id].inScaleF2Q,
                                               inIsNCHW,
                                               inDims);
    out_conv_m[subgraph_id] = new SubgraphDataConv(
                                               cs_m[subgraph_id].outConvType,
                                               outIsSigned,
                                               cs_m[subgraph_id].outScaleF2Q,
                                               outIsNCHW,
                                               outDims);

    // Check if last few layers can be offloaded to DSPs
    //       and DSPs are available
    DeviceIds e_ids, e2_ids;
    for (uint32_t i = 0; i < num_es_per_subgraph_m; i++)
      e_ids.insert(static_cast<DeviceId>(
                               subgraph_id * num_es_per_subgraph_m + i));
    // uint32_t num_dsps_used = 0;
    if (num_eves_m > 0 && num_dsps_m > 0 && ! cs_m[subgraph_id].runFullNet)
    {
      if (cs_m[subgraph_id].layerIndex2LayerGroupId.empty())
      {
        int32_t start_layer = net->numLayers -1;
        int32_t end_layer = 0;
        if (net->TIDLLayers[start_layer].layerType == (int32_t) TIDL_DataLayer)
          start_layer -= 1;
        if (net->TIDLLayers[end_layer].layerType == (int32_t) TIDL_DataLayer)
          end_layer += 1;
        int32_t i = start_layer;
        for ( ; i > end_layer; i--)
        {
          int32_t layer_type = net->TIDLLayers[i].layerType;
          if (layer_type != (int32_t) TIDL_SoftMaxLayer &&
              layer_type != (int32_t) TIDL_InnerProductLayer &&
              layer_type != (int32_t) TIDL_PoolingLayer)
            break;
        }
        i += 1;
        if (i <= start_layer)
        {
          if (num_lg2_dsps_used_m < num_dsps_m)
          {
            if (enable_trace_m)
              printf("Subgraph %d: assign layers %d to %d to group 2 for DSP\n",
                     subgraph_id, i, start_layer);
            while (i <= start_layer)
              cs_m[subgraph_id].layerIndex2LayerGroupId[i++] = 2;
          }
        }
      }
      else
      {
        if (enable_trace_m)
          printf("Subgraph %d: using layer2group map in config file for DSP\n",
                 subgraph_id);
      }

      if (! cs_m[subgraph_id].layerIndex2LayerGroupId.empty())
      {
        e2_ids.insert(static_cast<DeviceId>(num_lg2_dsps_used_m));
        num_lg2_dsps_used_m += 1;
        if (num_subgraphs_m == 1)  // Allocate all dsps if only one subgraph
        {
          while (num_lg2_dsps_used_m < num_dsps_m)
            e2_ids.insert(static_cast<DeviceId>(num_lg2_dsps_used_m++));
        }
      }
    }
    delete net;

    if (e2_ids.empty())
      cs_m[subgraph_id].runFullNet = true;
    cs_m[subgraph_id].enableApiTrace = enable_trace_m;

    // Constructing Es and EOPs, each subgraph -> num_eves_per_subgraph_m EOPs
    res_eop.eops = new std::vector<ExecutionObjectPipeline*>;
    uint32_t buffer_factor = 2;  // double buffering factor
    if (num_eves_m > 0)
    {
      es_m[subgraph_id]  = new Executor(DeviceType::EVE, e_ids,
                                        cs_m[subgraph_id], 1);
      if (! e2_ids.empty())
      {
        e2s_m[subgraph_id] = new Executor(DeviceType::DSP, e2_ids,
                                          cs_m[subgraph_id], 2);
        for (uint32_t j = 0; j < buffer_factor; j++)
          for (uint32_t i = 0; i < num_es_per_subgraph_m; i++)
            res_eop.eops->emplace_back(new ExecutionObjectPipeline(
                                  {(*es_m[subgraph_id])[i],
                                   (*e2s_m[subgraph_id])[i % e2_ids.size()]}));
      }
      else
      {
        for (uint32_t j = 0; j < buffer_factor; j++)
          for (uint32_t i = 0; i < num_es_per_subgraph_m; i++)
            res_eop.eops->emplace_back(new ExecutionObjectPipeline(
                                                   {(*es_m[subgraph_id])[i]}));
      }
    }
    else
    {
      es_m[subgraph_id]  = new Executor(DeviceType::DSP, e_ids,
                                        cs_m[subgraph_id], 1);
      for (uint32_t j = 0; j < buffer_factor; j++)
        for (uint32_t i = 0; i < num_es_per_subgraph_m; i++)
          res_eop.eops->emplace_back(new ExecutionObjectPipeline(
                                                   {(*es_m[subgraph_id])[i]}));
    }

    if (enable_trace_m)
      printf("Subgraph %d: Allocating input/output buffers for %d EOPs\n",
             subgraph_id, res_eop.eops->size());
    // Allocate input/output buffers
    for (auto eop : *(res_eop.eops))
    {
      size_t in_size  = eop->GetInputBufferSizeInBytes();
      size_t out_size = eop->GetOutputBufferSizeInBytes();
      void*  in_ptr   = malloc(in_size);
      void*  out_ptr  = malloc(out_size);
      assert(in_ptr != nullptr && out_ptr != nullptr);

      ArgInfo in(in_ptr, in_size);
      ArgInfo out(out_ptr, out_size);
      eop->SetInputOutputBuffer(in, out);
    }

    res_eop.free_eop_index = 0;
    res_eop.is_used.resize(res_eop.eops->size(), false);
  }
}

uint32_t ResM::GetNumEOPs(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);
  ResEOP& res_eop = (*eops_m)[subgraph_id];
  assert (res_eop.eops != nullptr);

  return res_eop.eops->size();
}

ExecutionObjectPipeline* ResM::GetEOP(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);
  ResEOP& res_eop = (*eops_m)[subgraph_id];
  assert(res_eop.eops != nullptr);

  std::unique_lock<std::mutex> lock(res_eop.mutex_eops);

  // Return an available EOP (round robin allocation)
  uint32_t curr_eop = res_eop.free_eop_index;
  res_eop.cv_eops.wait(lock, [this, subgraph_id, curr_eop]{
           return this->eops_m->at(subgraph_id).is_used[curr_eop] == false; });
  res_eop.is_used[curr_eop] = true;
  res_eop.free_eop_index = (curr_eop + 1) % res_eop.eops->size();
  if (enable_trace_m)
    printf("Subgraph %d: return EOP %d for GetEOP()\n", subgraph_id, curr_eop);
  return res_eop.eops->at(curr_eop);
}

void ResM::FreeEOP(uint32_t subgraph_id, ExecutionObjectPipeline* eop)
{
  assert(subgraph_id < num_subgraphs_m);
  ResEOP& res_eop = (*eops_m)[subgraph_id];
  assert(res_eop.eops != nullptr);

  {
    std::unique_lock<std::mutex> lock(res_eop.mutex_eops);
    for (uint32_t i = 0; i < res_eop.is_used.size(); i++)
      if (res_eop.eops->at(i) == eop)
      {
        res_eop.is_used[i] = false;
        if (enable_trace_m)
          printf("Subgraph %d: FreeEOP %d\n", subgraph_id, i);
        break;
      }
  }
  res_eop.cv_eops.notify_all();
}

Configuration& ResM::GetConfiguration(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);
  assert((*eops_m)[subgraph_id].eops != nullptr);
  return cs_m[subgraph_id];
}

const SubgraphDataConv& ResM::GetInConv(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);
  assert(in_conv_m[subgraph_id] != nullptr);
  return *in_conv_m[subgraph_id];
}

const SubgraphDataConv& ResM::GetOutConv(uint32_t subgraph_id)
{
  assert(subgraph_id < num_subgraphs_m);
  assert(out_conv_m[subgraph_id] != nullptr);
  return *out_conv_m[subgraph_id];
}

