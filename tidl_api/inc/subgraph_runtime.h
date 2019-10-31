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

//! @file subgraph_runtime.h

#pragma once

extern "C" {

//! @brief Top level inference to run a TIDL subgraph
//! @param total_subgraphs  total number of TIDL subgraphs in whole inference
//! @param subgraph_id  index of current TIDL subgraph
//! @param batch_size  number of samples/inferences in this batch
//! @param num_inputs_per_inference  number of inputs to TIDL subgraph
//!        for every sample/inference
//! @param num_outputs_per_inference  number of outputs from TIDL subgraph
//!        for every sample/inference
//! @param input_tensors  input data to TIDL subgraph, layout as
//!        batch1_input1, batch1_input2, ..., batch1_inputM,
//!        ... ... ...
//!        batchN_input1, batchN_input2, ..., batchN_inputM
//! @param output_tensors  output data from TIDL subgraph, layout as
//!        batch1_output1, batch1_output2, ..., batch1_outputK,
//!        ... ... ...
//!        batchN_output1, batchN_output2, ..., batchN_outputK
extern void TidlRunSubgraph(int total_subgraphs,
                            int subgraph_id,
                            int batch_size,
                            int num_inputs_per_inference,
                            int num_outputs_per_inference,
                            float **input_tensors,
                            float **output_tensors
                           );

}  // extern "C"


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

