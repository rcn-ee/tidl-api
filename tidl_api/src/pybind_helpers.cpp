#include "pybind_common.h"

template<typename T>
void AllocateMemoryT(const vector<T *>& eos)
{
    // Allocate input and output buffers for each execution object
    for (auto eo : eos)
    {
        size_t in_size  = eo->GetInputBufferSizeInBytes();
        size_t out_size = eo->GetOutputBufferSizeInBytes();
        void*  in_ptr   = malloc(in_size);
        void*  out_ptr  = malloc(out_size);
        assert(in_ptr != nullptr && out_ptr != nullptr);

        ArgInfo in  = { ArgInfo(in_ptr,  in_size)};
        ArgInfo out = { ArgInfo(out_ptr, out_size)};
        eo->SetInputOutputBuffer(in, out);
    }
}

// Allocate input and output memory for each EO
void AllocateMemory(const vector<ExecutionObject *>& eos)
{
    AllocateMemoryT<ExecutionObject>(eos);
}

// Allocate input and output memory for each EO
void AllocateMemory(const vector<ExecutionObjectPipeline *>& eos)
{
    AllocateMemoryT<ExecutionObjectPipeline>(eos);
}



// Free the input and output memory associated with each EO
template<typename T>
void FreeMemoryT(const vector<T *>& eos)
{
    for (auto eo : eos)
    {
        free(eo->GetInputBufferPtr());
        free(eo->GetOutputBufferPtr());
    }
}

void FreeMemory(const vector<ExecutionObject *>& eos)
{
    FreeMemoryT<ExecutionObject>(eos);
}


// Free the input and output memory associated with each EO
void FreeMemory(const vector<ExecutionObjectPipeline *>& eos)
{
    FreeMemoryT<ExecutionObjectPipeline>(eos);
}

