#pragma once
// TrtEngine — a thin wrapper over one TensorRT engine, one instance per model. See
// docs/BUILD_GUIDE.md Part 7.4.
//
// ---------------------------------------------------------------------------------------------
// TensorRT's three objects
//
//   IRuntime          — deserialises a .engine file (built offline by trtexec, Part 7.3). Needed
//                       only while loading.
//   ICudaEngine       — the optimised network: weights plus the kernels chosen for this GPU.
//                       Read-only once loaded.
//   IExecutionContext — the per-inference state (intermediate activations, the tensor
//                       addresses to read and write). One engine can have several contexts;
//                       this project uses one context per engine.
//
// TensorRT 10 identifies inputs and outputs by tensor NAME ("images", "output0", ...). The guide's
// Part 7.4 snippet predates that. TensorRT 8's integer "binding" indices and enqueueV2 are gone.
// Each tensor gets a device address once, with setTensorAddress(), and enqueueV3() then runs the
// whole network.
//
// ---------------------------------------------------------------------------------------------
// CUDA streams: why infer is split into enqueue() and sync()
//
// A CUDA stream is an in-order queue of GPU work. Calls like enqueueV3 or cudaMemcpyAsync on a
// stream do not do the work. They append it to the queue and return immediately, while the GPU
// works through the queue in the background. Work in DIFFERENT streams has no ordering between
// streams, so the GPU is free to run it concurrently.
//
// That is what makes Part 7.4's "three models on independent CUDA streams" work. Each TrtEngine
// owns a stream, and MlInferenceEngine does:
//
//     yolo.enqueue(); lanes.enqueue(); depth.enqueue();   // three queues filled, nothing waited on
//     yolo.sync();    lanes.sync();    depth.sync();      // now wait for all three
//
// The three networks then overlap on the GPU, rather than each waiting for the previous one to
// finish, which is what a single blocking infer() per model would force.
//
// Outputs are copied into PINNED (page-locked) host memory. The OS can move ordinary heap memory,
// so a GPU copy from it has to go through a hidden staging buffer and cannot be truly
// asynchronous. Pinned memory can be written by the GPU directly, so the copy runs as part of the
// stream.
// ---------------------------------------------------------------------------------------------

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace nvinfer1 {
class IRuntime;
class ICudaEngine;
class IExecutionContext;
}  // namespace nvinfer1

namespace ar_drive_assist {

struct TensorInfo {
    std::string name;
    std::vector<int64_t> shape;  // fixed; dynamic shapes are rejected at load (see TrtEngine.cpp)
    std::size_t elements = 0;
};

class TrtEngine {
public:
    TrtEngine();
    ~TrtEngine();
    TrtEngine(const TrtEngine&) = delete;
    TrtEngine& operator=(const TrtEngine&) = delete;

    // Loads a serialized engine. Throws std::runtime_error with the reason on any failure: a
    // model that half-loads must not reach the inference loop.
    void load(const std::string& enginePath);

    const TensorInfo& input() const { return input_; }
    const std::vector<TensorInfo>& outputs() const { return outputs_; }

    // Device pointer to the FP32 NCHW input tensor. Pre-processing writes here, on stream(), so
    // the frame never makes a round trip through host memory.
    float* deviceInput() { return static_cast<float*>(deviceBuffers_.front()); }
    cudaStream_t stream() const { return stream_; }

    // Queues inference plus the device-to-host copy of every output on stream(). Returns
    // immediately; nothing may be read from hostOutput() until sync() returns.
    void enqueue();
    // Blocks until everything queued on stream() has finished. Throws on a CUDA error.
    void sync();

    // Host copy of output `index` (order of outputs()), valid after sync().
    const float* hostOutput(std::size_t index) const { return hostOutputs_.at(index); }

    // Convenience for tests and tools: host input -> blocking inference.
    void inferBlocking(const std::vector<float>& hostInput);

private:
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_ = nullptr;

    TensorInfo input_;
    std::vector<TensorInfo> outputs_;
    std::vector<void*> deviceBuffers_;  // [0] = input, then outputs in outputs_ order
    std::vector<float*> hostOutputs_;   // pinned
};

}  // namespace ar_drive_assist
