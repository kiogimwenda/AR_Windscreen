#include "ar_drive_assist/inference/TrtEngine.h"

#include <NvInfer.h>

#include <cstdio>
#include <fstream>
#include <iterator>

namespace ar_drive_assist {
namespace {

// TensorRT reports through a logger object it calls back into. Warnings and errors go to
// stderr; its INFO/VERBOSE chatter is dropped.
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) std::fprintf(stderr, "[TensorRT] %s\n", msg);
    }
};
Logger gLogger;

void check(cudaError_t err, const char* what) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("TrtEngine: ") + what + ": " +
                                 cudaGetErrorString(err));
    }
}

TensorInfo describe(const nvinfer1::ICudaEngine& engine, const char* name,
                    const std::string& path) {
    if (engine.getTensorDataType(name) != nvinfer1::DataType::kFLOAT) {
        // --fp16 in Part 7.3 changes INTERNAL precision only; the engine's inputs and outputs stay
        // FP32 unless the ONNX itself declared otherwise. An FP16 I/O tensor here means the export
        // changed, and every buffer size below would be wrong by 2x.
        throw std::runtime_error("TrtEngine: " + path + ": tensor '" + name + "' is not FP32");
    }
    const nvinfer1::Dims dims = engine.getTensorShape(name);
    TensorInfo t;
    t.name = name;
    t.elements = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        // -1 marks a dynamic dimension. Every export in this project fixes its shapes (see
        // scripts/export_onnx.py), so a -1 means the wrong engine was loaded.
        if (dims.d[i] < 0) {
            throw std::runtime_error("TrtEngine: " + path + ": tensor '" + name +
                                     "' has a dynamic shape");
        }
        t.shape.push_back(dims.d[i]);
        t.elements *= static_cast<std::size_t>(dims.d[i]);
    }
    return t;
}

}  // namespace

TrtEngine::TrtEngine() = default;

TrtEngine::~TrtEngine() {
    // Order matters: the context references the engine, and the engine was produced by the runtime.
    // The stream is synchronised first, so nothing is still writing into the buffers freed below.
    if (stream_) cudaStreamSynchronize(stream_);
    context_.reset();
    engine_.reset();
    runtime_.reset();
    for (void* p : deviceBuffers_) cudaFree(p);
    for (float* p : hostOutputs_) cudaFreeHost(p);
    if (stream_) cudaStreamDestroy(stream_);
}

void TrtEngine::load(const std::string& enginePath) {
    if (engine_) throw std::logic_error("TrtEngine::load called twice");

    std::ifstream file(enginePath, std::ios::binary);
    if (!file) throw std::runtime_error("TrtEngine: cannot open " + enginePath);
    const std::vector<char> blob((std::istreambuf_iterator<char>(file)),
                                 std::istreambuf_iterator<char>());

    runtime_.reset(nvinfer1::createInferRuntime(gLogger));
    if (!runtime_) throw std::runtime_error("TrtEngine: createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) {
        // Most often an engine built with a different TensorRT version or on a different GPU
        // (Part 7.3: engines are per machine). TensorRT has already logged the exact reason.
        throw std::runtime_error("TrtEngine: cannot deserialize " + enginePath +
                                 " (built for another GPU or TensorRT version?)");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("TrtEngine: createExecutionContext failed");

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* name = engine_->getIOTensorName(i);
        TensorInfo t = describe(*engine_, name, enginePath);
        if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
            if (!input_.name.empty()) {
                throw std::runtime_error("TrtEngine: " + enginePath + " has more than one input");
            }
            input_ = std::move(t);
        } else {
            outputs_.push_back(std::move(t));
        }
    }
    if (input_.name.empty() || outputs_.empty()) {
        throw std::runtime_error("TrtEngine: " + enginePath + " needs one input and >=1 output");
    }

    // A non-blocking stream does not implicitly wait on the legacy "default stream". Without this
    // flag, any library call that happens to use the default stream would serialise all three
    // engines again and quietly undo the overlap.
    check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");

    void* buf = nullptr;
    check(cudaMalloc(&buf, input_.elements * sizeof(float)), "cudaMalloc(input)");
    deviceBuffers_.push_back(buf);
    if (!context_->setTensorAddress(input_.name.c_str(), buf)) {
        throw std::runtime_error("TrtEngine: setTensorAddress failed for " + input_.name);
    }
    for (const TensorInfo& out : outputs_) {
        check(cudaMalloc(&buf, out.elements * sizeof(float)), "cudaMalloc(output)");
        deviceBuffers_.push_back(buf);
        if (!context_->setTensorAddress(out.name.c_str(), buf)) {
            throw std::runtime_error("TrtEngine: setTensorAddress failed for " + out.name);
        }
        void* host = nullptr;
        check(cudaMallocHost(&host, out.elements * sizeof(float)), "cudaMallocHost");
        hostOutputs_.push_back(static_cast<float*>(host));
    }
}

void TrtEngine::enqueue() {
    if (!context_) throw std::logic_error("TrtEngine::enqueue before load");
    if (!context_->enqueueV3(stream_)) throw std::runtime_error("TrtEngine: enqueueV3 failed");
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
        check(
            cudaMemcpyAsync(hostOutputs_[i], deviceBuffers_[i + 1],
                            outputs_[i].elements * sizeof(float), cudaMemcpyDeviceToHost, stream_),
            "cudaMemcpyAsync(output)");
    }
}

void TrtEngine::sync() {
    check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
}

void TrtEngine::inferBlocking(const std::vector<float>& hostInput) {
    if (hostInput.size() != input_.elements) {
        throw std::invalid_argument("TrtEngine::inferBlocking: input has " +
                                    std::to_string(hostInput.size()) +
                                    " elements, engine expects " + std::to_string(input_.elements));
    }
    check(cudaMemcpyAsync(deviceInput(), hostInput.data(), hostInput.size() * sizeof(float),
                          cudaMemcpyHostToDevice, stream_),
          "cudaMemcpyAsync(input)");
    enqueue();
    sync();
}

}  // namespace ar_drive_assist
