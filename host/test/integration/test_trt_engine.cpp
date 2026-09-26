// TrtEngine GPU integration test — see docs/BUILD_GUIDE.md Part 7.4.
//
// Needs a real NVIDIA GPU, so this does not run in CI (Part 13.5). It needs none of the project's
// downloaded models. The test builds its own tiny network in code, with TensorRT's builder API:
//
//     x (1x3x4x4)  ──►  y = 2x + 1
//                  └──► r = relu(x)
//
// It serialises that network to an engine file, as trtexec does, then loads and runs it through
// TrtEngine. The outputs are computable by hand, so every element can be checked exactly. That
// covers the parts of TrtEngine that are easy to get wrong: tensor discovery by name, multiple
// outputs in the right order, stream-ordered copies into pinned memory, and rejecting bad
// engines.

#include <NvInfer.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "ar_drive_assist/inference/TrtEngine.h"

using ar_drive_assist::TrtEngine;

namespace {

class QuietLogger : public nvinfer1::ILogger {
    void log(Severity s, const char* msg) noexcept override {
        if (s <= Severity::kERROR) std::fprintf(stderr, "[TensorRT build] %s\n", msg);
    }
} gBuildLogger;

// What trtexec does with an ONNX file, done by hand for a two-layer network: define the network,
// let the builder optimise it for this GPU, serialise the result.
std::string buildTinyEngine(const std::string& path) {
    std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(gBuildLogger));
    std::unique_ptr<nvinfer1::INetworkDefinition> net(builder->createNetworkV2(0));

    auto* x = net->addInput("x", nvinfer1::DataType::kFLOAT, nvinfer1::Dims4{1, 3, 4, 4});

    // Constants of shape 1x1x1x1 broadcast across x. The weight memory must outlive the build.
    static const float kTwo = 2.0f, kOne = 1.0f;
    auto* two = net->addConstant(nvinfer1::Dims4{1, 1, 1, 1},
                                 nvinfer1::Weights{nvinfer1::DataType::kFLOAT, &kTwo, 1});
    auto* one = net->addConstant(nvinfer1::Dims4{1, 1, 1, 1},
                                 nvinfer1::Weights{nvinfer1::DataType::kFLOAT, &kOne, 1});
    auto* mul = net->addElementWise(*x, *two->getOutput(0), nvinfer1::ElementWiseOperation::kPROD);
    auto* add = net->addElementWise(*mul->getOutput(0), *one->getOutput(0),
                                    nvinfer1::ElementWiseOperation::kSUM);
    add->getOutput(0)->setName("y");
    net->markOutput(*add->getOutput(0));

    auto* relu = net->addActivation(*x, nvinfer1::ActivationType::kRELU);
    relu->getOutput(0)->setName("r");
    net->markOutput(*relu->getOutput(0));

    std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
    std::unique_ptr<nvinfer1::IHostMemory> plan(builder->buildSerializedNetwork(*net, *config));
    if (!plan) return "";
    std::ofstream(path, std::ios::binary)
        .write(static_cast<const char*>(plan->data()), static_cast<std::streamsize>(plan->size()));
    return path;
}

class TrtEngineTest : public testing::Test {
protected:
    static void SetUpTestSuite() {
        enginePath_ = testing::TempDir() + "tiny_" + std::to_string(::getpid()) + ".engine";
        ASSERT_FALSE(buildTinyEngine(enginePath_).empty()) << "TensorRT failed to build the engine";
    }
    static void TearDownTestSuite() { std::remove(enginePath_.c_str()); }
    static std::string enginePath_;
};
std::string TrtEngineTest::enginePath_;

std::vector<float> ramp() {
    std::vector<float> in(48);
    for (std::size_t i = 0; i < in.size(); ++i) in[i] = static_cast<float>(i) - 24.0f;  // -24..23
    return in;
}

}  // namespace

TEST_F(TrtEngineTest, DiscoversTensorsByNameWithFixedShapes) {
    TrtEngine e;
    e.load(enginePath_);
    EXPECT_EQ(e.input().name, "x");
    EXPECT_EQ(e.input().shape, (std::vector<int64_t>{1, 3, 4, 4}));
    EXPECT_EQ(e.input().elements, 48u);
    ASSERT_EQ(e.outputs().size(), 2u);
    // Order is TensorRT's I/O order, which is the order outputs were marked. Callers index
    // hostOutput() by the position of the name in outputs(), never by assuming a position.
    std::vector<std::string> names{e.outputs()[0].name, e.outputs()[1].name};
    std::sort(names.begin(), names.end());
    EXPECT_EQ(names, (std::vector<std::string>{"r", "y"}));
}

TEST_F(TrtEngineTest, ComputesEveryElementCorrectly) {
    TrtEngine e;
    e.load(enginePath_);
    const auto in = ramp();
    e.inferBlocking(in);

    for (std::size_t o = 0; o < e.outputs().size(); ++o) {
        const bool isY = e.outputs()[o].name == "y";
        const float* out = e.hostOutput(o);
        for (std::size_t i = 0; i < in.size(); ++i) {
            const float expected = isY ? 2.0f * in[i] + 1.0f : std::max(in[i], 0.0f);
            ASSERT_FLOAT_EQ(out[i], expected) << e.outputs()[o].name << "[" << i << "]";
        }
    }
}

// Outputs of a later run must reflect THAT run's input: catches a copy left on the wrong stream
// or read before sync().
TEST_F(TrtEngineTest, RepeatedRunsSeeTheirOwnInput) {
    TrtEngine e;
    e.load(enginePath_);
    std::size_t yIndex = e.outputs()[0].name == "y" ? 0 : 1;
    for (float v : {1.0f, -3.0f, 10.5f}) {
        e.inferBlocking(std::vector<float>(48, v));
        EXPECT_FLOAT_EQ(e.hostOutput(yIndex)[47], 2.0f * v + 1.0f);
    }
}

// Two engines enqueued back to back on their own streams, then both synced: the Part 7.4
// pattern. Both results must be correct: the concurrency must not mix up buffers.
TEST_F(TrtEngineTest, TwoEnginesOnSeparateStreamsBothCorrect) {
    TrtEngine a, b;
    a.load(enginePath_);
    b.load(enginePath_);
    ASSERT_NE(a.stream(), b.stream());

    std::vector<float> inA(48, 1.0f), inB(48, 5.0f);
    cudaMemcpy(a.deviceInput(), inA.data(), 48 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(b.deviceInput(), inB.data(), 48 * sizeof(float), cudaMemcpyHostToDevice);
    a.enqueue();
    b.enqueue();
    a.sync();
    b.sync();

    const std::size_t ya = a.outputs()[0].name == "y" ? 0 : 1;
    const std::size_t yb = b.outputs()[0].name == "y" ? 0 : 1;
    EXPECT_FLOAT_EQ(a.hostOutput(ya)[0], 3.0f);
    EXPECT_FLOAT_EQ(b.hostOutput(yb)[0], 11.0f);
}

TEST_F(TrtEngineTest, RejectsWrongSizedInput) {
    TrtEngine e;
    e.load(enginePath_);
    EXPECT_THROW(e.inferBlocking(std::vector<float>(47, 0.0f)), std::invalid_argument);
}

TEST(TrtEngineLoad, MissingFileThrows) {
    TrtEngine e;
    EXPECT_THROW(e.load("/nonexistent.engine"), std::runtime_error);
}

TEST(TrtEngineLoad, GarbageFileThrows) {
    const std::string p = testing::TempDir() + "garbage_" + std::to_string(::getpid()) + ".engine";
    std::ofstream(p) << "this is not a TensorRT engine";
    TrtEngine e;
    EXPECT_THROW(e.load(p), std::runtime_error);
    std::remove(p.c_str());
}

// --- The project's real engines (built by scripts/build_tensorrt_engines.sh)
// ---------------------- These pin down the tensor names and shapes the pre- and post-processing
// code depends on. A re-export at a different size fails here, not as garbage boxes on screen.
// Skipped (not failed) when the engines have not been built on this machine, since engines are
// per-machine and gitignored.

namespace {
struct ExpectedEngine {
    const char* file;
    const char* input;
    std::vector<int64_t> inputShape;
    std::vector<std::pair<std::string, std::vector<int64_t>>> outputs;
};
// GoogleTest prints a parameter's raw bytes unless told how to print it.
void PrintTo(const ExpectedEngine& e, std::ostream* os) {
    *os << e.file;
}

const std::vector<ExpectedEngine> kRealEngines = {
    {"yolov8m.engine", "images", {1, 3, 736, 1280}, {{"output0", {1, 84, 19320}}}},
    {"ufld.engine",
     "input",
     {1, 3, 320, 1600},
     {{"loc_row", {1, 200, 72, 4}},
      {"loc_col", {1, 100, 81, 4}},
      {"exist_row", {1, 2, 72, 4}},
      {"exist_col", {1, 2, 81, 4}}}},
    {"midas.engine", "input", {1, 3, 256, 448}, {{"depth", {1, 256, 448}}}},
    // YOLOv8m-seg: 4 box + 80 class + 32 mask-coefficient channels, and 32 prototypes at 1/4 res.
    {"yolov8m_seg.engine",
     "images",
     {1, 3, 736, 1280},
     {{"output0", {1, 116, 19320}}, {"output1", {1, 32, 184, 320}}}},
    // Sign detector: 4 box + 29 sign-class channels.
    {"signs.engine", "images", {1, 3, 736, 1280}, {{"output0", {1, 33, 19320}}}},
};
}  // namespace

class RealEngine : public testing::TestWithParam<ExpectedEngine> {};

TEST_P(RealEngine, LoadsWithExpectedTensorsAndProducesFiniteOutput) {
    const ExpectedEngine& want = GetParam();
    const std::string path = std::string(HOST_SOURCE_DIR) + "/models/engines/" + want.file;
    if (!std::ifstream(path)) GTEST_SKIP() << path << " not built on this machine";

    TrtEngine e;
    e.load(path);
    EXPECT_EQ(e.input().name, want.input);
    EXPECT_EQ(e.input().shape, want.inputShape);
    ASSERT_EQ(e.outputs().size(), want.outputs.size());
    for (const auto& [name, shape] : want.outputs) {
        auto it = std::find_if(e.outputs().begin(), e.outputs().end(),
                               [&](const auto& t) { return t.name == name; });
        ASSERT_NE(it, e.outputs().end()) << "missing output " << name;
        EXPECT_EQ(it->shape, shape) << name;
    }

    // A mid-grey frame: every output must be finite. A NaN/Inf here usually means an FP16
    // overflow in a layer TensorRT should have kept in FP32.
    e.inferBlocking(std::vector<float>(e.input().elements, 0.5f));
    for (std::size_t o = 0; o < e.outputs().size(); ++o) {
        const float* out = e.hostOutput(o);
        const std::size_t n = e.outputs()[o].elements;
        EXPECT_TRUE(std::all_of(out, out + n, [](float v) { return std::isfinite(v); }))
            << e.outputs()[o].name;
    }
}

INSTANTIATE_TEST_SUITE_P(Phase5, RealEngine, testing::ValuesIn(kRealEngines), [](const auto& info) {
    std::string n = info.param.file;
    return n.substr(0, n.find('.'));
});
