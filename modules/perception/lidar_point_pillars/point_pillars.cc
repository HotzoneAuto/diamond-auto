// headers in STL
#include <chrono>
#include <iostream>

#include "cyber/common/log.h"

// headers in local files
#include "modules/perception/lidar_point_pillars/point_pillars.h"
using namespace std;

namespace apollo {
namespace perception {
namespace lidar {

const float PointPillars::kPillarXSize = Params::kPillarXSize;
const float PointPillars::kPillarYSize = Params::kPillarYSize;
const float PointPillars::kPillarZSize = Params::kPillarZSize;
const float PointPillars::kMinXRange = Params::kMinXRange;
const float PointPillars::kMinYRange = Params::kMinYRange;
const float PointPillars::kMinZRange = Params::kMinZRange;
const float PointPillars::kMaxXRange = Params::kMaxXRange;
const float PointPillars::kMaxYRange = Params::kMaxYRange;
const float PointPillars::kMaxZRange = Params::kMaxZRange;
const int PointPillars::kNumClass = Params::kNumClass;
const int PointPillars::kMaxNumPillars = Params::kMaxNumPillars;
const int PointPillars::kMaxNumPointsPerPillar = Params::kMaxNumPointsPerPillar;
const int PointPillars::kNumPointFeature = Params::kNumPointFeature;
const int PointPillars::kGridXSize =
    static_cast<int>((kMaxXRange - kMinXRange) / kPillarXSize);
const int PointPillars::kGridYSize =
    static_cast<int>((kMaxYRange - kMinYRange) / kPillarYSize);
const int PointPillars::kGridZSize =
    static_cast<int>((kMaxZRange - kMinZRange) / kPillarZSize);
const int PointPillars::kRpnInputSize = 64 * kGridXSize * kGridYSize;
const int PointPillars::kNumAnchor = Params::kNumAnchor;
const int PointPillars::kNumOutputBoxFeature = Params::kNumOutputBoxFeature;
const int PointPillars::kRpnBoxOutputSize = kNumAnchor * kNumOutputBoxFeature;
const int PointPillars::kRpnClsOutputSize = kNumAnchor * kNumClass;
const int PointPillars::kRpnDirOutputSize = kNumAnchor * 2;
const int PointPillars::kBatchSize = Params::kBatchSize;
const int PointPillars::kNumIndsForScan = Params::kNumIndsForScan;
const int PointPillars::kNumThreads = Params::kNumThreads;
// if you change kNumThreads, need to modify NUM_THREADS_MACRO in
// common.h
const int PointPillars::kNumBoxCorners = Params::kNumBoxCorners;
const std::vector<int> PointPillars::kAnchorStrides = Params::AnchorStrides();
const std::vector<int> PointPillars::kAnchorRanges{
    0,
    kGridXSize,
    0,
    kGridYSize,
    static_cast<int>(kGridXSize * 0.25),
    static_cast<int>(kGridXSize * 0.75),
    static_cast<int>(kGridYSize * 0.25),
    static_cast<int>(kGridYSize * 0.75)};
const std::vector<int> PointPillars::kNumAnchorSets = Params::NumAnchorSets();
const std::vector<std::vector<float>> PointPillars::kAnchorDxSizes =
    Params::AnchorDxSizes();
const std::vector<std::vector<float>> PointPillars::kAnchorDySizes =
    Params::AnchorDySizes();
const std::vector<std::vector<float>> PointPillars::kAnchorDzSizes =
    Params::AnchorDzSizes();
const std::vector<std::vector<float>> PointPillars::kAnchorZCoors =
    Params::AnchorZCoors();
const std::vector<std::vector<int>> PointPillars::kNumAnchorRo =
    Params::NumAnchorRo();
const std::vector<std::vector<float>> PointPillars::kAnchorRo =
    Params::AnchorRo();

PointPillars::PointPillars(const bool reproduce_result_mode,
                           const float score_threshold,
                           const float nms_overlap_threshold,
                           const std::string pfe_torch_file,
                           const std::string rpn_onnx_file)
    : reproduce_result_mode_(reproduce_result_mode),
      score_threshold_(score_threshold),
      nms_overlap_threshold_(nms_overlap_threshold),
      pfe_torch_file_(pfe_torch_file),
      rpn_onnx_file_(rpn_onnx_file) {
  if (reproduce_result_mode_) {
    preprocess_points_ptr_.reset(new PreprocessPoints(
        kMaxNumPillars, kMaxNumPointsPerPillar, kNumPointFeature, kGridXSize,
        kGridYSize, kGridZSize, kPillarXSize, kPillarYSize, kPillarZSize,
        kMinXRange, kMinYRange, kMinZRange, kNumIndsForScan));
  } else {
    preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
        kNumThreads, kMaxNumPillars, kMaxNumPointsPerPillar, kNumPointFeature,
        kNumIndsForScan, kGridXSize, kGridYSize, kGridZSize, kPillarXSize,
        kPillarYSize, kPillarZSize, kMinXRange, kMinYRange, kMinZRange));
  }

  anchor_mask_cuda_ptr_.reset(new AnchorMaskCuda(
      kNumThreads, kNumIndsForScan, kNumAnchor, kMinXRange, kMinYRange,
      kPillarXSize, kPillarYSize, kGridXSize, kGridYSize));

  scatter_cuda_ptr_.reset(
      new ScatterCuda(kNumThreads, kMaxNumPillars, kGridXSize, kGridYSize));

  const float float_min = std::numeric_limits<float>::lowest();
  const float float_max = std::numeric_limits<float>::max();
  postprocess_cuda_ptr_.reset(
      new PostprocessCuda(float_min, float_max, kNumAnchor, kNumClass,
                          score_threshold_, kNumThreads, nms_overlap_threshold_,
                          kNumBoxCorners, kNumOutputBoxFeature));

  DeviceMemoryMalloc();
  InitTorch();
  InitTRT();
  InitAnchors();
}

PointPillars::~PointPillars() {
  delete[] anchors_px_;
  delete[] anchors_py_;
  delete[] anchors_pz_;
  delete[] anchors_dx_;
  delete[] anchors_dy_;
  delete[] anchors_dz_;
  delete[] anchors_ro_;
  delete[] box_anchors_min_x_;
  delete[] box_anchors_min_y_;
  delete[] box_anchors_max_x_;
  delete[] box_anchors_max_y_;

  GPU_CHECK(cudaFree(dev_x_coors_));
  GPU_CHECK(cudaFree(dev_y_coors_));
  GPU_CHECK(cudaFree(dev_num_points_per_pillar_));
  GPU_CHECK(cudaFree(dev_sparse_pillar_map_));
  GPU_CHECK(cudaFree(dev_pillar_point_feature_));
  GPU_CHECK(cudaFree(dev_pillar_coors_));

  GPU_CHECK(cudaFree(dev_cumsum_along_x_));
  GPU_CHECK(cudaFree(dev_cumsum_along_y_));

  GPU_CHECK(cudaFree(dev_box_anchors_min_x_));
  GPU_CHECK(cudaFree(dev_box_anchors_min_y_));
  GPU_CHECK(cudaFree(dev_box_anchors_max_x_));
  GPU_CHECK(cudaFree(dev_box_anchors_max_y_));
  GPU_CHECK(cudaFree(dev_anchor_mask_));

  GPU_CHECK(cudaFree(pfe_buffers_[0]));
  GPU_CHECK(cudaFree(pfe_buffers_[1]));
  GPU_CHECK(cudaFree(pfe_buffers_[2]));

  GPU_CHECK(cudaFree(rpn_buffers_[0]));
  GPU_CHECK(cudaFree(rpn_buffers_[1]));
  GPU_CHECK(cudaFree(rpn_buffers_[2]));
  GPU_CHECK(cudaFree(rpn_buffers_[3]));

  GPU_CHECK(cudaFree(dev_scattered_feature_));

  GPU_CHECK(cudaFree(dev_anchors_px_));
  GPU_CHECK(cudaFree(dev_anchors_py_));
  GPU_CHECK(cudaFree(dev_anchors_pz_));
  GPU_CHECK(cudaFree(dev_anchors_dx_));
  GPU_CHECK(cudaFree(dev_anchors_dy_));
  GPU_CHECK(cudaFree(dev_anchors_dz_));
  GPU_CHECK(cudaFree(dev_anchors_ro_));
  GPU_CHECK(cudaFree(dev_filtered_box_));
  GPU_CHECK(cudaFree(dev_filtered_score_));
  GPU_CHECK(cudaFree(dev_filtered_label_));
  GPU_CHECK(cudaFree(dev_filtered_dir_));
  GPU_CHECK(cudaFree(dev_box_for_nms_));
  GPU_CHECK(cudaFree(dev_filter_count_));

  rpn_context_->destroy();
  rpn_runtime_->destroy();
  rpn_engine_->destroy();
}

void PointPillars::DeviceMemoryMalloc() {
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_x_coors_),
                       kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_y_coors_),
                       kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_num_points_per_pillar_),
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sparse_pillar_map_),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_point_feature_),
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_coors_),
                       kMaxNumPillars * 4 * sizeof(float)));

  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_cumsum_along_x_),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_cumsum_along_y_),
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  // for make anchor mask kernel
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_min_x_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_min_y_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_max_x_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_anchors_max_y_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchor_mask_),
                       kNumAnchor * sizeof(int)));

  // for trt inference
  // create GPU buffers and a stream
  GPU_CHECK(
      cudaMalloc(&pfe_buffers_[0], kMaxNumPillars * kMaxNumPointsPerPillar *
                                       kNumPointFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(&pfe_buffers_[1], kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMalloc(&pfe_buffers_[2], kMaxNumPillars * 4 * sizeof(float)));

  GPU_CHECK(cudaMalloc(&rpn_buffers_[0], kRpnInputSize * sizeof(float)));
  GPU_CHECK(cudaMalloc(&rpn_buffers_[1], kRpnBoxOutputSize * sizeof(float)));
  GPU_CHECK(cudaMalloc(&rpn_buffers_[2], kRpnClsOutputSize * sizeof(float)));
  GPU_CHECK(cudaMalloc(&rpn_buffers_[3], kRpnDirOutputSize * sizeof(float)));

  // for scatter kernel
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_scattered_feature_),
                       kNumThreads * kGridYSize * kGridXSize * sizeof(float)));

  // for filter
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_px_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_py_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_pz_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_dx_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_dy_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_dz_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_anchors_ro_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_box_),
                       kNumAnchor * kNumOutputBoxFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_score_),
                       kNumAnchor * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_label_),
                       kNumAnchor * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_filtered_dir_),
                       kNumAnchor * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_box_for_nms_),
                       kNumAnchor * kNumBoxCorners * sizeof(float)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_filter_count_), sizeof(int)));
}

void PointPillars::InitAnchors() {
  // allocate memory for anchors
  anchors_px_ = new float[kNumAnchor]();
  anchors_py_ = new float[kNumAnchor]();
  anchors_pz_ = new float[kNumAnchor]();
  anchors_dx_ = new float[kNumAnchor]();
  anchors_dy_ = new float[kNumAnchor]();
  anchors_dz_ = new float[kNumAnchor]();
  anchors_ro_ = new float[kNumAnchor]();
  box_anchors_min_x_ = new float[kNumAnchor]();
  box_anchors_min_y_ = new float[kNumAnchor]();
  box_anchors_max_x_ = new float[kNumAnchor]();
  box_anchors_max_y_ = new float[kNumAnchor]();
  // deallocate these memories in destructor

  GenerateAnchors(anchors_px_, anchors_py_, anchors_pz_, anchors_dx_,
                  anchors_dy_, anchors_dz_, anchors_ro_);

  ConvertAnchors2BoxAnchors(anchors_px_, anchors_py_, box_anchors_min_x_,
                            box_anchors_min_y_, box_anchors_max_x_,
                            box_anchors_max_y_);

  PutAnchorsInDeviceMemory();
}

void PointPillars::GenerateAnchors(float* anchors_px_, float* anchors_py_,
                                   float* anchors_pz_, float* anchors_dx_,
                                   float* anchors_dy_, float* anchors_dz_,
                                   float* anchors_ro_) {
  for (int i = 0; i < kNumAnchor; ++i) {
    anchors_px_[i] = 0;
    anchors_py_[i] = 0;
    anchors_pz_[i] = 0;
    anchors_dx_[i] = 0;
    anchors_dy_[i] = 0;
    anchors_dz_[i] = 0;
    anchors_ro_[i] = 0;
    box_anchors_min_x_[i] = 0;
    box_anchors_min_y_[i] = 0;
    box_anchors_max_x_[i] = 0;
    box_anchors_max_y_[i] = 0;
  }

  int ind = 0;
  for (size_t head = 0; head < kNumAnchorSets.size(); ++head) {
    float x_stride = kPillarXSize * kAnchorStrides[head];
    float y_stride = kPillarYSize * kAnchorStrides[head];
    int x_ind_start = kAnchorRanges[head * 4 + 0] / kAnchorStrides[head];
    int x_ind_end = kAnchorRanges[head * 4 + 1] / kAnchorStrides[head];
    int y_ind_start = kAnchorRanges[head * 4 + 2] / kAnchorStrides[head];
    int y_ind_end = kAnchorRanges[head * 4 + 3] / kAnchorStrides[head];
    // coors of first anchor's center
    float x_offset = kMinXRange + x_stride / 2.0;
    float y_offset = kMinYRange + y_stride / 2.0;

    std::vector<float> anchor_x_count, anchor_y_count;
    for (int i = x_ind_start; i < x_ind_end; ++i) {
      float anchor_coor_x = static_cast<float>(i) * x_stride + x_offset;
      anchor_x_count.push_back(anchor_coor_x);
    }
    for (int i = y_ind_start; i < y_ind_end; ++i) {
      float anchor_coor_y = static_cast<float>(i) * y_stride + y_offset;
      anchor_y_count.push_back(anchor_coor_y);
    }

    for (int y = 0; y < y_ind_end - y_ind_start; ++y) {
      for (int x = 0; x < x_ind_end - x_ind_start; ++x) {
        int ro_count = 0;
        for (size_t c = 0; c < kNumAnchorRo[head].size(); ++c) {
          for (int i = 0; i < kNumAnchorRo[head][c]; ++i) {
            anchors_px_[ind] = anchor_x_count[x];
            anchors_py_[ind] = anchor_y_count[y];
            anchors_ro_[ind] = kAnchorRo[head][ro_count];
            anchors_pz_[ind] = kAnchorZCoors[head][c];
            anchors_dx_[ind] = kAnchorDxSizes[head][c];
            anchors_dy_[ind] = kAnchorDySizes[head][c];
            anchors_dz_[ind] = kAnchorDzSizes[head][c];
            ro_count++;
            ind++;
          }
        }
      }
    }
  }
}

void PointPillars::PutAnchorsInDeviceMemory() {
  GPU_CHECK(cudaMemcpy(dev_box_anchors_min_x_, box_anchors_min_x_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_box_anchors_min_y_, box_anchors_min_y_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_box_anchors_max_x_, box_anchors_max_x_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_box_anchors_max_y_, box_anchors_max_y_,
                       kNumAnchor * sizeof(float), cudaMemcpyHostToDevice));

  GPU_CHECK(cudaMemcpy(dev_anchors_px_, anchors_px_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_py_, anchors_py_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_pz_, anchors_pz_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_dx_, anchors_dx_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_dy_, anchors_dy_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_dz_, anchors_dz_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_anchors_ro_, anchors_ro_, kNumAnchor * sizeof(float),
                       cudaMemcpyHostToDevice));
}

void PointPillars::ConvertAnchors2BoxAnchors(float* anchors_px,
                                             float* anchors_py,
                                             float* box_anchors_min_x_,
                                             float* box_anchors_min_y_,
                                             float* box_anchors_max_x_,
                                             float* box_anchors_max_y_) {
  // flipping box's dimension
  float flipped_anchors_dx[kNumAnchor] = {};
  float flipped_anchors_dy[kNumAnchor] = {};
  int ind = 0;
  for (size_t head = 0; head < kNumAnchorSets.size(); ++head) {
    int num_x_inds =
        (kAnchorRanges[head * 4 + 1] - kAnchorRanges[head * 4 + 0]) /
        kAnchorStrides[head];
    int num_y_inds =
        (kAnchorRanges[head * 4 + 3] - kAnchorRanges[head * 4 + 2]) /
        kAnchorStrides[head];
    int base_ind = ind;
    int ro_count = 0;
    for (int x = 0; x < num_x_inds; ++x) {
      for (int y = 0; y < num_y_inds; ++y) {
        ro_count = 0;
        for (size_t c = 0; c < kNumAnchorRo[head].size(); ++c) {
          for (int i = 0; i < kNumAnchorRo[head][c]; ++i) {
            if (kAnchorRo[head][ro_count] <= 0.78) {
              flipped_anchors_dx[base_ind] = kAnchorDxSizes[head][c];
              flipped_anchors_dy[base_ind] = kAnchorDySizes[head][c];
            } else {
              flipped_anchors_dx[base_ind] = kAnchorDySizes[head][c];
              flipped_anchors_dy[base_ind] = kAnchorDxSizes[head][c];
            }
            ro_count++;
            base_ind++;
          }
        }
      }
    }
    for (int x = 0; x < num_x_inds; ++x) {
      for (int y = 0; y < num_y_inds; ++y) {
        for (size_t i = 0; i < kAnchorRo[head].size(); ++i) {
          box_anchors_min_x_[ind] =
              anchors_px[ind] - flipped_anchors_dx[ind] / 2.0f;
          box_anchors_min_y_[ind] =
              anchors_py[ind] - flipped_anchors_dy[ind] / 2.0f;
          box_anchors_max_x_[ind] =
              anchors_px[ind] + flipped_anchors_dx[ind] / 2.0f;
          box_anchors_max_y_[ind] =
              anchors_py[ind] + flipped_anchors_dy[ind] / 2.0f;
          ind++;
        }
      }
    }
  }
}

void PointPillars::InitTorch() {
  if (gpu_id_ >= 0) {
    device_type_ = torch::kCUDA;
    device_id_ = gpu_id_;
  } else {
    device_type_ = torch::kCPU;
  }

  // Init torch net
  torch::Device device(device_type_, device_id_);
  pfe_net_ = torch::jit::load(pfe_torch_file_, device);
}

void PointPillars::InitTRT() {
  // create a TensorRT model from the onnx model and serialize it to a stream
  nvinfer1::IHostMemory* rpn_trt_model_stream{nullptr};
  OnnxToTRTModel(rpn_onnx_file_, &rpn_trt_model_stream);
  if (rpn_trt_model_stream == nullptr) {
    std::cerr << "Failed to load ONNX file " << std::endl;
  }

  // deserialize the engine
  rpn_runtime_ = nvinfer1::createInferRuntime(g_logger_);
  if (rpn_runtime_ == nullptr) {
    std::cerr << "Failed to create TensorRT Runtime object." << std::endl;
  }
  rpn_engine_ = rpn_runtime_->deserializeCudaEngine(
      rpn_trt_model_stream->data(), rpn_trt_model_stream->size(), nullptr);
  if (rpn_engine_ == nullptr) {
    std::cerr << "Failed to create TensorRT Engine." << std::endl;
  }
  rpn_trt_model_stream->destroy();
  rpn_context_ = rpn_engine_->createExecutionContext();
  if (rpn_context_ == nullptr) {
    std::cerr << "Failed to create TensorRT Execution Context." << std::endl;
  }
}

void PointPillars::OnnxToTRTModel(
    const std::string& model_file,  // name of the onnx model
    nvinfer1::IHostMemory**
        trt_model_stream) {  // output buffer for the TensorRT model
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);

  // create the builder
  const auto explicit_batch =
      static_cast<uint32_t>(kBatchSize) << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(g_logger_);
  nvinfer1::INetworkDefinition* network =
      builder->createNetworkV2(explicit_batch);

  auto parser = nvonnxparser::createParser(*network, g_logger_);
  if (!parser->parseFromFile(model_file.c_str(), verbosity)) {
    std::string msg("failed to parse onnx file");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);
  nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
  config->setMaxWorkspaceSize(1 << 20);
  nvinfer1::ICudaEngine* engine =
      builder->buildEngineWithConfig(*network, *config);

  parser->destroy();

  // serialize the engine, then close everything down
  *trt_model_stream = engine->serialize();
  engine->destroy();
  network->destroy();
  config->destroy();
  builder->destroy();
}

void PointPillars::PreprocessCPU(const float* in_points_array,
                                 const int in_num_points) {
  int x_coors[kMaxNumPillars] = {};
  int y_coors[kMaxNumPillars] = {};
  float num_points_per_pillar[kMaxNumPillars] = {};

  float* pillar_point_feature =
      new float[kMaxNumPillars * kMaxNumPointsPerPillar * kNumPointFeature];
  float* pillar_coors = new float[kMaxNumPillars * 4];
  float* sparse_pillar_map = new float[kNumIndsForScan * kNumIndsForScan];

  preprocess_points_ptr_->Preprocess(in_points_array, in_num_points, x_coors,
                                     y_coors, num_points_per_pillar,
                                     pillar_point_feature, pillar_coors,
                                     sparse_pillar_map, host_pillar_count_);

  GPU_CHECK(cudaMemset(dev_x_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_pillar_point_feature_, 0,
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float)));
  GPU_CHECK(
      cudaMemset(dev_pillar_coors_, 0, kMaxNumPillars * 4 * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0,
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0,
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  GPU_CHECK(cudaMemcpy(dev_x_coors_, x_coors, kMaxNumPillars * sizeof(int),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_y_coors_, y_coors, kMaxNumPillars * sizeof(int),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_point_feature_, pillar_point_feature,
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_coors_, pillar_coors,
                       kMaxNumPillars * 4 * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_num_points_per_pillar_, num_points_per_pillar,
                       kMaxNumPillars * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map_, sparse_pillar_map,
                       kNumIndsForScan * kNumIndsForScan * sizeof(float),
                       cudaMemcpyHostToDevice));

  delete[] pillar_point_feature;
  delete[] pillar_coors;
  delete[] sparse_pillar_map;
}

void PointPillars::PreprocessGPU(const float* in_points_array,
                                 const int in_num_points) {
  float* dev_points;
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_points),
                       in_num_points * kNumPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemcpy(dev_points, in_points_array,
                       in_num_points * kNumPointFeature * sizeof(float),
                       cudaMemcpyHostToDevice));

  GPU_CHECK(cudaMemset(dev_x_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0,
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_point_feature_, 0,
                       kMaxNumPillars * kMaxNumPointsPerPillar *
                           kNumPointFeature * sizeof(float)));
  GPU_CHECK(
      cudaMemset(dev_pillar_coors_, 0, kMaxNumPillars * 4 * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0,
                       kNumIndsForScan * kNumIndsForScan * sizeof(int)));
  host_pillar_count_[0] = 0;

  GPU_CHECK(cudaMemset(dev_anchor_mask_, 0, kNumAnchor * sizeof(int)));

  preprocess_points_cuda_ptr_->DoPreprocessPointsCuda(
      dev_points, in_num_points, dev_x_coors_, dev_y_coors_,
      dev_num_points_per_pillar_, dev_pillar_point_feature_, dev_pillar_coors_,
      dev_sparse_pillar_map_, host_pillar_count_);

  GPU_CHECK(cudaFree(dev_points));
}

void PointPillars::Preprocess(const float* in_points_array,
                              const int in_num_points) {
  if (reproduce_result_mode_) {
    PreprocessCPU(in_points_array, in_num_points);
  } else {
    PreprocessGPU(in_points_array, in_num_points);
  }
}

void PointPillars::DoInference(const float* in_points_array,
                               const int in_num_points,
                               std::vector<float>* out_detections,
                               std::vector<int>* out_labels) {
  if (device_id_ < 0) {
    AERROR << "Torch is not using GPU!";
    return;
  }

  Preprocess(in_points_array, in_num_points);
  anchor_mask_cuda_ptr_->DoAnchorMaskCuda(
      dev_sparse_pillar_map_, dev_cumsum_along_x_, dev_cumsum_along_y_,
      dev_box_anchors_min_x_, dev_box_anchors_min_y_, dev_box_anchors_max_x_,
      dev_box_anchors_max_y_, dev_anchor_mask_);

  cudaStream_t stream;
  GPU_CHECK(cudaStreamCreate(&stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[0], dev_pillar_point_feature_,
                            kMaxNumPillars * kMaxNumPointsPerPillar *
                                kNumPointFeature * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[1], dev_num_points_per_pillar_,
                            kMaxNumPillars * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[2], dev_pillar_coors_,
                            kMaxNumPillars * 4 * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  torch::Tensor tensor_pillar_point_feature = torch::from_blob(pfe_buffers_[0],
      {kMaxNumPillars, kMaxNumPointsPerPillar, kNumPointFeature},
      torch::kCUDA);
  torch::Tensor tensor_num_points_per_pillar = torch::from_blob(
      pfe_buffers_[1], {kMaxNumPillars}, torch::kCUDA);
  torch::Tensor tensor_pillar_coors = torch::from_blob(pfe_buffers_[2],
      {kMaxNumPillars, 4}, torch::kCUDA);
  torch::Device device(device_type_, device_id_);
  tensor_pillar_point_feature.to(device);
  tensor_num_points_per_pillar.to(device);
  tensor_pillar_coors.to(device);

//***********************************************************************
//bug:operation failed in the TorchScript interpreter
  auto pfe_output = pfe_net_.forward({tensor_pillar_point_feature,
                                      tensor_num_points_per_pillar,
                                      tensor_pillar_coors}).toTensor();

  GPU_CHECK(
      cudaMemset(dev_scattered_feature_, 0, kRpnInputSize * sizeof(float)));
  scatter_cuda_ptr_->DoScatterCuda(
      host_pillar_count_[0], dev_x_coors_, dev_y_coors_,
      pfe_output.data_ptr<float>(), dev_scattered_feature_);
  GPU_CHECK(cudaMemcpyAsync(rpn_buffers_[0], dev_scattered_feature_,
                            kBatchSize * kRpnInputSize * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  rpn_context_->enqueueV2(rpn_buffers_, stream, nullptr);
  GPU_CHECK(cudaMemset(dev_filter_count_, 0, sizeof(int)));
//bug:radix_sort: failed on 2nd step: cudaErrorInvalidValue: invalid argument
  postprocess_cuda_ptr_->DoPostprocessCuda(
      reinterpret_cast<float*>(rpn_buffers_[1]),
      reinterpret_cast<float*>(rpn_buffers_[2]),
      reinterpret_cast<float*>(rpn_buffers_[3]), dev_anchor_mask_,
      dev_anchors_px_, dev_anchors_py_, dev_anchors_pz_, dev_anchors_dx_,
      dev_anchors_dy_, dev_anchors_dz_, dev_anchors_ro_, dev_filtered_box_,
      dev_filtered_score_, dev_filtered_label_, dev_filtered_dir_,
      dev_box_for_nms_, dev_filter_count_, out_detections, out_labels);
//***************************************************************************

  // release the stream and the buffers
  cudaStreamDestroy(stream);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
