#pragma once

// headers in STL
#include <stdio.h>

// headers in CUDA
#include <cuda_runtime_api.h>

namespace apollo {
namespace perception {
namespace lidar {

// using MACRO to allocate memory inside CUDA kernel
#define NUM_3D_BOX_CORNERS_MACRO 8
#define NUM_2D_BOX_CORNERS_MACRO 4
#define NUM_THREADS_MACRO 64  // need to be changed when num_threads_ is changed

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

#define GPU_CHECK(ans) \
  { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char* file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort) exit(code);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
