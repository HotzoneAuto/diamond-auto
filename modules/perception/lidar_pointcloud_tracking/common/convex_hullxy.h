#pragma once
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/qhull.h>


namespace apollo {
namespace perception {

template <typename PointInT>
class ConvexHull2DXY : public pcl::ConvexHull<PointInT> {
 public:
  typedef boost::shared_ptr<pcl::ConvexHull<PointInT>> Ptr;
  typedef boost::shared_ptr<const pcl::ConvexHull<PointInT>> ConstPtr;
  typedef pcl::PointCloud<PointInT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  using pcl::ConvexHull<PointInT>::reconstruct;
  using pcl::ConvexHull<PointInT>::compute_area_;
  using pcl::ConvexHull<PointInT>::total_area_;
  using pcl::ConvexHull<PointInT>::total_volume_;
  using pcl::ConvexHull<PointInT>::qhull_flags;

  ConvexHull2DXY() = default;
  virtual ~ConvexHull2DXY() = default;

  void Reconstruct2dxy(PointCloudPtr hull,
                       std::vector<pcl::Vertices> *polygons) {
    hull->header = input_->header;
    if (!initCompute() || input_->points.empty() || indices_->empty()) {
      hull->points.clear();
      return;
    }

    PerformReconstruction2dxy(hull, polygons, true);

    hull->width = static_cast<uint32_t>(hull->points.size());
    hull->height = 1;
    hull->is_dense = true;

    deinitCompute();
  }

  void PerformReconstruction2dxy(PointCloudPtr hull,
                                 std::vector<pcl::Vertices> *polygons,
                                 bool fill_polygon_data = false) {
    int dimension = 2;

    // True if qhull should free points in qh_freeqhull() or reallocation
    boolT ismalloc = True;
    // output from qh_produce_output(), use NULL to skip qh_produce_output()
    FILE *outfile = NULL;

#ifndef HAVE_QHULL_2011
    if (compute_area_) {
      outfile = stderr;
    }
#endif

    // option flags for qhull, see qh_opt.htm
    const char *flags = qhull_flags.c_str();
    // error messages from qhull code
    FILE *errfile = stderr;

    // Array of coordinates for each point
    coordT *points = reinterpret_cast<coordT *>(
        calloc(indices_->size() * dimension, sizeof(coordT)));
    if (points == NULL) {
      hull->points.resize(0);
      hull->width = hull->height = 0;
      polygons->resize(0);
      return;
    }

    // Build input data, using appropriate projection
    int j = 0;
    for (size_t i = 0; i < indices_->size(); ++i, j += dimension) {
      points[j + 0] = static_cast<coordT>(input_->points[(*indices_)[i]].x);
      points[j + 1] = static_cast<coordT>(input_->points[(*indices_)[i]].y);
    }

    // Compute convex hull
    int exitcode =
        qh_new_qhull(dimension, static_cast<int>(indices_->size()), points,
                     ismalloc, const_cast<char *>(flags), outfile, errfile);
#ifdef HAVE_QHULL_2011
    if (compute_area_) {
      qh_prepare_output();
    }
#endif

    // 0 if no error from qhull or it doesn't find any vertices
    if (exitcode != 0 || qh num_vertices == 0) {
      PCL_ERROR(
          "[pcl::%s::performReconstrution2D] "
          "ERROR: qhull was unable to compute "
          "a convex hull for the given point "
          "cloud (%lu)!\n",
          getClassName().c_str(), indices_->size());

      hull->points.resize(0);
      hull->width = hull->height = 0;
      polygons->resize(0);

      qh_freeqhull(!qh_ALL);
      int curlong, totlong;
      qh_memfreeshort(&curlong, &totlong);
      return;
    }

    // Qhull returns the area in volume for 2D
    if (compute_area_) {
      total_area_ = qh totvol;
      total_volume_ = 0.0;
    }

    int num_vertices = qh num_vertices;
    hull->points.resize(num_vertices);
    memset(&hull->points[0], static_cast<int>(hull->points.size()),
           sizeof(PointInT));

    vertexT *vertex;
    int i = 0;

    std::vector<std::pair<int, Eigen::Vector4f>,
                Eigen::aligned_allocator<std::pair<int, Eigen::Vector4f>>>
        idx_points(num_vertices);
    idx_points.resize(hull->points.size());
    memset(&idx_points[0], static_cast<int>(hull->points.size()),
           sizeof(std::pair<int, Eigen::Vector4f>));

    FORALLvertices {
      hull->points[i] = input_->points[(*indices_)[qh_pointid(vertex->point)]];
      idx_points[i].first = qh_pointid(vertex->point);
      ++i;
    }

    // Sort
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hull, centroid);
    for (size_t j = 0; j < hull->points.size(); j++) {
      idx_points[j].second[0] = hull->points[j].x - centroid[0];
      idx_points[j].second[1] = hull->points[j].y - centroid[1];
    }

    std::sort(idx_points.begin(), idx_points.end(), pcl::comparePoints2D);

    polygons->resize(1);
    (*polygons)[0].vertices.resize(hull->points.size());

    for (int j = 0; j < static_cast<int>(hull->points.size()); j++) {
      hull->points[j] = input_->points[(*indices_)[idx_points[j].first]];
      (*polygons)[0].vertices[j] = static_cast<unsigned int>(j);
    }

    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(&curlong, &totlong);

    hull->width = static_cast<uint32_t>(hull->points.size());
    hull->height = 1;
    hull->is_dense = false;
    return;
  }

  std::string getClassName() const { return ("ConvexHull2DXY"); }

 protected:
  using pcl::PCLBase<PointInT>::input_;
  using pcl::PCLBase<PointInT>::indices_;
  using pcl::PCLBase<PointInT>::initCompute;
  using pcl::PCLBase<PointInT>::deinitCompute;
};

}  // namespace perception
}  // namespace apollo

