#include "modules/perception/lidar_pointcloud_tracking/min_box.h"

namespace apollo {
namespace perception {

using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;

const float EPSILON = 1e-6;

bool MinBoxObjectBuilder::Build(const ObjectBuilderOptions& options,
                                std::vector<std::shared_ptr<Object>>* objects) {
  if (objects == nullptr) {
    return false;
  }

  for (size_t i = 0; i < objects->size(); ++i) {
    if ((*objects)[i]) {
      (*objects)[i]->id = i;
      BuildObject(options, (*objects)[i]);
    }
  }

  return true;
}

double MinBoxObjectBuilder::ComputeAreaAlongOneEdge(
    std::shared_ptr<Object> obj, size_t first_in_point, Eigen::Vector3d* center,
    double* lenth, double* width, Eigen::Vector3d* dir) {
  std::vector<Eigen::Vector3d> ns;
  Eigen::Vector3d v(0.0, 0.0, 0.0);
  Eigen::Vector3d vn(0.0, 0.0, 0.0);
  Eigen::Vector3d n(0.0, 0.0, 0.0);
  double len = 0;
  double wid = 0;
  size_t index = (first_in_point + 1) % obj->polygon.points.size();
  for (size_t i = 0; i < obj->polygon.points.size(); ++i) {
    if (i != first_in_point && i != index) {
      // compute v
      Eigen::Vector3d o(0.0, 0.0, 0.0);
      Eigen::Vector3d a(0.0, 0.0, 0.0);
      Eigen::Vector3d b(0.0, 0.0, 0.0);
      o[0] = obj->polygon.points[i].x;
      o[1] = obj->polygon.points[i].y;
      o[2] = 0;
      b[0] = obj->polygon.points[first_in_point].x;
      b[1] = obj->polygon.points[first_in_point].y;
      b[2] = 0;
      a[0] = obj->polygon.points[index].x;
      a[1] = obj->polygon.points[index].y;
      a[2] = 0;
      double k =
          ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
      k = k / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
      k = k * -1;
      // n is pedal of src
      n[0] = (b[0] - a[0]) * k + a[0];
      n[1] = (b[1] - a[1]) * k + a[1];
      n[2] = 0;
      // compute height from src to line
      Eigen::Vector3d edge1 = o - b;
      Eigen::Vector3d edge2 = a - b;
      // cross product
      double height = fabs(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
      height = height / sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1]);
      if (height > wid) {
        wid = height;
        v = o;
        vn = n;
      }
    } else {
      n[0] = obj->polygon.points[i].x;
      n[1] = obj->polygon.points[i].y;
      n[2] = 0;
    }
    ns.push_back(n);
  }
  size_t point_num1 = 0;
  size_t point_num2 = 0;
  for (size_t i = 0; i < ns.size() - 1; ++i) {
    Eigen::Vector3d p1 = ns[i];
    for (size_t j = i + 1; j < ns.size(); ++j) {
      Eigen::Vector3d p2 = ns[j];
      double dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                         (p1[1] - p2[1]) * (p1[1] - p2[1]));
      if (dist > len) {
        len = dist;
        point_num1 = i;
        point_num2 = j;
      }
    }
  }
  Eigen::Vector3d vp1 = v + ns[point_num1] - vn;
  Eigen::Vector3d vp2 = v + ns[point_num2] - vn;
  (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
  (*center)[2] = obj->polygon.points[0].z;
  obj->vertex1 = vp1;
  obj->vertex2 = vp2;
  obj->vertex3 = ns[point_num1];
  obj->vertex4 = ns[point_num2];
  if (len > wid) {
    *dir = ns[point_num2] - ns[point_num1];
  } else {
    *dir = vp1 - ns[point_num1];
  }
  *lenth = len > wid ? len : wid;
  *width = len > wid ? wid : len;
  return (*lenth) * (*width);
}

void MinBoxObjectBuilder::ReconstructPolygon(const Eigen::Vector3d& ref_ct,
                                             std::shared_ptr<Object> obj) {
  if (obj->polygon.points.size() <= 0) {
    return;
  }
  size_t max_point_index = 0;
  size_t min_point_index = 0;
  Eigen::Vector3d p;
  p[0] = obj->polygon.points[0].x;
  p[1] = obj->polygon.points[0].y;
  p[2] = obj->polygon.points[0].z;
  Eigen::Vector3d max_point = p - ref_ct;//ref_ct初始化为０，也就是坐标原点，即lidar中心位置
  Eigen::Vector3d min_point = p - ref_ct;
  for (size_t i = 1; i < obj->polygon.points.size(); ++i) {
    Eigen::Vector3d p;
    p[0] = obj->polygon.points[i].x;
    p[1] = obj->polygon.points[i].y;
    p[2] = obj->polygon.points[i].z;
    Eigen::Vector3d ray = p - ref_ct;
    // clock direction           顺时针
    if (max_point[0] * ray[1] - ray[0] * max_point[1] < EPSILON) {//原点与序号x的点向量依次与序号x+1到原点的向量做数量积
      max_point = ray;                                            //找到最右边的点
      max_point_index = i;
    }
    // unclock direction
    if (min_point[0] * ray[1] - ray[0] * min_point[1] > EPSILON) {
      min_point = ray;
      min_point_index = i;                                 //找到最左边的点
    }
  }
  //以下代码为筛选有效边长,如果相邻的两个点在line的后面,则因为遮挡原因,视这条边为无效边
  Eigen::Vector3d line = max_point - min_point;        //以最坐边和最右边的两个点画一条线
  double total_len = 0;
  double max_dis = 0;
  bool has_out = false;
  for (size_t i = min_point_index, count = 0;
       count < obj->polygon.points.size();
       i = (i + 1) % obj->polygon.points.size(), ++count) {
    Eigen::Vector3d p_x;
    p_x[0] = obj->polygon.points[i].x;
    p_x[1] = obj->polygon.points[i].y;
    p_x[2] = obj->polygon.points[i].z;
    size_t j = (i + 1) % obj->polygon.points.size();
    if (j != min_point_index && j != max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {    //j点在line的靠近雷达一侧
        double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                           (p[1] - p_x[1]) * (p[1] - p_x[1]));
        total_len += dist;
        if (dist - max_dis > EPSILON) {
          max_dis = dist;
        }
      } else {
        // outline
        has_out = true;
      }
    } else if ((i == min_point_index && j == max_point_index) ||
               (i == max_point_index && j == min_point_index)) {
      size_t k = (j + 1) % obj->polygon.points.size();
      Eigen::Vector3d p_k;
      p_k[0] = obj->polygon.points[k].x;
      p_k[1] = obj->polygon.points[k].y;
      p_k[2] = obj->polygon.points[k].z;
      Eigen::Vector3d p_j;
      p_j[0] = obj->polygon.points[j].x;
      p_j[1] = obj->polygon.points[j].y;
      p_j[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
      } else {
        // outline
        has_out = true;
      }
    } else if (j == min_point_index || j == max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p_x - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {
        double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                           (p[1] - p_x[1]) * (p[1] - p_x[1]));
        total_len += dist;
        if (dist > max_dis) {
          max_dis = dist;
        }
      } else {
        // outline
        has_out = true;
      }
    }
  }
  //截止这里,有效边筛选结束
  size_t count = 0;
  double min_area = std::numeric_limits<double>::max();
  for (size_t i = min_point_index; count < obj->polygon.points.size();
       i = (i + 1) % obj->polygon.points.size(), ++count) {
    Eigen::Vector3d p_x;
    p_x[0] = obj->polygon.points[i].x;
    p_x[1] = obj->polygon.points[i].y;
    p_x[2] = obj->polygon.points[i].z;
    size_t j = (i + 1) % obj->polygon.points.size();
    Eigen::Vector3d p_j;
    p_j[0] = obj->polygon.points[j].x;
    p_j[1] = obj->polygon.points[j].y;
    p_j[2] = obj->polygon.points[j].z;
    double dist = sqrt((p_x[0] - p_j[0]) * (p_x[0] - p_j[0]) +
                       (p_x[1] - p_j[1]) * (p_x[1] - p_j[1]));
    if (dist < max_dis && (dist / total_len) < 0.5) {
      continue;
    }
    if (j != min_point_index && j != max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
        Eigen::Vector3d center;
        double length = 0;
        double width = 0;
        Eigen::Vector3d dir;
        double area =
            ComputeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
        if (area < min_area) {
          obj->center = center;
          obj->length = length;
          obj->width = width;
          obj->direction = dir;
          min_area = area;
        }
      } else {
        // outline
      }
    } else if ((i == min_point_index && j == max_point_index) ||
               (i == max_point_index && j == min_point_index)) {
      if (!has_out) {
        continue;
      }
      Eigen::Vector3d center;
      double length = 0;
      double width = 0;
      Eigen::Vector3d dir;
      double area =
          ComputeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
      if (area < min_area) {
        obj->center = center;
        obj->length = length;
        obj->width = width;
        obj->direction = dir;
        min_area = area;
      }
    } else if (j == min_point_index || j == max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[i].x;
      p[1] = obj->polygon.points[i].y;
      p[2] = obj->polygon.points[i].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
        Eigen::Vector3d center;
        double length = 0.0;
        double width = 0.0;
        Eigen::Vector3d dir;
        double area =
            ComputeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
        if (area < min_area) {
          obj->center = center;
          obj->length = length;
          obj->width = width;
          obj->direction = dir;
          min_area = area;
        }
      } else {
        // outline
      }
    }
  }
  obj->direction.normalize();
}

void MinBoxObjectBuilder::ComputePolygon2dxy(std::shared_ptr<Object> obj) {
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl_util::PointCloudPtr cloud = obj->cloud;//存放目标点云
  SetDefaultValue(cloud, obj, &min_pt, &max_pt);//设置默认值
  if (cloud->points.size() < 4u) {                   //点云个数小于4个时,返回
    return;
  }
  GetCloudMinMax3D<pcl_util::Point>(cloud, &min_pt, &max_pt);//为啥又get一遍?zz吧 SetDefaultValue
  obj->height = static_cast<double>(max_pt[2]) - static_cast<double>(min_pt[2]);//SetDefaultValue这个里面不是算过啦?又强制float转换成了double
  // 以下这段代码并没有什么用，不知道为什么加上
  // const double min_eps = 10 * std::numeric_limits<double>::epsilon();//10*最小非零浮点数?是在干吗?

  // const double diff_x = cloud->points[1].x - cloud->points[0].x;
  // const double diff_y = cloud->points[1].y - cloud->points[0].y;
  // size_t idx = 0;
  // for (idx = 2; idx < cloud->points.size(); ++idx) {
  //   const double tdiff_x = cloud->points[idx].x - cloud->points[0].x;
  //   const double tdiff_y = cloud->points[idx].y - cloud->points[0].y;
  //   if ((diff_x * tdiff_y - tdiff_x * diff_y) > min_eps) {    //两个向量的X乘,向量10Xx0,如果点x在10的右边,则<0结束
  //     break;
  //   }
  // }
  // if (idx >= cloud->points.size()) {
  //   cloud->points[0].x += min_eps;
  //   cloud->points[0].y += min_eps;
  //   cloud->points[1].x -= min_eps;
  // }

  obj->min_height = min_pt[2];
  obj->max_height = max_pt[2];   //hrn 19.11.20

  PointCloudPtr pcd_xy(new PointCloud);     //将点云投影到地平面上,z坐标全部换成目标障碍物的最小z坐标
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl_util::Point p = cloud->points[i];
    p.z = min_pt[2];
    pcd_xy->push_back(p);
  }

  ConvexHull2DXY<pcl_util::Point> hull;    //自定义的求凸包的类
  hull.setInputCloud(pcd_xy);
  hull.setDimension(2);
  std::vector<pcl::Vertices> poly_vt;
  PointCloudPtr plane_hull(new PointCloud);
  hull.Reconstruct2dxy (plane_hull, &poly_vt);   //凸包点云存放在plane_hull中,poly_vt中的Vertices存放一组点的索引，索引是plane_hull中的点对应的索引


  if (poly_vt.size() == 1u) {                           //
    std::vector<int> ind(poly_vt[0].vertices.begin(),    //将poly_vt[0].vertices数组中的数据全部复制到ind中
                         poly_vt[0].vertices.end());
    TransformPointCloud(plane_hull, ind, &obj->polygon);
  } else {
    obj->polygon.points.resize(4);                               //poly_vt.size()不为1 的情况下,将minmax3d的四个顶点作为凸包
    obj->polygon.points[0].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[0].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[0].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[1].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[1].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[1].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[2].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[2].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[2].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[3].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[3].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[3].z = static_cast<double>(min_pt[2]);
  }
}

void MinBoxObjectBuilder::ComputeGeometricFeature(const Eigen::Vector3d& ref_ct,
                                                  std::shared_ptr<Object> obj) {
  // step 1: compute 2D xy plane's polygen
  ComputePolygon2dxy(obj);
  // step 2: construct box
  ReconstructPolygon(ref_ct, obj);
}

void MinBoxObjectBuilder::BuildObject(ObjectBuilderOptions options,
                                      std::shared_ptr<Object> object) {
  ComputeGeometricFeature(options.ref_center, object);
}

void MinBoxObjectBuilder::TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const std::vector<int>& indices,
                         pcl_util::PointDCloud* trans_cloud) {
  if (trans_cloud->size() != indices.size()) {
    trans_cloud->resize(indices.size());
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    const pcl_util::Point& p = cloud->at(indices[i]);
    Eigen::Vector3d v(p.x, p.y, p.z);
    pcl_util::PointD& tp = trans_cloud->at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    //tp.intensity = p.intensity;
  }
}

}  // namespace perception
}  // namespace apollo
