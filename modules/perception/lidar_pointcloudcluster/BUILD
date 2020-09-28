load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

 cc_binary(
     name = "lidar_pointcloudcluster.so",
     linkopts = ["-shared"],
     linkstatic = False,
     deps = [":lidar_pointcloudcluster_lib"],
 )

 cc_library(
     name = "lidar_pointcloudcluster_lib",
     srcs = ["lidar_pointcloudcluster.cc"],
     hdrs = ["lidar_pointcloudcluster.h"],
     deps = [
         "//cyber",
         "//modules/drivers/proto:pointcloud_cc_proto",
         "//modules/perception/proto:obst_box_cc_proto",
         "@eigen",
 	    "@pcl",
 	    "@local_config_vtk//:vtk",
 	    "@boost",
    ],
 )

cpplint()
