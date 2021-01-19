load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracked_object",
    srcs = ["tracked_object.cc"],
    hdrs = ["tracked_object.h"],
    deps = [
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "//modules/perception/lidar_pointcloud_tracking/common:geometry_util",
    ],
)

cc_library(
    name = "base_filter",
    hdrs = ["base_filter.h"],
    deps = [
        ":tracked_object",
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "@eigen",
    ],
)

cc_library(
    name = "log",
    hdrs = ["log.h"],
)

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    deps = [
        ":base_filter",
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "//modules/perception/lidar_pointcloud_tracking/common:geometry_util",
    ],
)

cc_library(
    name = "object_track",
    srcs = ["object_track.cc"],
    hdrs = ["object_track.h"],
    deps = [
        ":tracked_object",
        ":base_filter",
        ":kalman_filter",
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "@eigen",
        "@boost",
    ],
)

cc_library(
    name = "track_object_distance",
    srcs = ["track_object_distance.cc"],
    hdrs = ["track_object_distance.h"],
    deps = [
        ":tracked_object",
        ":object_track",
        "//modules/perception/lidar_pointcloud_tracking/common:geometry_util",
        "@eigen",
    ],
)

cc_library(
    name = "base_matcher",
    hdrs = ["base_matcher.h"],
    deps = [
        ":object_track",
        ":tracked_object",
        "@eigen",
    ],
)

cc_library(
    name = "base_tracker",
    hdrs = ["base_tracker.h"],
    deps = [
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "//modules/perception/lidar_pointcloud_tracking/common:pcl_types",
    ],
)

cc_library(
    name = "hungarian_matcher",
    srcs = ["hungarian_matcher.cc"],
    hdrs = ["hungarian_matcher.h"],
    deps = [
        ":base_matcher",
        "//modules/perception/lidar_pointcloud_tracking/common:geometry_util",
        "//modules/perception/lidar_pointcloud_tracking/common:graph_util",
        "//modules/perception/lidar_pointcloud_tracking/common:hungarian_bigraph_matcher",
        ":track_object_distance"
    ],
)

cc_library(
    name = "feature_descriptor",
    srcs = ["feature_descriptor.cc"],
    hdrs = ["feature_descriptor.h"],
    deps = [
        "//modules/perception/lidar_pointcloud_tracking/common:pcl_types",
        "@eigen",
    ],
)

cc_library(
    name = "hm_tracker",
    srcs = ["hm_tracker.cc"],
    hdrs = ["hm_tracker.h"],
    deps = [
        "//cyber",
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "//modules/perception/lidar_pointcloud_tracking/common:tracker_config",
        "//modules/perception/lidar_pointcloud_tracking/common:geometry_util",
        "//modules/perception/proto:tracker_config_cc_proto",
        ":base_tracker",
        ":base_matcher",
        ":object_track",
        ":tracked_object",
        ":kalman_filter",
        ":hungarian_matcher",
        ":track_object_distance",
        ":feature_descriptor",
        ":log",
    ],
)

cc_library(
    name = "min_box",
    srcs = ["min_box.cc"],
    hdrs = ["min_box.h"],
    deps = [
        "//modules/perception/lidar_pointcloud_tracking/common:convex_hullxy",
        "//modules/perception/lidar_pointcloud_tracking/common:pcl_types",
        "//modules/perception/lidar_pointcloud_tracking/common:object",
        "//modules/perception/lidar_pointcloud_tracking/common:base_object_builder",
        "@pcl",
        "@eigen",
    ],
)

cpplint()
