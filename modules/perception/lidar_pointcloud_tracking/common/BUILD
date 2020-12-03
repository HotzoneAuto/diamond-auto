load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object",
    srcs = ["object.cc"],
    hdrs = ["object.h"],
    deps = [
        ":pcl_types",
        "@eigen",
    ],
)

cc_library(
    name = "tracker_config",
    hdrs = ["tracker_config.h"],
)

cc_library(
    name = "base_object_builder",
    hdrs = ["base_object_builder.h"],
    deps = [
        ":object",
        ":pcl_types",
        ":geometry_util",
    ]
)

cc_library(
    name = "pcl_types",
    hdrs = ["pcl_types.h"],
    deps = [
        "@pcl",
    ],
)

cc_library(
    name = "convex_hullxy",
    hdrs = ["convex_hullxy.h"],
    deps = [
        "@pcl",
    ],
)

cc_library(
    name = "graph_util",
    srcs = ["graph_util.cc"],
    hdrs = ["graph_util.h"],
)

cc_library(
    name = "geometry_util",
    srcs = ["geometry_util.cc"],
    hdrs = ["geometry_util.h"],
    deps = [
        ":pcl_types",
        "@eigen",
        "@pcl",
        "@local_config_vtk//:vtk",
    ],
)

cc_library(
    name = "hungarian_bigraph_matcher",
    srcs = ["hungarian_bigraph_matcher.cc"],
    hdrs = ["hungarian_bigraph_matcher.h"],
)

cpplint()
