load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "canbus_component_stop.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":canbus_component_stop_lib"],
)

cc_library(
    name = "canbus_component_stop_lib",
    srcs = ["canbus_component_stop.cc"],
    hdrs = ["canbus_component_stop.h"],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "@com_github_gflags_gflags//:gflags",

    ],
)

cpplint()
