load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "openzennode",
    srcs = ["OpenZenNode.cc"],
    deps = [
        ":ManagedThread",
        "//cyber",
        "//modules/drivers/proto:imu_cc_proto",
        "//modules/drivers/imu/proto:service_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
        "@openzen",
    ],
)


cc_binary(
    name = "libimu.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":imu_component_lib"],
)

cc_library(
    name = "imu_component_lib",
    srcs = ["imu_component.cc"],
    hdrs = ["imu_component.h"],
    copts = ['-DMODULE_NAME=\\"LPMS\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/proto:imu_cc_proto",
        "//modules/drivers/imu/proto:device_config_cc_proto",
        "//modules/common/util:message_util",
        "@openzen",
    ],
)


cc_library(
    name = "ManagedThread",
    hdrs = ["ManagedThread.h"],
    deps = [
    ],
)

cc_binary(
    name = "checker",
    srcs = ["checker.cc"],
    deps = [
        "@openzen",
    ],
)



cpplint()
