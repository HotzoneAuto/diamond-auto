load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "openzennode",
	hdrs = ["LPMS_component.h"],
    srcs = ["LPMS_component.cc"],
    deps = [
        ":ManagedThread",
        "//cyber",
        "//modules/drivers/proto:imu_cc_proto",
        "//modules/drivers/LPMS/proto:service_cc_proto",
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
