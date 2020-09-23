load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libwheel_angle.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":wheel_lib",
    ],
)

cc_library(
    name = "wheel_lib",
    srcs = ["wheel_angle.cc"],
    hdrs = ["wheel_angle.h"],
    copts = ["-DMODULE_NAME=\\\"wheel\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:uart",
        "//modules/common/util:message_util",
        "//modules/drivers/proto:wheelangle_cc_proto",
        "//modules/drivers/wheel/proto:device_conf_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)
