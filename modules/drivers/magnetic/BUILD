load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "libmagnetic.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":magnetic_lib",
    ],
)

cc_library(
    name = "magnetic_lib",
    srcs = ["magnetic_component.cc"],
    hdrs = ["magnetic_component.h"],
    copts = ["-DMODULE_NAME=\\\"drivers\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:uart",
        "//modules/drivers/proto:magnetic_cc_proto",
        "//modules/drivers/magnetic/proto:magnetic_device_conf_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)


cc_binary(
    name = "read",
    srcs = ["read.cc"],
    deps = [
        "//cyber",
        "//modules/common/util:uart",
        "//modules/drivers/proto:magnetic_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "magnetic",
    srcs = ["magnetic.h"],
    deps = [
        # "//modules/common/util:uart",
    ],
)