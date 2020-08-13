load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
