load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_gflags",
    srcs = ["control_gflags.cc"],
    hdrs = ["control_gflags.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)
