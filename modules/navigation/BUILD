load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])



cc_binary(
    name = "libnavigation_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":navigation_component_lib"],
)

cc_library(
    name = "navigation_component_lib",
    srcs = ["navigation_component.cc"],
    hdrs = ["navigation_component.h"],
    copts = ['-DMODULE_NAME=\\"navigation\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/drivers/proto:rfid_cc_proto",
        "//modules/navigation/proto:navigation_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)


cpplint()
