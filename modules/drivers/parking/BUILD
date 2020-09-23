load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libparking_brake.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":parking_brake_lib",
    ],
)

cc_library(
    name="parking_brake_lib",
    srcs=["parking_brake.cc"],
    hdrs=["parking_brake.h"],
    copts=["-DMODULE_NAME=\\\"parking_brake\\\""],
    deps=[
        "//cyber",
        "//modules/common/util:uart",
        "//modules/drivers/parking/proto:parking_brake_conf_cc_proto",
    ],
)
