load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ipcamera_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":ipcamera_component_lib"],
)

cc_library(
    name = "ipcamera_component_lib",
    srcs = ["ipcamera_component.cc"],
    hdrs = ["ipcamera_component.h",
            "ipcamera_main.h"],
    deps = [
        "//cyber",
        "//modules/drivers/ipcamera/proto:ipcamera_conf_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//cyber/examples/proto:examples_cc_proto",
        "@ipcamera",
        "@opencv",
    ],
)

cpplint()
