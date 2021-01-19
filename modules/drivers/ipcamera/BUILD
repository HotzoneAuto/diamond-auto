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
    hdrs = ["ipcamera_component.h"],
    copts = ["-DMODULE_NAME=\\\"ipcamera\\\""],
    deps = [
        ":ipcamera_main_lib",
        "//cyber",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/drivers/ipcamera/proto:ipcamera_conf_cc_proto",
        "@opencv",
    ],
)
cc_library(
    name = "ipcamera_main_lib",
    srcs = ["ipcamera_main.cc"],
    hdrs = ["ipcamera_main.h"],
    deps = [
        "@ipcamera",
    ],
)

cc_binary(
    name = "ipcamera_reader.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":ipcamera_reader_lib",
    ],
)

cc_library(
    name = "ipcamera_reader_lib",
    srcs = ["ipcamera_reader.cc"],
    hdrs = ["ipcamera_reader.h"],
    copts = ["-DMODULE_NAME=\\\"ipcamera_reader\\\""],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/drivers/ipcamera/proto:ipcamera_conf_cc_proto",
        "@opencv",
    ],
)

cpplint()
