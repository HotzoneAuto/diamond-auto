load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "streamRetrieve_lib",
    srcs = ["streamRetrieve.cc"],
    hdrs = ["StreamRetrieve.h"],
    deps = [
        "@opencv",
        "@ipcamera",
    ],
)

cc_binary(
    name = "ipcamera_main",
    srcs = ["ipcamera_main.cc"],
    deps = [
        ":streamRetrieve_lib",
        #"@ipcamera",
    ],
)

cc_library(
    name = "ipcamera_component_lib",
    srcs = ["ipcamera_component.cc"],
    hdrs = ["ipcamera_component.h"],
    copts = ['-DMODULE_NAME=\\"ipcamera\\"'],
    deps = [
        "//cyber",
    ],
)

cpplint()
