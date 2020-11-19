load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "streamRetrieve",
    srcs = ["streamRetrieve.cc"],
    hdrs = ["StreamRetrieve.h"],
    deps = [
        "@opencv",
        #"@ipcamera",
    ],
)

#cc_library(
#    name = "ipcamera_component_lib",
#    srcs = ["ipcamera_component.cc"],
#    hdrs = ["ipcamera_component.h"],
#    copts = ['-DMODULE_NAME=\\"ipcamera\\"'],
#    deps = [
#        "//cyber",
#    ],
#)

#cc_library(
#    name = "compress_component_lib",
#    srcs = ["compress_component.cc"],
#    hdrs = ["compress_component.h"],
#    copts = ['-DMODULE_NAME=\\"camera\\"'],
#    deps = [
#        "//cyber",
#        "//modules/common/proto:error_code_cc_proto",
#        "//modules/common/proto:header_cc_proto",
#        "@opencv",
#    ],
#)

cpplint()
