load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librfid.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":rfid_lib",
    ],
)

cc_library(
    name = "rfid_lib",
    srcs = ["rfid_component.cc"],
    hdrs = ["rfid_component.h"],
    copts = ["-DMODULE_NAME=\\\"drivers\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:uart",
        "//modules/drivers/proto:rfid_cc_proto",
        "//modules/drivers/rfid/proto:rfid_device_conf_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)



cc_binary(
    name = "rfid",
    srcs = ["example.cc"],
    deps = [
        "//cyber",
        "//modules/common/util:uart",
        "//modules/drivers/proto:rfid_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)
