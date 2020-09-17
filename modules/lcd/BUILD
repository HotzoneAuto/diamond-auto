load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "liblcd.so",
    linkopts = ["-shared"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":lcd_lib",
    ],
)

cc_library(
    name="lcd_lib",
    srcs=["lcd_display.cc"],
    hdrs=["lcd_display.h"],
    copts=["-DMODULE_NAME=\\\"drivers\\\""],
    deps=[
        "//cyber",
        "//modules/common/util:uart",
        "//modules/lcd/proto:lcd_device_conf_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
    ],
)
