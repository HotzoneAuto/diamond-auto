load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lcd",
    srcs = ["lcd_display.cc"],
    deps = [
        "//cyber",
        "//modules/common/util:uart",
        "//modules/canbus/proto:chassis_cc_proto",
    ],
)
cpplint()