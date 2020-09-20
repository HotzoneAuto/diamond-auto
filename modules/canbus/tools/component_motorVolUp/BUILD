load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "mortor_vol_up.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":mortor_vol_up_lib"],
)

cc_library(
    name = "mortor_vol_up_lib",
    srcs = ["mortor_vol_up.cc"],
    hdrs = ["mortor_vol_up.h"],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
	"//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_github_gflags_gflags//:gflags",

    ],
)

cpplint()
