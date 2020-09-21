load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "motor_vol_up_node",
    srcs = ["motor_vol_up_node.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
	"//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "@com_github_gflags_gflags//:gflags",

    ],
)

cpplint()
