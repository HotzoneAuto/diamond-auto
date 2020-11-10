load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "canbus_tester",
    srcs = ["canbus_tester.cc"],
    deps = [
        "//cyber",
        "//modules/canbus:canbus_lib",
        "//modules/canbus/common:canbus_common",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:message_util",
        "//modules/control/proto:control_cmd_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "motor_vol_down_node",
    srcs = ["motor_vol_down.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/vehicle:vehicle_factory",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "@com_github_gflags_gflags//:gflags",

    ],
)

cc_binary(
    name = "motor_vol_up_node",
    srcs = ["motor_vol_up.cc"],
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

cc_binary(
    name = "xbox_one",
    srcs = ["xbox_one.cc"],
    copts = ["-DMODULE_NAME=\\\"xbox_one\\\""],
    deps = [
        "//cyber",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/common/adapters:adapter_gflags",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:message_util",
	"@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
