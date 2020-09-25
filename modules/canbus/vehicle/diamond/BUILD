load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "diamond_vehicle_factory",
    srcs = [
        "diamond_vehicle_factory.cc",
    ],
    hdrs = [
        "diamond_vehicle_factory.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":diamond_controller",
        ":diamond_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "diamond_message_manager",
    srcs = [
        "diamond_message_manager.cc",
    ],
    hdrs = [
        "diamond_message_manager.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/diamond/protocol:canbus_diamond_protocol",
    ],
)

cc_library(
    name = "diamond_controller",
    srcs = [
        "diamond_controller.cc",
    ],
    hdrs = [
        "diamond_controller.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":diamond_message_manager",
        "//cyber",
        "//modules/common/util:uart",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/magnetic:magnetic",
        "//modules/drivers/proto:wheelangle_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/diamond/protocol:canbus_diamond_protocol",
        "//modules/drivers/proto:parking_cc_proto",
    ],
)

cpplint()
