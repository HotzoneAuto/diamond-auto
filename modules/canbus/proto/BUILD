load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "canbus_proto",
    deps = [
        ":canbus_proto_lib",
    ],
)

proto_library(
    name = "canbus_proto_lib",
    srcs = [
        "canbus_conf.proto",
        "chassis.proto",
        "chassis_detail.proto",
        "vehicle_parameter.proto",
        "diamond.proto",
    ],
    deps = [
        "//modules/common/configs/proto:vehicle_config_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)

py_proto(
    name = "chassis_detail_pb2",
    src = "chassis_detail.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/configs/proto:vehicle_config_pb2",
    ],
)



py_proto(
    name = "chassis_pb2",
    src = "chassis.proto",
    deps = [
        "//modules/common/configs/proto:vehicle_config_pb2",
        "//modules/common/proto:drive_state_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:vehicle_signal_pb2",
    ],
)

py_proto(
    name = "canbus_conf_pb2",
    src = "canbus_conf.proto",
    deps = [
        "//modules/canbus/proto:vehicle_parameter_pb2",
        "//modules/drivers/canbus/proto:can_card_parameter_pb2",
    ],
)

py_proto(
    name = "diamond_pb2",
    src = "diamond.proto",
)

py_proto(
    name = "vehicle_parameter_pb2",
    src = "vehicle_parameter.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/configs/proto:vehicle_config_pb2",
    ],
)
