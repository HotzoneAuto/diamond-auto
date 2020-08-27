load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])



cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_component_lib"],
)

cc_library(
    name = "control_component_lib",
    srcs = ["demo_control_component.cc"],
    hdrs = ["demo_control_component.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        ":control_lib",
        ":pid",
    ],
)

cc_library(
    name = "control_lib",
    srcs = [],
    hdrs = ["control_wheel_angle_real.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/drivers/proto:magnetic_cc_proto",
        "//modules/drivers/proto:rfid_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "pid",
    srcs = [],
    hdrs = ["diamondauto_control_pid.h"],
)



cpplint()
