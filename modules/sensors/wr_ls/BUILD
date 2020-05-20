package(default_visibility = ["//visibility:public"])


cc_library(
    name = "parser_base",
    srcs = ["parser_base.cpp"],
    hdrs = [
        "parser_base.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        # "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cc_library(
    name = "wr_ls_constants",
    # srcs = ["wr_ls_constants.cc"],
    hdrs = [
        "wr_ls_constants.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    # deps = [
        # "//cyber",
        # "//modules/sensors/proto:sensors_cc_proto",
    # ],
)



cc_library(
    name = "wr_ls_common",
    srcs = ["wr_ls_common.cpp"],
    hdrs = [
        "wr_ls_common.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        ":parser_base",
        ":wr_ls_constants",
        "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)


cc_library(
    name = "wr_ls_common_tcp",
    srcs = ["wr_ls_common_tcp.cpp"],
    hdrs = [
        "wr_ls_common_tcp.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        ":wr_ls_common",
        "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
        # "@boost",
    ],
)


cc_library(
    name = "wr_ls_sensor_frame",
    srcs = ["wr_ls_sensor_frame.cpp"],
    hdrs = [
        "wr_ls_sensor_frame.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        ":wr_ls_constants",
        "//cyber",
    ],
)



cc_library(
    name = "wr_ls1207de_parser",
    srcs = ["wr_ls1207de_parser.cpp"],
    hdrs = [
        "wr_ls1207de_parser.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        ":parser_base",
        ":wr_ls_sensor_frame",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)
