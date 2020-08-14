load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_graph",
    srcs = [
        "graph.cc",
    ],
    linkopts = ["-lrt"],
    deps = [
        "//cyber:init",
        "//cyber/service_discovery:topology_manager",
        # "//third_party:ncurses",
        # "//third_party:pthread",
        "//third_party:uuid",
    ],
)