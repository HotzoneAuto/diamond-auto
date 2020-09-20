load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lidar_listener",
    srcs = ["lidar_listener.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "@eigen",
	"@local_config_pcl//:pcl",
	"@local_config_vtk//:vtk",
	"@boost",
    ],
)

cpplint()
