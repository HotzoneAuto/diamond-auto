load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ipcamera",
    hdrs = glob(["**/*"]),
    includes = [
        ".",
        "GenICam/*",
        "Infra/*",
        "Memory/*",
        "Media/*",
    ],
    copts = [
        "-I/opt/apollo/pkgs/ipcamera/include",
        "-c",
        "-Wall",
        "-g",
        "-m64",
    ],
    linkopts = [
        "-L/opt/apollo/pkgs/ipcamera/lib",
        "-lMVSDK",
        "-lRecordVideo",
        "-lImageConvert",
    ],
)
