load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opengl",
    includes = ["."],
    linkopts = [
        "-lGLU",
        "-lGL",
        "-lSM",
        "-lICE",
        "-lXext",
    ],
)
