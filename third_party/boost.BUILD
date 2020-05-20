licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "boost",
    copts = ["-fPIC"],
    includes = ["."],
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu/",
        "-lboost_system",
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_thread",
        "-lboost_signals",
    ],
)
