load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "benchmark",
    srcs = [
        "src/arraysize.h",
        "src/benchmark.cc",
        "src/benchmark_api_internal.h",
        "src/benchmark_register.cc",
        "src/check.h",
        "src/colorprint.cc",
        "src/colorprint.h",
        "src/commandlineflags.cc",
        "src/commandlineflags.h",
        "src/complexity.cc",
        "src/complexity.h",
        "src/console_reporter.cc",
        "src/csv_reporter.cc",
        "src/cycleclock.h",
        "src/internal_macros.h",
        "src/json_reporter.cc",
        "src/log.h",
        "src/mutex.h",
        "src/re.h",
        "src/reporter.cc",
        "src/sleep.cc",
        "src/sleep.h",
        "src/stat.h",
        "src/string_util.cc",
        "src/string_util.h",
        "src/sysinfo.cc",
        "src/sysinfo.h",
        "src/timers.cc",
        "src/timers.h",
    ],
    hdrs = [
        "include/benchmark/benchmark.h",
        "include/benchmark/benchmark_api.h",
        "include/benchmark/macros.h",
        "include/benchmark/reporter.h",
    ],
    defines = ["HAVE_POSIX_REGEX"],
    includes = ["include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)
