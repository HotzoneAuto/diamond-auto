load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ipcamera",
    #hdrs = [
    #    "GenICam/Defs.h",
    #    "GenICam/AcquisitionControl.h"
    #    "GenICam/Camera.h"
    #    "GenICam/DeviceControl.h"
    #    "GenICam/EventSubscribe.h"
    #    "GenICam/ISPControl.h"
    #    "Infra/PrintLog.h",
    #    "Infra/Thread.h",
    #    "Media/ImageConvert.h",
    #    "Media/RecordVideo.h",
    #    "Media/VideoRender.h",
    #    "Memory/ScopedPtr.h",
    #],
    hdrs = glob(["*"]),
    includes = [
        ".",
    ],
    linkopts = [
        "-L/opt/apollo/pkgs/ipcamera/lib",
        "-lMVSDK",
        "-lRecordVideo",
        "-lImageConvert",
    ],
)
