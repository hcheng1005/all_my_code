load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "my_typedef",
    hdrs = ["my_typedef.h"],
)

cc_library(
    name = "Optics",
    hdrs = ["Optics.hpp"],
    deps = [        
        "@local_config_pcl//:pcl",
    ]
)


cc_library(
    name = "iou",
    srcs = ["iou.cc"],
    hdrs = ["iou.h"],
    alwayslink = True,
)

cc_library(
    name = "kalman",
    srcs = ["kalman.cc"],
    hdrs = ["kalman.h"],
    deps = [
        ":iou",
        "@eigen",
    ],
    alwayslink = True,
)


cc_library(
    name = "lshape",
    srcs = ["lshape.cc"],
    hdrs = ["lshape.h"],
    deps = [
        "@eigen",
    ],
    alwayslink = True,
)


cc_library(
    name = "hm_tracker",
    srcs = ["hm_tracker.cc"],
    hdrs = ["hm_tracker.h"],
    deps = [
        "//cyber/common:macros",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/lidar/common:lidar_object",
        "//modules/perception/lidar/common:lidar_timer",
        "//modules/perception/lidar/lib/interface:base_multi_target_tracker",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/onboard/transform_wrapper:transform_vel",
        "//modules/perception/common:perception_gflags",
        ":iou",
        ":kalman",
        ":lshape",
        ":Optics",
        "//modules/perception/common/graph:hungarian_optimizer",
        "//modules/perception/common/graph:secure_matrix"
    ],
    alwayslink = True,
)

cpplint()
