package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cubic_spline",
    srcs = ["cubic_spline.cc"],
    hdrs = ["cubic_spline.h"],
    deps = [
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "interpolate_main",
    srcs = ["interpolate_main.cc"],
    deps = [
        ":cubic_spline",
        "@gflags",
        "@glog",
    ],
)

cc_test(
    name = "cubic_test",
    srcs = ["cubic_test.cc"],
    deps = [
        ":cubic_spline",
        "//utils/testing:test_main",
    ],
)