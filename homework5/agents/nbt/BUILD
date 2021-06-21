package(default_visibility = ["//visibility:public"])

cc_library(
    name = "nbt_agent",
    hdrs = ["nbt_agent.h"],
    deps = [
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//homework5/simulation:vehicle_agent",
        "//homework5/simulation:vehicle_agent_factory",
        "@glog",
        "//common/proto:cc_map_proto",
        "//common/proto:cc_route_proto",
        "//common/utils/file",
    ],
)
