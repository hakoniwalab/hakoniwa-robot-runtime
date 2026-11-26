#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hakoniwa::robot_runtime::runtime {

struct Vector3State {
    double x {0.0};
    double y {0.0};
    double z {0.0};
};

struct EulerState {
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};
};

struct SurfaceFrictionOperation {
    std::vector<std::string> surfaces;
    double sliding_friction {0.0};
    std::uint64_t created_at_usec {0};
};

struct BodyMotionOperation {
    std::string body;
    Vector3State position;
    EulerState orientation;
    Vector3State linear_velocity;
    Vector3State angular_velocity;
    std::uint64_t created_at_usec {0};
};

struct HumanoidMotionOperation {
    std::string humanoid;
    Vector3State linear_velocity;
    Vector3State angular_velocity;
    std::string animation;
    std::string ride;
};

struct PlantOperations {
    std::vector<SurfaceFrictionOperation> surface_frictions;
    std::vector<BodyMotionOperation> body_motions;
    std::vector<HumanoidMotionOperation> humanoid_motions;
};

struct PlantCapabilities {
    bool surface_friction {false};
    bool body_motion {false};
    bool humanoid_motion {false};
};

} // namespace hakoniwa::robot_runtime::runtime
