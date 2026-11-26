#pragma once

#include "plugins/ackermann_step/ackermann_step.hpp"
#include "runtime/service/async_service_pipeline.hpp"
#include "runtime/service/builtin_robot_config.hpp"

#include "ackermann_msgs/pdu_cpptype_conv_AckermannDrive.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::test_support {

inline std::shared_ptr<runtime::ServicePlugin> make_person_plugin(
    const std::string& person, const std::string& service, std::uint64_t physics_step_usec)
{
    return std::make_shared<runtime::ServicePlugin>(
        plugin::PluginLibrary::open_builtin("person_step", HAKO_PLUGIN_KIND_SERVICE),
        runtime::person_plugin_config(person, {service, "hako_srv_msgs/PersonStep", true}, physics_step_usec * 1000));
}

inline std::vector<runtime::RuntimeActuatorConfig> ackermann_actuators(
    const runtime::RuntimeAckermannActuatorBindings& ids)
{
    using runtime::ActuatorCommandType;
    return {
        {.component_id = ids.steering_left, .command_type = ActuatorCommandType::Position},
        {.component_id = ids.steering_right, .command_type = ActuatorCommandType::Position},
        {.component_id = ids.drive_left, .command_type = ActuatorCommandType::Velocity},
        {.component_id = ids.drive_right, .command_type = ActuatorCommandType::Velocity},
    };
}

class AckermannTopicReader final : public runtime::IRobotPduReader {
public:
    void send(const plugins::ackermann_step::AckermannDriveCommand& command)
    {
        HakoCpp_AckermannDrive pdu {};
        pdu.steering_angle = command.steering_angle_rad;
        pdu.steering_angle_velocity = command.steering_angle_velocity_rad_s;
        pdu.speed = command.speed_m_s;
        pdu.acceleration = command.acceleration_m_s2;
        pdu.jerk = command.jerk_m_s3;
        std::vector<std::byte> packet(256);
        const auto length = hako::pdu::msgs::ackermann_msgs::AckermannDrive {}.cpp2pdu(
            pdu, reinterpret_cast<char*>(packet.data()), packet.size());
        if (length <= 0) {
            throw std::runtime_error("could not encode Ackermann topic test input");
        }
        packet.resize(static_cast<std::size_t>(length));
        pending_ = std::move(packet);
    }

    std::optional<runtime::RobotPduSample> take_latest(std::uint64_t time_usec) override
    {
        auto packet = std::exchange(pending_, std::nullopt);
        if (!packet) {
            return std::nullopt;
        }
        return runtime::RobotPduSample {std::move(*packet), time_usec};
    }

    void reset() noexcept override { pending_.reset(); }

private:
    std::optional<std::vector<std::byte>> pending_;
};

inline std::shared_ptr<runtime::ServicePlugin> make_ackermann_plugin(
    const runtime::RuntimeAckermannControllerConfig& drive, const std::string& body, const std::string& service,
    std::uint64_t physics_step_usec, const std::vector<std::string>& surfaces = {})
{
    auto config = runtime::ackermann_plugin_config(drive, ackermann_actuators(drive.actuators), true, surfaces);
    config.bodies.push_back(body);
    config.services.push_back({service, "hako_srv_msgs/AckermannStep", true});
    config.physics_step_ns = physics_step_usec * 1000;
    return std::make_shared<runtime::ServicePlugin>(
        plugin::PluginLibrary::open_builtin("ackermann_step", HAKO_PLUGIN_KIND_SERVICE), std::move(config));
}

inline runtime::RobotServiceComponents make_topic_car(
    const runtime::RuntimeAckermannControllerConfig& drive,
    const std::vector<runtime::RuntimeActuatorConfig>& actuators,
    const std::shared_ptr<AckermannTopicReader>& reader, std::uint64_t physics_step_usec)
{
    auto config = runtime::ackermann_plugin_config(drive, actuators, false, {});
    config.pdus.push_back({drive.pdu_name, reader});
    config.physics_step_ns = physics_step_usec * 1000;
    auto robot = std::make_shared<runtime::ServicePlugin>(
        plugin::PluginLibrary::open_builtin("ackermann_step", HAKO_PLUGIN_KIND_SERVICE), std::move(config));
    auto source = std::make_shared<runtime::RobotServiceSource>("ackermann-pdu:" + drive.component_id,
        std::vector<std::shared_ptr<runtime::AsyncServiceCommandSource>> {},
        [robot](const auto& context) { return robot->poll_pdus(context); });
    return {source, std::make_shared<runtime::RobotServiceController>(drive.component_id, source, robot), {}};
}

} // namespace hakoniwa::robot_runtime::test_support
