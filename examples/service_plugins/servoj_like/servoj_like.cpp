#include "hakoniwa/robot_runtime/plugin/robot_plugin.hpp"

#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJRequestPacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJResponsePacket.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace {

namespace plugin = hakoniwa::robot_runtime::plugin;

HAKO_ROBOT_SERVICE(my_robot_srvs_msgs, ServoJ);

constexpr std::size_t joint_count = 6;
constexpr std::size_t servo_service = 0;

class ServoJRobot final : public plugin::RobotPlugin {
public:
    explicit ServoJRobot(const plugin::RobotContext& context) : RobotPlugin(context)
    {
        if (context.actuators.size() != joint_count) {
            throw std::invalid_argument("expected six SO-101 actuators");
        }
        bind<ServoJService>(servo_service, &ServoJRobot::on_goal, &ServoJRobot::reply);
    }

    void step(plugin::StepView& step) override
    {
        if (context().services.size() != 1) {
            throw std::runtime_error("expected one ServoJ service");
        }
        const auto& tick = step.tick();
        const auto state = step.service_state(servo_service);
        if (state == plugin::ServiceState::Canceling) {
            active_ = false;
            step.finish(servo_service, plugin::ServiceResult::Canceled);
        } else if (state == plugin::ServiceState::Doing) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                start_[i] = step.position(i);
            }
            start_ns_ = tick.sim_time_ns;
            active_ = true;
            commanded_ = false;
            step.finish(servo_service, plugin::ServiceResult::Ok);
        }
        if (commanded_ && !tick.selected) {
            active_ = false;
        }
        if (!active_) {
            return;
        }
        const double elapsed = static_cast<double>(tick.sim_time_ns - start_ns_) * 1e-9;
        const double progress = std::min(elapsed / goal_.t, 1.0);
        for (std::size_t i = 0; i < joint_count; ++i) {
            step.command(i, std::lerp(start_[i], goal_.joint[i], progress));
        }
        commanded_ = true;
    }

    void reset() override
    {
        active_ = false;
        commanded_ = false;
    }

private:
    bool on_goal(const HakoCpp_ServoJRequest& goal)
    {
        if (!std::isfinite(goal.t) || goal.t < 1e-6 || goal.t > 60.0) {
            return false;
        }
        for (std::size_t i = 0; i < joint_count; ++i) {
            const auto value = goal.joint[i];
            const auto& actuator = context().actuators[i];
            if (!std::isfinite(value) || value < actuator.minimum || value > actuator.maximum) {
                return false;
            }
        }
        goal_ = goal;
        return true;
    }

    HakoCpp_ServoJResponse reply(const plugin::StateView&)
    {
        return {0};
    }

    std::array<double, joint_count> start_ {};
    HakoCpp_ServoJRequest goal_ {};
    std::uint64_t start_ns_ {0};
    bool active_ {false};
    bool commanded_ {false};
};

} // namespace

HAKO_ROBOT_PLUGIN(ServoJRobot, servoj_like)
