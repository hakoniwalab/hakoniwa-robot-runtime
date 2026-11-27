#include "hakoniwa/robot_runtime/plugin/robot_plugin.hpp"

#include "my_robot_srvs_msgs/pdu_cpptype_conv_GetAngleRequestPacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_GetAngleResponsePacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJRequestPacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJResponsePacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_StopRequestPacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_StopResponsePacket.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace {

namespace plugin = hakoniwa::robot_runtime::plugin;

HAKO_ROBOT_SERVICE(my_robot_srvs_msgs, ServoJ);
HAKO_ROBOT_SERVICE(my_robot_srvs_msgs, Stop);
HAKO_ROBOT_SERVICE(my_robot_srvs_msgs, GetAngle);

constexpr std::size_t joint_count = 6;
class ServoJRobot final : public plugin::RobotPlugin {
public:
    explicit ServoJRobot(const plugin::RobotContext& context) : RobotPlugin(context)
    {
        if (context.actuators.size() != joint_count) {
            throw std::invalid_argument("expected six SO-101 actuators");
        }
        if (context.services.size() != 3) {
            throw std::runtime_error("expected ServoJ, Stop and GetAngle services");
        }
        servo_service = bind<ServoJService>(&ServoJRobot::on_goal, &ServoJRobot::reply_goal);
        stop_service = bind<StopService>(&ServoJRobot::on_stop, &ServoJRobot::reply_stop);
        get_angle_service = bind<GetAngleService>(&ServoJRobot::on_get_angle, &ServoJRobot::reply_get_angle);
    }

    void step(plugin::StepView& step) override
    {
        const auto& tick = step.tick();
        const auto state = step.service_state(servo_service);
        if (state == plugin::ServiceState::Canceling) {
            motion_ = Motion::Idle;
            step.finish(servo_service, plugin::ServiceResult::Canceled);
        } else if (state == plugin::ServiceState::Doing) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                start_[i] = step.position(i);
            }
            start_ns_ = tick.sim_time_ns;
            motion_ = Motion::ServoJ;
            commanded_ = false;
            step.finish(servo_service, plugin::ServiceResult::Ok);
        }
        // If both requests are accepted on this tick, Stop takes precedence.
        update_stop(step);
        update_get_angle(step);
        if (commanded_ && !tick.selected) {
            motion_ = Motion::Idle;
        }
        if (motion_ == Motion::Idle) {
            return;
        }
        if (motion_ == Motion::Hold) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                step.command(i, held_[i]);
            }
        } else {
            const double elapsed = static_cast<double>(tick.sim_time_ns - start_ns_) * 1e-9;
            const double progress = std::min(elapsed / goal_.t, 1.0);
            for (std::size_t i = 0; i < joint_count; ++i) {
                step.command(i, std::lerp(start_[i], goal_.joint[i], progress));
            }
        }
        commanded_ = true;
    }

    void reset() override
    {
        motion_ = Motion::Idle;
        commanded_ = false;
    }

private:
    enum class Motion { Idle, ServoJ, Hold };

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

    HakoCpp_ServoJResponse reply_goal(const plugin::StateView&)
    {
        return {0};
    }

    bool on_stop(const HakoCpp_StopRequest& request)
    {
        return request.mode == 0;
    }

    HakoCpp_StopResponse reply_stop(const plugin::StateView&)
    {
        return {0};
    }

    void update_stop(plugin::StepView& step)
    {
        const auto state = step.service_state(stop_service);
        if (state == plugin::ServiceState::Canceling) {
            step.finish(stop_service, plugin::ServiceResult::Canceled);
        } else if (state == plugin::ServiceState::Doing) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                held_[i] = step.position(i);
            }
            motion_ = Motion::Hold;
            commanded_ = false;
            step.finish(stop_service, plugin::ServiceResult::Ok);
        }
    }

    bool on_get_angle(const HakoCpp_GetAngleRequest&)
    {
        return true;
    }

    HakoCpp_GetAngleResponse reply_get_angle(const plugin::StateView&)
    {
        return angle_response_;
    }

    void update_get_angle(plugin::StepView& step)
    {
        const auto state = step.service_state(get_angle_service);
        if (state == plugin::ServiceState::Canceling) {
            step.finish(get_angle_service, plugin::ServiceResult::Canceled);
        } else if (state == plugin::ServiceState::Doing) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                angle_response_.joint[i] = step.position(i);
            }
            angle_response_.sim_time_ns = step.tick().sim_time_ns;
            step.finish(get_angle_service, plugin::ServiceResult::Ok);
        }
    }

    std::size_t servo_service {};
    std::size_t stop_service {};
    std::size_t get_angle_service {};
    std::array<double, joint_count> start_ {};
    std::array<double, joint_count> held_ {};
    HakoCpp_ServoJRequest goal_ {};
    HakoCpp_GetAngleResponse angle_response_ {};
    std::uint64_t start_ns_ {0};
    Motion motion_ {Motion::Idle};
    bool commanded_ {false};
};

} // namespace

HAKO_ROBOT_PLUGIN(ServoJRobot, servoj_like, ServoJService, StopService, GetAngleService)
