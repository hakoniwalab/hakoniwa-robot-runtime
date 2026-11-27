// Test fixture for the C++ template layer (stage 2c-4): a robot written as a
// C++ class with HakoCpp types. One service, hako_srv_msgs/AddTwoInts:
// the request sets a joint target to a + b (tenths of a radian); the reply
// carries the sum. Built as a shared library. Variant ADDER_WRONG_TYPE binds a
// type the robot does not have; ADDER_THROWING_STEP fails in step().
#include "hakoniwa/robot_runtime/plugin/robot_plugin.hpp"

#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_AddTwoIntsResponsePacket.hpp"
#ifdef ADDER_WRONG_TYPE
#include "hako_srv_msgs/pdu_cpptype_conv_GetWorldTimeRequestPacket.hpp"
#include "hako_srv_msgs/pdu_cpptype_conv_GetWorldTimeResponsePacket.hpp"
#endif

#include <stdexcept>

namespace {

namespace plugin = hakoniwa::robot_runtime::plugin;

HAKO_ROBOT_SERVICE(hako_srv_msgs, AddTwoInts);
#ifdef ADDER_WRONG_TYPE
HAKO_ROBOT_SERVICE(hako_srv_msgs, GetWorldTime);
#endif

class Adder final : public plugin::RobotPlugin {
public:
    explicit Adder(const plugin::RobotContext& context) : RobotPlugin(context)
    {
        if (context.actuators.size() != 1) {
            throw std::invalid_argument("adder needs exactly one actuator");
        }
#ifdef ADDER_WRONG_TYPE
        bind<GetWorldTimeService>(0, &Adder::on_get_world_time, &Adder::reply_get_world_time);
#else
        bind<AddTwoIntsService>(0, &Adder::on_add, &Adder::reply_add);
#endif
    }

    void step(plugin::StepView& step) override
    {
#ifdef ADDER_THROWING_STEP
        (void)step;
        throw std::runtime_error("adder step failed");
#else
        if (step.service_state(0) == plugin::ServiceState::Doing) {
            step.finish(0, plugin::ServiceResult::Ok);
            commanding_ = true;
        }
        if (commanding_) {
            step.command(0, static_cast<double>(sum_) / 10.0);
        }
#endif
    }

    void reset() override
    {
        sum_ = 0;
        commanding_ = false;
    }

private:
    bool on_add(const HakoCpp_AddTwoIntsRequest& request)
    {
        if (request.a < 0 || request.b < 0) {
            return false;
        }
        sum_ = request.a + request.b;
        return true;
    }

    HakoCpp_AddTwoIntsResponse reply_add(const plugin::StateView& state)
    {
        HakoCpp_AddTwoIntsResponse response {};
        response.sum = sum_ * 100 + static_cast<long long>(state.position(0) * 10.0 + 0.5);
        return response;
    }

#ifdef ADDER_WRONG_TYPE
    bool on_get_world_time(const HakoCpp_GetWorldTimeRequest&) { return true; }
    HakoCpp_GetWorldTimeResponse reply_get_world_time(const plugin::StateView&) { return {}; }
#endif

    long long sum_ {0};
    bool commanding_ {false};
};

} // namespace

HAKO_ROBOT_PLUGIN(Adder, adder)
