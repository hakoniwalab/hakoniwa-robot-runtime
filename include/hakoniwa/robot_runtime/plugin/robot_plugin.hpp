#ifndef HAKONIWA_ROBOT_RUNTIME_PLUGIN_ROBOT_PLUGIN_HPP
#define HAKONIWA_ROBOT_RUNTIME_PLUGIN_ROBOT_PLUGIN_HPP

#include "hakoniwa/robot_runtime/plugin/service_plugin.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace hakoniwa::robot_runtime::plugin {

struct ActuatorInfo {
    std::string id;
    std::uint8_t command_type;
    double minimum;
    double maximum;
};

struct ServiceInfo {
    std::string name;
    std::string type;
    bool lockstep;
};

struct RobotContext {
    std::string robot;
    std::vector<ActuatorInfo> actuators;
    std::vector<std::string> bodies, humanoids, pdus;
    std::vector<ServiceInfo> services;
    std::string config_json;
    std::uint64_t physics_step_ns;
};

enum class ServiceState {
    Idle = HAKO_SERVICE_STATE_IDLE,
    Doing = HAKO_SERVICE_STATE_DOING,
    Canceling = HAKO_SERVICE_STATE_CANCELING,
};

enum class ServiceResult {
    Pending = HAKO_SERVICE_RESULT_PENDING,
    Ok = HAKO_SERVICE_RESULT_OK,
    Canceled = HAKO_SERVICE_RESULT_CANCELED,
    Error = HAKO_SERVICE_RESULT_ERROR,
};

namespace detail {

template <class Class, class... Services>
struct PluginTable;

inline void check_index(std::size_t index, std::size_t count, const char* message)
{
    if (index >= count) {
        throw std::out_of_range(message);
    }
}

struct FrictionNames {
    explicit FrictionNames(std::vector<std::string> names) : strings(std::move(names))
    {
        pointers.reserve(strings.size());
        for (const auto& name : strings) {
            pointers.push_back(name.c_str());
        }
    }

    std::vector<std::string> strings;
    std::vector<const char*> pointers;
};

// Runtime copies these strings after the C hook returns.
struct StepStorage {
    std::deque<FrictionNames> frictions;
    std::deque<std::string> motion_names;
};

} // namespace detail

// A read-only view. Its state and PDU byte pointers are valid during the call.
class StateView {
public:
    const HakoTickV1& tick() const { return tick_; }
    std::size_t joint_count() const { return state_.joints.count; }

    double position(std::size_t joint) const
    {
        detail::check_index(joint, joint_count(), "joint index out of range");
        return state_.joints.position[joint];
    }

    double velocity(std::size_t joint) const
    {
        detail::check_index(joint, joint_count(), "joint index out of range");
        return state_.joints.velocity[joint];
    }

    double effort(std::size_t joint) const
    {
        detail::check_index(joint, joint_count(), "joint index out of range");
        return state_.joints.effort[joint];
    }

    const HakoBodyStateV1& body(std::size_t index) const
    {
        detail::check_index(index, state_.body_count, "body index out of range");
        return state_.bodies[index];
    }

    const HakoHumanoidStateV1& humanoid(std::size_t index) const
    {
        detail::check_index(index, state_.humanoid_count, "humanoid index out of range");
        return state_.humanoids[index];
    }

    std::size_t pdu_count() const { return state_.pdu_count; }

    const HakoPduViewV1& pdu(std::size_t index) const
    {
        detail::check_index(index, pdu_count(), "PDU input index out of range");
        return state_.pdus[index];
    }

protected:
    StateView(const HakoTickV1& tick, const HakoRobotStateV1& state) : tick_(tick), state_(state) {}

private:
    template <class Class, class... Services>
    friend struct detail::PluginTable;

    const HakoTickV1& tick_;
    const HakoRobotStateV1& state_;
};

class StepView : public StateView {
public:
    ServiceState service_state(std::size_t service) const
    {
        detail::check_index(service, slot_count_, "service index out of range");
        return static_cast<ServiceState>(slots_[service].state);
    }

    void finish(std::size_t service, ServiceResult result)
    {
        detail::check_index(service, slot_count_, "service index out of range");
        slots_[service].result = static_cast<std::uint8_t>(result);
    }

    void command(std::size_t joint, double value, std::uint8_t type = HAKO_JOINT_COMMAND_POSITION)
    {
        auto& commands = output_.commands;
        detail::check_index(joint, commands.count, "joint command index out of range");
        commands.command_type[joint] = type;
        commands.value[joint] = value;
        commands.has_command[joint] = 1;
    }

    void surface_friction(std::vector<std::string> surfaces, double sliding_friction)
    {
        detail::check_index(output_.friction_count, output_.friction_capacity, "surface friction capacity exceeded");
        if (surfaces.size() > std::numeric_limits<std::uint32_t>::max()) {
            throw std::length_error("too many surfaces in a friction operation");
        }
        const auto& names = storage_.frictions.emplace_back(std::move(surfaces)).pointers;
        output_.frictions[output_.friction_count] = {
            static_cast<std::uint32_t>(names.size()), names.data(), sliding_friction};
        ++output_.friction_count;
    }

    void body_motion(std::string body, HakoVector3V1 position, HakoEulerV1 orientation,
        HakoVector3V1 linear_velocity, HakoVector3V1 angular_velocity)
    {
        detail::check_index(output_.body_motion_count, output_.body_motion_capacity, "body motion capacity exceeded");
        const auto* name = keep_name(std::move(body));
        output_.body_motions[output_.body_motion_count] = {name, position, orientation,
            linear_velocity, angular_velocity};
        ++output_.body_motion_count;
    }

    void humanoid_motion(std::string humanoid, HakoVector3V1 linear_velocity, HakoVector3V1 angular_velocity,
        std::string animation, std::string ride)
    {
        detail::check_index(output_.humanoid_motion_count, output_.humanoid_motion_capacity,
            "humanoid motion capacity exceeded");
        const auto* name = keep_name(std::move(humanoid));
        const auto* animation_name = keep_name(std::move(animation));
        const auto* ride_name = keep_name(std::move(ride));
        output_.humanoid_motions[output_.humanoid_motion_count] = {
            name, linear_velocity, angular_velocity, animation_name, ride_name};
        ++output_.humanoid_motion_count;
    }

private:
    template <class Class, class... Services>
    friend struct detail::PluginTable;

    StepView(const HakoTickV1& tick, const HakoRobotStateV1& state, HakoServiceSlotV1* slots,
        std::size_t slot_count, HakoRobotOutputV1& output, detail::StepStorage& storage)
        : StateView(tick, state), slots_(slots), slot_count_(slot_count), output_(output), storage_(storage)
    {
        storage_.frictions.clear();
        storage_.motion_names.clear();
    }

    const char* keep_name(std::string name)
    {
        return storage_.motion_names.emplace_back(std::move(name)).c_str();
    }

    HakoServiceSlotV1* slots_;
    std::size_t slot_count_;
    HakoRobotOutputV1& output_;
    detail::StepStorage& storage_;
};

namespace detail {

// Generated decoders take a packet pointer without a length. Check the outer
// layout first, including the fixed part they read as a C struct.
template <class Service, class WirePacket>
bool valid_packet(const void* data, std::size_t size, std::size_t initialized_base_size)
{
    constexpr auto base = Service::base_offset;
    using Metadata = typename Service::Metadata;
    if (!data || size < base + sizeof(WirePacket)) {
        return false;
    }
    const auto address = reinterpret_cast<std::uintptr_t>(data);
    if (address % alignof(Metadata) != 0 || (address + base) % alignof(WirePacket) != 0) {
        return false;
    }
    Metadata metadata {};
    std::memcpy(&metadata, data, sizeof(metadata));
    return Service::valid_metadata(metadata) && metadata.base_off == base
        && metadata.heap_off >= base + initialized_base_size
        && metadata.heap_off <= metadata.total_size && metadata.total_size <= size;
}

} // namespace detail

class RobotPlugin {
public:
    explicit RobotPlugin(const RobotContext& context) : context_(context), handlers_(context.services.size()) {}
    virtual ~RobotPlugin() = default;
    const RobotContext& context() const { return context_; }
    virtual void step(StepView& step) = 0;
    virtual void reset() {}
    virtual void cancel(std::size_t) {}

protected:
    template <class Service, class Self>
    std::size_t bind(bool (Self::*on_request)(const typename Service::Request&),
        typename Service::Response (Self::*on_response)(const StateView&))
    {
        const auto& services = context_.services;
        const auto matches = [](const ServiceInfo& service) { return service.type == Service::type; };
        const auto first = std::find_if(services.begin(), services.end(), matches);
        if (first == services.end()) {
            throw std::invalid_argument("robot " + context_.robot + " has no service of type "
                + std::string(Service::type));
        }
        if (std::find_if(first + 1, services.end(), matches) != services.end()) {
            throw std::invalid_argument("robot " + context_.robot + " has multiple services of type "
                + std::string(Service::type) + "; use the index overload of bind");
        }
        return bind<Service>(static_cast<std::size_t>(first - services.begin()), on_request, on_response);
    }

    // Use an index when a robot has multiple services of the same type.
    template <class Service, class Self>
    std::size_t bind(std::size_t service,
        bool (Self::*on_request)(const typename Service::Request&),
        typename Service::Response (Self::*on_response)(const StateView&))
    {
        if (service >= handlers_.size()) {
            throw std::invalid_argument("service index out of range: " + std::to_string(service));
        }
        const auto& type = context_.services[service].type;
        if (type != Service::type) {
            throw std::invalid_argument("service " + context_.services[service].name + " has type " + type
                + ", cannot bind " + std::string(Service::type));
        }
        if (handlers_[service].request) {
            throw std::invalid_argument("robot " + context_.robot + " service "
                + context_.services[service].name + " is already bound");
        }
        handlers_[service] = {
            [on_request](RobotPlugin& plugin, const HakoServiceRequestV1& request) {
                using WirePacket = typename Service::WireRequestPacket;
                if (!detail::valid_packet<Service, WirePacket>(request.data, request.size, sizeof(WirePacket))) {
                    return false;
                }
                typename Service::RequestPacket packet {};
                typename Service::RequestConverter converter;
                // The generated decoder reads the packet through a mutable pointer.
                auto* bytes = const_cast<char*>(static_cast<const char*>(request.data));
                return converter.pdu2cpp(bytes, packet) && (static_cast<Self&>(plugin).*on_request)(packet.body);
            },
            [on_response](RobotPlugin& plugin, const StateView& state, const HakoServiceResponseV1& response) {
                using WirePacket = typename Service::WireResponsePacket;
                if (response.size > static_cast<std::size_t>(std::numeric_limits<int>::max())
                    || !detail::valid_packet<Service, WirePacket>(
                        response.data, response.size, sizeof(decltype(WirePacket::header)))) {
                    return false;
                }
                auto* bytes = static_cast<char*>(response.data);
                // Core writes the header into a reused buffer. Decode with an empty body so stale
                // variable-length fields cannot refer to a previous reply's heap.
                std::memset(bytes + Service::base_offset + offsetof(WirePacket, body), 0,
                    sizeof(decltype(WirePacket::body)));
                typename Service::ResponsePacket packet {};
                typename Service::ResponseConverter converter;
                if (!converter.pdu2cpp(bytes, packet)) {
                    return false;
                }
                packet.body = (static_cast<Self&>(plugin).*on_response)(state);
                return converter.cpp2pdu(packet, bytes, static_cast<int>(response.size)) > 0;
            },
        };
        return service;
    }

private:
    template <class Class, class... Services>
    friend struct detail::PluginTable;

    struct Handlers {
        std::function<bool(RobotPlugin&, const HakoServiceRequestV1&)> request;
        std::function<bool(RobotPlugin&, const StateView&, const HakoServiceResponseV1&)> response;
    };

    RobotContext context_;
    std::vector<Handlers> handlers_;
    detail::StepStorage step_storage_;
};

namespace detail {

inline std::vector<std::string> copy_names(const char* const* names, std::size_t count)
{
    std::vector<std::string> copy;
    copy.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        copy.emplace_back(names[i]);
    }
    return copy;
}

inline RobotContext robot_context(const HakoRobotCreateV1& args)
{
    RobotContext context {
        args.robot, {},
        copy_names(args.bodies, args.body_count),
        copy_names(args.humanoids, args.humanoid_count),
        copy_names(args.pdus, args.pdu_count),
        {}, args.config_json, args.physics_step_ns,
    };
    context.actuators.reserve(args.actuator_count);
    for (std::size_t i = 0; i < args.actuator_count; ++i) {
        const auto& actuator = args.actuators[i];
        context.actuators.push_back({actuator.id, actuator.command_type, actuator.minimum, actuator.maximum});
    }
    context.services.reserve(args.service_count);
    for (std::size_t i = 0; i < args.service_count; ++i) {
        const auto& service = args.services[i];
        context.services.push_back({service.name, service.type, service.lockstep != 0});
    }
    return context;
}

inline void write_error(char* error, std::size_t size, const char* message) noexcept
{
    if (error && size != 0) {
        std::snprintf(error, size, "%s", message);
    }
}

template <class Class, class... Services>
struct PluginTable {
    static constexpr HakoServicePluginV1 make(const char* name)
    {
        return {
            {sizeof(HakoServicePluginV1), HAKO_SERVICE_PLUGIN_ABI_VERSION, HAKO_PLUGIN_KIND_SERVICE,
                HAKO_SERVICE_PLUGIN_INTERFACE_REVISION, name},
            declaration_json.data(),
            create, destroy, on_request, on_cancel, on_step, on_response, reset,
        };
    }

private:
    static constexpr std::array<std::string_view, sizeof...(Services)> types {Services::type...};

    static consteval auto make_declaration()
    {
        constexpr std::string_view start = R"({"service_types":[)";
        constexpr std::string_view operations = [] {
            if constexpr (requires { Class::plant_operations; }) {
                return std::string_view(Class::plant_operations);
            }
            return std::string_view("[]");
        }();
        constexpr std::string_view middle = R"(],"plant_operations":)";
        // Space for the final brace, NUL and a comma between adjacent types.
        constexpr auto size = start.size() + middle.size() + operations.size() + 2
            + (std::size_t {0} + ... + (std::string_view(Services::type).size() + 2))
            + (types.empty() ? 0 : types.size() - 1);
        std::array<char, size> json {};
        auto out = std::copy(start.begin(), start.end(), json.begin());
        for (const auto& type : types) {
            if (out != json.begin() + start.size()) {
                *out++ = ',';
            }
            *out++ = '"';
            out = std::copy(type.begin(), type.end(), out);
            *out++ = '"';
        }
        out = std::copy(middle.begin(), middle.end(), out);
        out = std::copy(operations.begin(), operations.end(), out);
        *out = '}';
        return json;
    }

    static constexpr auto declaration_json = make_declaration();

    static void check_bindings(const RobotPlugin& plugin)
    {
        const auto& context = plugin.context_;
        const auto prefix = "robot " + context.robot;
        for (std::size_t index = 0; index < context.services.size(); ++index) {
            const auto& type = context.services[index].type;
            if (plugin.handlers_[index].request && std::find(types.begin(), types.end(), type) == types.end()) {
                throw std::invalid_argument(prefix + " bound undeclared service type " + type);
            }
        }
        for (const auto type : types) {
            bool bound = false;
            for (std::size_t index = 0; index < context.services.size(); ++index) {
                if (context.services[index].type == type && plugin.handlers_[index].request) {
                    bound = true;
                    break;
                }
            }
            if (!bound) {
                throw std::invalid_argument(prefix + " declared service type " + std::string(type)
                    + " but did not bind it");
            }
        }
    }

    static HakoRobotPlugin* create(const HakoRobotCreateV1* args, char* error, std::size_t size) noexcept
    {
        try {
            const auto context = robot_context(*args);
            auto plugin = std::make_unique<Class>(context);
            check_bindings(*plugin);
            return reinterpret_cast<HakoRobotPlugin*>(plugin.release());
        } catch (const std::exception& exception) {
            write_error(error, size, exception.what());
        } catch (...) {
            write_error(error, size, "unknown exception in robot plugin constructor");
        }
        return nullptr;
    }

    static void destroy(HakoRobotPlugin* robot) noexcept
    {
        delete reinterpret_cast<Class*>(robot);
    }

    static std::int32_t on_request(
        HakoRobotPlugin* robot, std::uint32_t service, const HakoServiceRequestV1* request) noexcept
    {
        try {
            RobotPlugin& plugin = *reinterpret_cast<Class*>(robot);
            if (service >= plugin.handlers_.size()) {
                return 0;
            }
            const auto& handler = plugin.handlers_[service].request;
            return handler && handler(plugin, *request);
        } catch (...) {
            return 0;
        }
    }

    static void on_cancel(HakoRobotPlugin* robot, std::uint32_t service) noexcept
    {
        try {
            RobotPlugin& plugin = *reinterpret_cast<Class*>(robot);
            plugin.cancel(service);
        } catch (...) {
            // The C hook has no error channel.
        }
    }

    static std::int32_t on_step(HakoRobotPlugin* robot, const HakoTickV1* tick, const HakoRobotStateV1* state,
        HakoServiceSlotV1* slots, std::uint32_t slot_count, HakoRobotOutputV1* output,
        char* error, std::size_t size) noexcept
    {
        try {
            RobotPlugin& plugin = *reinterpret_cast<Class*>(robot);
            StepView view(*tick, *state, slots, slot_count, *output, plugin.step_storage_);
            plugin.step(view);
            return 0;
        } catch (const std::exception& exception) {
            write_error(error, size, exception.what());
        } catch (...) {
            write_error(error, size, "unknown exception in robot plugin step");
        }
        return 1;
    }

    static std::int32_t on_response(HakoRobotPlugin* robot, std::uint32_t service, const HakoTickV1* tick,
        const HakoRobotStateV1* state, const HakoServiceResponseV1* response) noexcept
    {
        try {
            RobotPlugin& plugin = *reinterpret_cast<Class*>(robot);
            if (service >= plugin.handlers_.size()) {
                return 0;
            }
            const auto& handler = plugin.handlers_[service].response;
            return handler && handler(plugin, StateView(*tick, *state), *response);
        } catch (...) {
            return 0;
        }
    }

    static void reset(HakoRobotPlugin* robot) noexcept
    {
        try {
            RobotPlugin& plugin = *reinterpret_cast<Class*>(robot);
            plugin.reset();
        } catch (...) {
            // The C hook has no error channel.
        }
    }
};

} // namespace detail
} // namespace hakoniwa::robot_runtime::plugin

#define HAKO_ROBOT_SERVICE(PKG, SRV) \
    struct SRV##Service { \
        using Request = HakoCpp_##SRV##Request; \
        using Response = HakoCpp_##SRV##Response; \
        using RequestPacket = HakoCpp_##SRV##RequestPacket; \
        using ResponsePacket = HakoCpp_##SRV##ResponsePacket; \
        using RequestConverter = ::hako::pdu::msgs::PKG::SRV##RequestPacket; \
        using ResponseConverter = ::hako::pdu::msgs::PKG::SRV##ResponsePacket; \
        using WireRequestPacket = Hako_##SRV##RequestPacket; \
        using WireResponsePacket = Hako_##SRV##ResponsePacket; \
        using Metadata = HakoPduMetaDataType; \
        static constexpr std::size_t base_offset = HAKO_PDU_META_DATA_SIZE(); \
        static constexpr std::string_view type = #PKG "/" #SRV; \
        static bool valid_metadata(const Metadata& metadata) { return !HAKO_PDU_METADATA_IS_INVALID(&metadata); } \
    }

#define HAKO_ROBOT_PLUGIN(Class, name, ...) \
    static constexpr HakoServicePluginV1 hako_robot_plugin_table_##name = \
        ::hakoniwa::robot_runtime::plugin::detail::PluginTable<Class __VA_OPT__(,) __VA_ARGS__>::make(#name); \
    HAKO_ROBOT_PLUGIN_ENTRY(name, hako_robot_plugin_table_##name)

#endif // HAKONIWA_ROBOT_RUNTIME_PLUGIN_ROBOT_PLUGIN_HPP
