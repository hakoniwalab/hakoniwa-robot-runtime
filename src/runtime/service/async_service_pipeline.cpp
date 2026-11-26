#include "runtime/service/async_service_pipeline.hpp"


#include <algorithm>
#include <exception>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

class AsyncServiceInput final : public IControllerInput {
public:
    AsyncServiceInput(
        std::string_view source_id, std::uint64_t sequence, std::uint64_t time_usec,
        AsyncServiceState state)
        : metadata_ {std::string(source_id), sequence, time_usec, std::nullopt}
        , state_(state)
    {
    }

    const ControllerInputMetadata& metadata() const noexcept override { return metadata_; }
    std::string_view type_name() const noexcept override { return "async-service-event"; }
    AsyncServiceState state() const noexcept { return state_; }

private:
    ControllerInputMetadata metadata_;
    AsyncServiceState state_;
};

class RobotServiceInput final : public IControllerInput {
public:
    RobotServiceInput(std::string_view source_id, std::uint64_t sequence, std::uint64_t time_usec,
        std::vector<CommandSourcePollResult> events)
        : metadata_ {std::string(source_id), sequence, time_usec, std::nullopt}
        , events_(std::move(events))
    {
    }

    const ControllerInputMetadata& metadata() const noexcept override { return metadata_; }
    std::string_view type_name() const noexcept override { return "robot-service-events"; }
    const CommandSourcePollResult& event(std::size_t index) const { return events_[index]; }

private:
    ControllerInputMetadata metadata_;
    std::vector<CommandSourcePollResult> events_;
};

const char* state_name(AsyncServiceState state) noexcept
{
    switch (state) {
    case AsyncServiceState::Idle: return "IDLE";
    case AsyncServiceState::Doing: return "DOING";
    case AsyncServiceState::Canceling: return "CANCELING";
    }
    return "unknown";
}

std::string state_mismatch(
    AsyncServiceState expected, std::optional<AsyncServiceState> actual)
{
    return std::string("service state mismatch: expected ") + state_name(expected)
        + ", got " + (actual ? state_name(*actual) : "unavailable");
}

} // namespace

AsyncServiceCommandSource::AsyncServiceCommandSource(
    std::string id, int service_id, std::shared_ptr<IAsyncServiceServer> server,
    std::shared_ptr<IRobotServices> robot, std::size_t service_index, bool lockstep)
    : id_(std::move(id)), service_id_(service_id), server_(std::move(server))
    , robot_(std::move(robot)), service_index_(service_index), lockstep_(lockstep)
{
    if (id_.empty() || service_id_ < 0 || !server_ || !robot_) {
        throw std::invalid_argument("async service source needs an ID, server and robot");
    }
#if !defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP) || !HAKONIWA_ROBOT_RUNTIME_ENABLE_LOCKSTEP
    if (lockstep_) {
        throw std::invalid_argument("async service lockstep is disabled: " + id_);
    }
#endif
}

std::string_view AsyncServiceCommandSource::id() const noexcept { return id_; }

void AsyncServiceCommandSource::lockstep_prepare() noexcept
{
    staged_request_.reset();
    lockstep_failure_.clear();
}

bool AsyncServiceCommandSource::lockstep_ready() noexcept
{
    if (!lockstep_) {
        return true;
    }
    if (!lockstep_failure_.empty()) {
        return false;
    }

    try {
        switch (server_->poll(service_id_)) {
        case AsyncServiceEvent::None:
            if (!staged_request_) {
                return false;
            }
            break;
        case AsyncServiceEvent::Request: {
            if (staged_request_ || expected_state_ != AsyncServiceState::Idle) {
                lockstep_failure_ = "service request arrived before the previous response";
                return false;
            }
            std::vector<std::byte> request;
            if (!server_->get_request(service_id_, request)) {
                lockstep_failure_ = "failed to read request";
                return false;
            }
            staged_request_ = std::move(request);
            break;
        }
        case AsyncServiceEvent::Cancel: {
            // The gate owns a staged request until the source takes it into a step.
            if (!staged_request_) {
                lockstep_failure_ = "cancel arrived without a staged request";
                return false;
            }
            const auto actual = server_->status(service_id_);
            if (actual != AsyncServiceState::Canceling) {
                lockstep_failure_ = state_mismatch(AsyncServiceState::Canceling, actual);
            } else if (!server_->reply(service_id_, AsyncServiceResultCode::Canceled, {})) {
                lockstep_failure_ = "failed to reply to staged request cancellation";
            } else {
                staged_request_.reset();
            }
            return false;
        }
        default:
            lockstep_failure_ = "failed to poll server";
            return false;
        }
        const auto actual = server_->status(service_id_);
        if (actual == AsyncServiceState::Doing) {
            return true;
        }
        lockstep_failure_ = state_mismatch(AsyncServiceState::Doing, actual);
    } catch (const std::exception& error) {
        lockstep_failure_ = error.what();
        if (lockstep_failure_.empty()) {
            lockstep_failure_ = "failed to stage request";
        }
    } catch (...) {
        lockstep_failure_ = "failed to stage request";
    }
    return false;
}

void AsyncServiceCommandSource::lockstep_poll_while_paused() noexcept
{
    if (staged_request_) {
        (void)lockstep_ready();
    }
}

CommandSourcePollResult AsyncServiceCommandSource::poll(const RuntimeStepContext& context)
{
    const auto event = lockstep_
        ? (staged_request_ ? AsyncServiceEvent::Request : AsyncServiceEvent::None)
        : server_->poll(service_id_);
    if (event == AsyncServiceEvent::None
        || (event == AsyncServiceEvent::Cancel
            && (expected_state_ == AsyncServiceState::Idle || completion_))) {
        return {nullptr, {id_, ComponentState::WaitingForInput, {}}};
    }
    if (event == AsyncServiceEvent::Error) {
        return {nullptr, {id_, ComponentState::Error, "failed to poll service server"}};
    }
    if (event != AsyncServiceEvent::Request && event != AsyncServiceEvent::Cancel) {
        throw std::logic_error("unknown async service event");
    }

    std::vector<std::byte> request;
    if (event == AsyncServiceEvent::Request) {
        if (lockstep_) {
            request = std::move(*staged_request_);
            staged_request_.reset();
        } else if (!server_->get_request(service_id_, request)) {
            return {nullptr, {id_, ComponentState::Error, "failed to read service request"}};
        }
    }

    const auto next_state = event == AsyncServiceEvent::Request
        ? AsyncServiceState::Doing : AsyncServiceState::Canceling;
    const auto actual = server_->status(service_id_);
    std::string error;
    if (event == AsyncServiceEvent::Request && expected_state_ != AsyncServiceState::Idle) {
        error = "service request arrived while another is active";
    } else if (actual != next_state) {
        // Carry the error to the controller so the existing lockstep
        // fail-stop path runs before physics, without calling the hook.
        error = state_mismatch(next_state, actual);
    } else if (event == AsyncServiceEvent::Request) {
        if (!robot_->on_request(service_index_, request)) {
            expected_state_ = AsyncServiceState::Doing;
            completion_ = AsyncServiceResultCode::Invalid;
            return {nullptr, {id_, ComponentState::Ready, {}}};
        }
    } else if (expected_state_ == AsyncServiceState::Doing) {
        robot_->on_cancel(service_index_);
    }
    const auto status = error.empty() ? ComponentState::Ready : ComponentState::Error;
    auto input = std::make_shared<AsyncServiceInput>(
        id_, ++sequence_, context.simulation_time_usec, next_state);
    return {std::move(input), {id_, status, std::move(error)}};
}

void AsyncServiceCommandSource::reset() noexcept
{
    // A local plant reset can happen after admission and before the source poll.
    // Keep the staged request until the step takes it or Core discards the run.
    sequence_ = 0;
}

RobotServiceSource::RobotServiceSource(
    std::string id, std::vector<std::shared_ptr<AsyncServiceCommandSource>> services)
    : id_(std::move(id)), services_(std::move(services))
{
    if (id_.empty() ||
        std::any_of(services_.begin(), services_.end(), [](const auto& service) { return !service; })) {
        throw std::invalid_argument("robot service source needs an ID and non-null service sources");
    }
}

std::string_view RobotServiceSource::id() const noexcept { return id_; }

CommandSourcePollResult RobotServiceSource::poll(const RuntimeStepContext& context)
{
    std::vector<CommandSourcePollResult> events;
    events.reserve(services_.size());
    ComponentStatus status {id_, ComponentState::WaitingForInput, {}};
    for (const auto& service : services_) {
        auto event = service->poll(context);
        if (status.state != ComponentState::Error) {
            if (event.status.state == ComponentState::Error) {
                status.state = ComponentState::Error;
                status.detail = event.status.detail;
            } else if (event.status.state == ComponentState::Ready) {
                status.state = ComponentState::Ready;
            }
        }
        events.push_back(std::move(event));
    }
    if (status.state == ComponentState::WaitingForInput) {
        return {nullptr, std::move(status)};
    }
    auto input = std::make_shared<RobotServiceInput>(
        id_, ++sequence_, context.simulation_time_usec, std::move(events));
    return {std::move(input), std::move(status)};
}

void RobotServiceSource::reset() noexcept
{
    for (const auto& service : services_) {
        service->reset();
    }
    sequence_ = 0;
}

RobotServiceController::RobotServiceController(
    std::string id, std::shared_ptr<RobotServiceSource> source, std::shared_ptr<IRobotServices> robot)
    : id_(std::move(id)), source_(std::move(source)), robot_(std::move(robot))
{
    if (!source_ || !robot_) {
        throw std::invalid_argument("robot service controller needs a source and robot");
    }
    slots_.resize(source_->services_.size());
}

std::string_view RobotServiceController::id() const noexcept { return id_; }
std::string_view RobotServiceController::source_id() const noexcept { return source_->id(); }

bool RobotServiceController::accepts(const IControllerInput& input) const noexcept
{
    return dynamic_cast<const RobotServiceInput*>(&input) != nullptr;
}

bool RobotServiceController::error_stops_run() const noexcept
{
    return std::any_of(source_->services_.begin(), source_->services_.end(),
        [](const auto& service) { return service->lockstep_; });
}

ControllerOutput RobotServiceController::update(
    std::shared_ptr<const IControllerInput> input, const RobotState& state,
    const RuntimeStepContext& context)
{
    ControllerOutput output {id_, {id_, ComponentState::WaitingForInput, {}}, {}};
    const auto record_error = [&](std::string detail) {
        if (output.status.state != ComponentState::Error) {
            output.status.state = ComponentState::Error;
            output.status.detail = std::move(detail);
        }
    };
    const auto* batch = dynamic_cast<const RobotServiceInput*>(input.get());
    for (std::size_t i = 0; i < slots_.size(); ++i) {
        auto& service = *source_->services_[i];
        slots_[i] = {};
        const auto* event = batch ? &batch->event(i) : nullptr;
        if (event && event->status.state == ComponentState::Error) {
            record_error(event->status.detail);
            continue;
        }
        const auto* service_input = event ? static_cast<const AsyncServiceInput*>(event->input.get()) : nullptr;
        const auto expected = service_input ? service_input->state() : service.expected_state_;
        const auto actual = service.server_->status(service.service_id_);
        if (actual != expected) {
            record_error(state_mismatch(expected, actual));
            if (service.completion_) {
                service.completion_.reset();
                service.expected_state_ = AsyncServiceState::Idle;
            }
            continue;
        }
        if (!service.completion_) {
            service.expected_state_ = expected;
            slots_[i].state = expected;
        }
    }

    auto result = robot_->on_step(state, context, slots_);
    bool active = false;
    for (std::size_t i = 0; i < slots_.size(); ++i) {
        const auto& slot = slots_[i];
        if (slot.state == AsyncServiceState::Idle) {
            continue;
        }
        active = true;
        auto& service = *source_->services_[i];
        switch (slot.result) {
        case AsyncServiceStepOutcome::Pending:
            if (service.lockstep_) {
                record_error("lockstep service must complete in the request step");
            }
            break;
        case AsyncServiceStepOutcome::Ok:
            service.completion_ = AsyncServiceResultCode::Ok;
            break;
        case AsyncServiceStepOutcome::Canceled:
            service.completion_ = AsyncServiceResultCode::Canceled;
            break;
        case AsyncServiceStepOutcome::Error:
            service.completion_ = AsyncServiceResultCode::Error;
            break;
        }
    }
    for (auto& command : result.commands) {
        command.source_id = source_->id();
    }
    if (output.status.state != ComponentState::Error) {
        if (!result.error.empty()) {
            output.status.state = result.commands.empty() ? ComponentState::Error : ComponentState::Degraded;
            output.status.detail = std::move(result.error);
        } else if (active || !result.commands.empty()) {
            output.status.state = ComponentState::Ready;
        }
    }
    output.commands = std::move(result.commands);
    return output;
}

void RobotServiceController::reset(const RobotState&)
{
    for (const auto& service : source_->services_) {
        service->completion_.reset();
        service->expected_state_ = AsyncServiceState::Idle;
    }
    robot_->reset();
}

AsyncServiceResponsePublisher::AsyncServiceResponsePublisher(
    std::string id, std::shared_ptr<AsyncServiceCommandSource> source)
    : id_(std::move(id)), source_(std::move(source))
{
    if (!source_) {
        throw std::invalid_argument("async service publisher needs a source");
    }
}

std::string_view AsyncServiceResponsePublisher::id() const noexcept { return id_; }

ComponentStatus AsyncServiceResponsePublisher::publish(
    const RobotState& state, const RuntimeStepContext& context) noexcept
{
    auto completion = std::exchange(source_->completion_, std::nullopt);
    if (!completion) {
        return {id_, ComponentState::WaitingForInput, {}};
    }
    source_->expected_state_ = AsyncServiceState::Idle;
    try {
        AsyncServiceResponseWriter write;
        if (*completion != AsyncServiceResultCode::Invalid) {
            write = [&](std::span<std::byte> packet) {
                return source_->robot_->on_response(source_->service_index_, state, context, packet);
            };
        }
        if (source_->server_->reply(source_->service_id_, *completion, write)) {
            return {id_, ComponentState::Ready, {}};
        }
        return {id_, ComponentState::Error, "failed to write or send service reply"};
    } catch (const std::exception& error) {
        return {id_, ComponentState::Error, error.what()};
    } catch (...) {
        return {id_, ComponentState::Error, "service reply raised an unknown exception"};
    }
}

void AsyncServiceResponsePublisher::reset() noexcept {}

} // namespace hakoniwa::robot_runtime::runtime
