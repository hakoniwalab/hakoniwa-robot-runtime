#include "runtime/service/async_service_pipeline.hpp"

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include <exception>
#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runtime {
namespace {

class AsyncServiceInput final : public IControllerInput {
public:
    AsyncServiceInput(
        std::string_view source_id, std::uint64_t sequence, std::uint64_t time_usec,
        AsyncServiceState state, std::string error)
        : metadata_ {std::string(source_id), sequence, time_usec, std::nullopt}
        , state_(state)
        , error_(std::move(error))
    {
    }

    const ControllerInputMetadata& metadata() const noexcept override { return metadata_; }
    std::string_view type_name() const noexcept override { return "async-service-event"; }
    AsyncServiceState state() const noexcept { return state_; }
    const std::string& error() const noexcept { return error_; }

private:
    ControllerInputMetadata metadata_;
    AsyncServiceState state_;
    std::string error_;
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
    std::shared_ptr<IAsyncService> service, bool lockstep)
    : id_(std::move(id)), service_id_(service_id), server_(std::move(server))
    , service_(std::move(service)), lockstep_(lockstep)
{
    if (id_.empty() || service_id_ < 0 || !server_ || !service_) {
        throw std::invalid_argument("async service source needs an ID, server and service");
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
        if (!staged_request_) {
            switch (server_->poll(service_id_)) {
            case AsyncServiceEvent::None:
                return false;
            case AsyncServiceEvent::Request: {
                if (expected_state_ != AsyncServiceState::Idle) {
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
            case AsyncServiceEvent::Cancel:
                lockstep_failure_ = "request was canceled";
                return false;
            default:
                lockstep_failure_ = "failed to poll server";
                return false;
            }
        }
        // A staged request already owns this service. Check the state without
        // polling again, which could consume another event or send BUSY.
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
        if (!service_->on_request(request)) {
            expected_state_ = AsyncServiceState::Doing;
            completion_ = AsyncServiceResultCode::Invalid;
            return {nullptr, {id_, ComponentState::Ready, {}}};
        }
    } else if (expected_state_ == AsyncServiceState::Doing) {
        service_->on_cancel();
    }
    const auto status = error.empty() ? ComponentState::Ready : ComponentState::Error;
    auto input = std::make_shared<AsyncServiceInput>(
        id_, ++sequence_, context.simulation_time_usec, next_state, error);
    return {std::move(input), {id_, status, std::move(error)}};
}

void AsyncServiceCommandSource::reset() noexcept
{
    // A local plant reset can happen after admission and before the source poll.
    // Keep the staged request until the step takes it or Core discards the run.
    sequence_ = 0;
}

AsyncServiceController::AsyncServiceController(
    std::string id, std::shared_ptr<AsyncServiceCommandSource> source)
    : id_(std::move(id)), source_(std::move(source))
{
    if (!source_) {
        throw std::invalid_argument("async service controller needs a source");
    }
}

std::string_view AsyncServiceController::id() const noexcept { return id_; }
std::string_view AsyncServiceController::source_id() const noexcept { return source_->id(); }

bool AsyncServiceController::accepts(const IControllerInput& input) const noexcept
{
    return dynamic_cast<const AsyncServiceInput*>(&input) != nullptr;
}

bool AsyncServiceController::error_stops_run() const noexcept { return source_->lockstep_; }

ControllerOutput AsyncServiceController::update(
    std::shared_ptr<const IControllerInput> input, const RobotState& state,
    const RuntimeStepContext& context)
{
    ControllerOutput output {id_, {id_, ComponentState::WaitingForInput, {}}, {}};
    const auto* service_input = dynamic_cast<const AsyncServiceInput*>(input.get());
    if (service_input && !service_input->error().empty()) {
        output.status = {id_, ComponentState::Error, service_input->error()};
        return output;
    }

    const auto expected = service_input ? service_input->state() : source_->expected_state_;
    const auto actual = source_->server_->status(source_->service_id_);
    if (actual != expected) {
        output.status = {
            id_, ComponentState::Error,
            state_mismatch(expected, actual),
        };
        if (source_->completion_) {
            source_->completion_.reset();
            source_->expected_state_ = AsyncServiceState::Idle;
        }
        return output;
    }
    if (source_->completion_ || expected == AsyncServiceState::Idle) {
        return output;
    }

    source_->expected_state_ = expected;
    auto result = source_->service_->on_step(state, *actual, context);
    switch (result.outcome) {
    case AsyncServiceStepOutcome::Pending:
        if (source_->lockstep_) {
            output.status = {
                id_, ComponentState::Error, "lockstep service must complete in the request step",
            };
            return output;
        }
        break;
    case AsyncServiceStepOutcome::Ok:
        source_->completion_ = AsyncServiceResultCode::Ok;
        break;
    case AsyncServiceStepOutcome::Canceled:
        source_->completion_ = AsyncServiceResultCode::Canceled;
        break;
    case AsyncServiceStepOutcome::Error:
        source_->completion_ = AsyncServiceResultCode::Error;
        break;
    }
    // Even the terminal substep can supply a command. The next one waits
    // for publication without invoking any service hook.
    for (auto& command : result.commands) {
        command.source_id = source_->id();
    }
    output.status.state = ComponentState::Ready;
    output.commands = std::move(result.commands);
    return output;
}

void AsyncServiceController::reset(const RobotState&)
{
    source_->completion_.reset();
    source_->expected_state_ = AsyncServiceState::Idle;
    source_->service_->reset();
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
                return source_->service_->on_response(state, context, packet);
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

#endif
