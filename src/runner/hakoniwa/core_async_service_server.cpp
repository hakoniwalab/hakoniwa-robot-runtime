#include "runner/hakoniwa/core_async_service_server.hpp"


#include "hako_asset_service.h"

#include <stdexcept>
#include <utility>

namespace hakoniwa::robot_runtime::runner::hakoniwa {
namespace {

int core_result_code(runtime::AsyncServiceResultCode code) noexcept
{
    switch (code) {
    case runtime::AsyncServiceResultCode::Ok:
        return HAKO_SERVICE_API_RESULT_CODE_OK;
    case runtime::AsyncServiceResultCode::Invalid:
        return HAKO_SERVICE_API_RESULT_CODE_INVALID;
    case runtime::AsyncServiceResultCode::Canceled:
        return HAKO_SERVICE_API_RESULT_CODE_CANCELED;
    case runtime::AsyncServiceResultCode::Error:
        return HAKO_SERVICE_API_RESULT_CODE_ERROR;
    }
    return HAKO_SERVICE_API_RESULT_CODE_ERROR;
}

} // namespace

CoreAsyncServiceServer::CoreAsyncServiceServer(
    std::string asset_name, std::vector<std::string> services)
    : asset_name_(std::move(asset_name))
{
    if (asset_name_.empty() || services.empty()) {
        throw std::invalid_argument("Core async services need an asset and service names");
    }
    services_.reserve(services.size());
    for (auto& service : services) {
        services_.push_back({std::move(service)});
    }
}

void CoreAsyncServiceServer::prepare()
{
    for (auto& service : services_) {
        if (service.id < 0) {
            service.id = hako_asset_service_server_create(asset_name_.c_str(), service.name.c_str());
            if (service.id < 0) {
                throw std::runtime_error("failed to create Core async service " + service.name);
            }
        }
    }
}

int CoreAsyncServiceServer::core_id(int service_id) const noexcept
{
    if (services_.empty()) {
        return service_id;
    }
    if (service_id < 0 || static_cast<std::size_t>(service_id) >= services_.size()) {
        return -1;
    }
    return services_[static_cast<std::size_t>(service_id)].id;
}

runtime::AsyncServiceEvent CoreAsyncServiceServer::poll(int service_id) noexcept
{
    service_id = core_id(service_id);
    if (service_id < 0) {
        return runtime::AsyncServiceEvent::Error;
    }
    switch (hako_asset_service_server_poll(service_id)) {
    case HAKO_SERVICE_SERVER_API_EVENT_NONE:
        return runtime::AsyncServiceEvent::None;
    case HAKO_SERVICE_SERVER_API_REQUEST_IN:
        return runtime::AsyncServiceEvent::Request;
    case HAKO_SERVICE_SERVER_API_REQUEST_CANCEL:
        return runtime::AsyncServiceEvent::Cancel;
    default:
        return runtime::AsyncServiceEvent::Error;
    }
}

std::optional<runtime::AsyncServiceState> CoreAsyncServiceServer::status(int service_id) noexcept
{
    int state = 0;
    service_id = core_id(service_id);
    if (service_id < 0 || hako_asset_service_server_status(service_id, &state) != 0) {
        return std::nullopt;
    }
    switch (state) {
    case HAKO_SERVICE_SERVER_API_STATUS_IDLE:
        return runtime::AsyncServiceState::Idle;
    case HAKO_SERVICE_SERVER_API_STATUS_DOING:
        return runtime::AsyncServiceState::Doing;
    case HAKO_SERVICE_SERVER_API_STATUS_CANCELING:
        return runtime::AsyncServiceState::Canceling;
    default:
        return std::nullopt;
    }
}

bool CoreAsyncServiceServer::get_request(int service_id, std::vector<std::byte>& request)
{
    char* packet = nullptr;
    std::size_t size = 0;
    service_id = core_id(service_id);
    if (service_id < 0 || hako_asset_service_server_get_request(service_id, &packet, &size) != 0
        || packet == nullptr) {
        return false;
    }
    const auto* first = reinterpret_cast<const std::byte*>(packet);
    request.assign(first, first + size);
    return true;
}

bool CoreAsyncServiceServer::reply(
    int service_id, runtime::AsyncServiceResultCode code,
    const runtime::AsyncServiceResponseWriter& write)
{
    char* packet = nullptr;
    std::size_t size = 0;
    service_id = core_id(service_id);
    if (service_id < 0 || hako_asset_service_server_get_response_buffer(
            service_id, &packet, &size, HAKO_SERVICE_API_STATUS_DONE,
            core_result_code(code)) != 0
        || packet == nullptr) {
        return false;
    }

    if (write && !write({reinterpret_cast<std::byte*>(packet), size})) {
        return false;
    }
    return hako_asset_service_server_put_response(service_id, packet, size) == 0;
}

} // namespace hakoniwa::robot_runtime::runner::hakoniwa
