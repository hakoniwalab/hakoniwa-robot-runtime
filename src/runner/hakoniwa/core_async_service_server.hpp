#pragma once

#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE) && HAKONIWA_ROBOT_RUNTIME_ENABLE_ASYNC_SERVICE

#include "runtime/service/async_service_server.hpp"

namespace hakoniwa::robot_runtime::runner::hakoniwa {

/** Core-pro transport. The default instance uses caller-owned Core IDs. */
class CoreAsyncServiceServer final : public runtime::IAsyncServiceServer {
public:
    CoreAsyncServiceServer() = default;
    /** A configured instance uses indices into services until Core IDs can be created. */
    CoreAsyncServiceServer(std::string asset_name, std::vector<std::string> services);
    void prepare();

    runtime::AsyncServiceEvent poll(int service_id) noexcept override;
    std::optional<runtime::AsyncServiceState> status(int service_id) noexcept override;
    bool get_request(int service_id, std::vector<std::byte>& request) override;
    bool reply(
        int service_id, runtime::AsyncServiceResultCode code,
        const runtime::AsyncServiceResponseWriter& write) override;

private:
    [[nodiscard]] int core_id(int service_id) const noexcept;

    struct Registration {
        std::string name;
        int id {-1};
    };
    std::string asset_name_;
    std::vector<Registration> services_;
};

} // namespace hakoniwa::robot_runtime::runner::hakoniwa

#endif
