#include "ackermann_step_service_fixture.hpp"
#include "runner/hakoniwa/hakoniwa_asset_driver.hpp"

#include "hako_asset_service.h"

using namespace ackermann_step_fixture;
using namespace hakoniwa::robot_runtime::runner::detail;

namespace {

class Callbacks final : public IHakoniwaAssetDriverCallbacks {
public:
    explicit Callbacks(const std::vector<std::string>& vehicles) : fixture_(vehicles) {}

    int on_initialize() noexcept override { return fixture_.on_initialize(); }
    bool sleep_before_first_step() const noexcept override { return true; }
    bool ready_to_step() noexcept override { return fixture_.ready_to_step(); }
    int on_step() noexcept override { return fixture_.on_step(); }
    int on_reset() noexcept override { return 0; }
    bool should_stop() const noexcept override { return fixture_.failed(); }
    int stop_result() const noexcept override { return fixture_.failed() ? -1 : 0; }

private:
    RuntimeFixture fixture_;
};

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3) return 2;
    std::vector<std::string> vehicles(argv + 3, argv + argc);
    if (vehicles.empty()) vehicles.emplace_back("A");
    Callbacks callbacks(vehicles);
    HakoniwaAssetDriver driver({kAsset, argv[1], kDelta, HAKO_ASSET_MODEL_PLANT, true}, callbacks);
    if (driver.register_asset() != 0 || hako_asset_service_initialize(argv[2]) != 0) {
        return 1;
    }
    std::cout << "SERVER registered" << std::endl;
    return driver.start() == 0 && !callbacks.should_stop() ? 0 : 1;
}
