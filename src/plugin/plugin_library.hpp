#pragma once

#include "hakoniwa/robot_runtime/plugin/controller_plugin.h"
#include "hakoniwa/robot_runtime/plugin/plugin_common.h"

#include <cstdint>
#include <memory>
#include <string>

namespace hakoniwa::robot_runtime::plugin {

/**
 * One loaded Robot Runtime plugin library (any kind).
 *
 * open() loads the shared library, resolves the common entry
 * (HAKO_ROBOT_RUNTIME_PLUGIN_ENTRY_V1) and validates the table header for the
 * kind the caller expects: the header is non-null, `kind` equals the expected
 * kind and is known to this host, `abi_version` is the version of that kind
 * this host was built with, and `struct_size` covers the kind's table as this
 * host knows it. Any failure throws std::runtime_error naming the path.
 *
 * The library is closed when the last shared_ptr goes away. Every object
 * created from the library (a plugin instance) holds the shared_ptr, so the
 * code is never unloaded under a live instance.
 */
class PluginLibrary final {
public:
    /** @throws std::runtime_error when loading or validation fails. */
    [[nodiscard]] static std::shared_ptr<PluginLibrary> open(
        const std::string& path,
        std::uint32_t expected_kind);

    ~PluginLibrary();

    PluginLibrary(const PluginLibrary&) = delete;
    PluginLibrary& operator=(const PluginLibrary&) = delete;

    [[nodiscard]] const std::string& path() const noexcept;
    /** plugin_id from the header, or "unknown" when the plugin gave none. */
    [[nodiscard]] const std::string& plugin_id() const noexcept;
    [[nodiscard]] const HakoPluginHeaderV1& header() const noexcept;

    /**
     * The controller table. Valid only for a library opened with
     * HAKO_PLUGIN_KIND_CONTROLLER (open() has checked kind and size).
     * @throws std::logic_error for another kind.
     */
    [[nodiscard]] const HakoControllerPluginV1& controller_table() const;

private:
    explicit PluginLibrary(std::string path);

    void load(std::uint32_t expected_kind);
    void validate_controller_table() const;
    void close() noexcept;

    std::string path_;
    std::string plugin_id_;
    void* handle_ {nullptr};
    const HakoPluginHeaderV1* header_ {nullptr};
};

} // namespace hakoniwa::robot_runtime::plugin
