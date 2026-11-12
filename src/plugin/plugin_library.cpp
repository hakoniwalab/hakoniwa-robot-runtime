#include "plugin/plugin_library.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace hakoniwa::robot_runtime::plugin {
namespace {

std::string kind_name(const std::uint32_t kind)
{
    if (kind == HAKO_PLUGIN_KIND_CONTROLLER) {
        return "controller";
    }
    if (kind == HAKO_PLUGIN_KIND_DATA_ADAPTER) {
        return "data adapter";
    }
    return "unknown kind " + std::to_string(kind);
}

/** True for the kinds this host can load. The data adapter is reserved. */
bool kind_supported(const std::uint32_t kind)
{
    return kind == HAKO_PLUGIN_KIND_CONTROLLER;
}

} // namespace

std::shared_ptr<PluginLibrary> PluginLibrary::open(
    const std::string& path,
    const std::uint32_t expected_kind)
{
    if (!kind_supported(expected_kind)) {
        throw std::runtime_error(
            "Robot Runtime plugin " + path + ": this host cannot load "
            + kind_name(expected_kind) + " plugins");
    }
    std::shared_ptr<PluginLibrary> library(new PluginLibrary(path));
    library->load(expected_kind);
    return library;
}

PluginLibrary::PluginLibrary(std::string path)
    : path_(std::move(path))
{
}

PluginLibrary::~PluginLibrary()
{
    close();
}

const std::string& PluginLibrary::path() const noexcept
{
    return path_;
}

const std::string& PluginLibrary::plugin_id() const noexcept
{
    return plugin_id_;
}

const HakoPluginHeaderV1& PluginLibrary::header() const noexcept
{
    return *header_;
}

const HakoControllerPluginV1& PluginLibrary::controller_table() const
{
    if (header_->kind != HAKO_PLUGIN_KIND_CONTROLLER) {
        throw std::logic_error(
            "Robot Runtime plugin " + path_ + " is not a controller plugin");
    }
    // The header is the first member of the controller table, so the header
    // address is the table address (checked by open()).
    return *reinterpret_cast<const HakoControllerPluginV1*>(header_);
}

void PluginLibrary::load(const std::uint32_t expected_kind)
{
    const std::string prefix = "Robot Runtime plugin " + path_;

#ifdef _WIN32
    handle_ = reinterpret_cast<void*>(LoadLibraryA(path_.c_str()));
    if (handle_ == nullptr) {
        throw std::runtime_error(
            "cannot load " + prefix + " (error "
            + std::to_string(GetLastError()) + ")");
    }
    auto entry = reinterpret_cast<HakoRobotRuntimePluginEntryV1>(
        GetProcAddress(
            static_cast<HMODULE>(handle_),
            HAKO_ROBOT_RUNTIME_PLUGIN_ENTRY_V1));
#else
    handle_ = dlopen(path_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
        const char* error = dlerror();
        throw std::runtime_error(
            "cannot load " + prefix + ": "
            + (error != nullptr ? error : "unknown error"));
    }
    auto entry = reinterpret_cast<HakoRobotRuntimePluginEntryV1>(
        dlsym(handle_, HAKO_ROBOT_RUNTIME_PLUGIN_ENTRY_V1));
#endif

    // From here on, every failure closes the library before throwing.
    try {
        if (entry == nullptr) {
            throw std::runtime_error(
                prefix + " has no entry " + HAKO_ROBOT_RUNTIME_PLUGIN_ENTRY_V1);
        }
        header_ = entry();
        if (header_ == nullptr) {
            throw std::runtime_error(prefix + " returned no plugin table");
        }
        if (header_->kind != expected_kind) {
            throw std::runtime_error(
                prefix + " is a " + kind_name(header_->kind)
                + " plugin; this component needs a "
                + kind_name(expected_kind) + " plugin");
        }
        plugin_id_ = header_->plugin_id != nullptr ? header_->plugin_id : "unknown";
        if (expected_kind == HAKO_PLUGIN_KIND_CONTROLLER) {
            validate_controller_table();
        }
    } catch (...) {
        close();
        throw;
    }
}

void PluginLibrary::validate_controller_table() const
{
    const std::string prefix = "Robot Runtime plugin " + path_;
    if (header_->abi_version != HAKO_CONTROLLER_PLUGIN_ABI_VERSION) {
        throw std::runtime_error(
            prefix + " has controller ABI version "
            + std::to_string(header_->abi_version) + "; this build needs "
            + std::to_string(HAKO_CONTROLLER_PLUGIN_ABI_VERSION)
            + " (rebuild the plugin against the same controller_plugin.h)");
    }
    if (header_->struct_size < sizeof(HakoControllerPluginV1)) {
        throw std::runtime_error(
            prefix + " has a controller table of "
            + std::to_string(header_->struct_size) + " bytes; this build needs at least "
            + std::to_string(sizeof(HakoControllerPluginV1)));
    }
    const auto& table = *reinterpret_cast<const HakoControllerPluginV1*>(header_);
    if (table.create == nullptr
        || table.update_period_ns == nullptr
        || table.on_start == nullptr
        || table.update == nullptr
        || table.on_stop == nullptr
        || table.reset == nullptr
        || table.destroy == nullptr) {
        throw std::runtime_error(
            prefix + " has a controller table with a null function");
    }
}

void PluginLibrary::close() noexcept
{
    if (handle_ == nullptr) {
        return;
    }
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(handle_));
#else
    dlclose(handle_);
#endif
    handle_ = nullptr;
    header_ = nullptr;
}

} // namespace hakoniwa::robot_runtime::plugin
