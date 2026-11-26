#include "factory/manifest_factory_internal.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    using namespace hakoniwa::robot_runtime::factory::detail;
    if (argc != 2) {
        return 1;
    }
    ResolvedManifest manifest;
    std::string error;
    if (resolve_manifest(argv[1], manifest, &error)
        || error.find("'services' was replaced by 'robots'") == std::string::npos) {
        std::cerr << "Expected the root services replacement diagnostic, got: " << error << '\n';
        return 1;
    }
    return 0;
}
