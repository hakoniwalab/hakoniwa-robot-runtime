// Contract tests for the sensor implementation catalog and the start-up check
// of sensor definition files. Uses its own CHECK so the tests also run in
// Release builds, where assert() is compiled out.
#include "runtime/sensor/sensor_catalog.hpp"

#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <vector>

using namespace hakoniwa::robot_runtime::runtime::sensor;
using json = nlohmann::json;

namespace {

int failures = 0;

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            ++failures;                                                         \
            std::cerr << __FILE__ << ":" << __LINE__ << ": CHECK failed: "      \
                      << #condition << std::endl;                               \
        }                                                                       \
    } while (false)

bool contains(const std::string& text, const std::string& part)
{
    return text.find(part) != std::string::npos;
}

const std::set<std::string> kAllLinked {"hakoniwa", "gazebo", "airsim"};

json sts3215()
{
    return json::parse(R"({
        "type": "joint",
        "update_rate": 100,
        "encoder": { "side": "joint", "bits": 12, "kind": "absolute" },
        "velocity": { "method": "difference", "window": 1, "lsb": "0.01465rpm" }
    })");
}

json gazebo_force_torque()
{
    return json::parse(R"({
        "type": "force_torque",
        "update_rate": 500,
        "noise": { "definition": "1sigma", "rate_hz": 500, "fxy": 0.2, "fz": 0.4 },
        "resolution": { "force": 0.1, "torque": 0.005 }
    })");
}

SensorCatalog load_shipped(const std::string& path)
{
    SensorCatalog catalog;
    std::string error;
    const bool loaded = load_sensor_catalog(path, catalog, &error);
    if (!loaded) {
        std::cerr << error << std::endl;
    }
    CHECK(loaded);
    return catalog;
}

void test_shipped_catalog_lists_the_three_implementations(const SensorCatalog& catalog)
{
    CHECK(catalog.implementations.count("hakoniwa") == 1);
    CHECK(catalog.implementations.count("gazebo") == 1);
    CHECK(catalog.implementations.count("airsim") == 1);
    CHECK(catalog.implementations.at("hakoniwa").count("joint") == 1);
    CHECK(catalog.implementations.at("hakoniwa").count("force_torque") == 1);
    CHECK(catalog.implementations.at("gazebo").count("joint") == 0);
}

void test_supported_definitions_pass(const SensorCatalog& catalog)
{
    std::string error;
    CHECK(validate_sensor_definition(catalog, kAllLinked, "hakoniwa", "joint", sts3215(), "sts.json", &error));
    CHECK(error.empty());
    auto with_schema = sts3215();
    with_schema["$schema"] = "https://example/sensor.schema.json";
    CHECK(validate_sensor_definition(catalog, kAllLinked, "hakoniwa", "joint", with_schema, "sts.json", &error));
    CHECK(validate_sensor_definition(
        catalog, kAllLinked, "gazebo", "force_torque", gazebo_force_torque(), "ft.json", &error));
}

void test_unknown_implementation_fails(const SensorCatalog& catalog)
{
    std::string error;
    CHECK(!validate_sensor_definition(catalog, kAllLinked, "gazebo2", "joint", sts3215(), "sts.json", &error));
    CHECK(contains(error, "sts.json"));
    CHECK(contains(error, "unknown sensor implementation \"gazebo2\""));
    CHECK(contains(error, "known: airsim, gazebo, hakoniwa"));
}

void test_unsupported_sensor_type_fails(const SensorCatalog& catalog)
{
    std::string error;
    CHECK(!validate_sensor_definition(catalog, kAllLinked, "gazebo", "joint", sts3215(), "sts.json", &error));
    CHECK(contains(error, "\"gazebo\" does not provide sensor type \"joint\""));

    error.clear();
    CHECK(!validate_sensor_definition(
        catalog, kAllLinked, "airsim", "force_torque", gazebo_force_torque(), "ft.json", &error));
    CHECK(contains(error, "\"airsim\" does not provide sensor type \"force_torque\""));
}

void test_unsupported_keys_fail_and_are_all_named(const SensorCatalog& catalog)
{
    auto definition = gazebo_force_torque();
    definition["nonlinearity"] = 0.002;
    definition["crosstalk"] = 0.01;
    std::string error;
    CHECK(!validate_sensor_definition(catalog, kAllLinked, "gazebo", "force_torque", definition, "ft.json", &error));
    CHECK(contains(error, "gazebo/force_torque does not support: crosstalk, nonlinearity"));

    // The same keys are supported by another implementation.
    error.clear();
    CHECK(validate_sensor_definition(catalog, kAllLinked, "hakoniwa", "force_torque", definition, "ft.json", &error));
}

void test_misspelt_key_fails(const SensorCatalog& catalog)
{
    auto definition = sts3215();
    definition["encodr"] = json::object();
    std::string error;
    CHECK(!validate_sensor_definition(catalog, kAllLinked, "hakoniwa", "joint", definition, "sts.json", &error));
    CHECK(contains(error, "hakoniwa/joint does not support: encodr"));
}

void test_definition_type_is_optional_but_must_match(const SensorCatalog& catalog)
{
    // Some formats (the drone_config IMU section) have no "type"; the sensor
    // instance gives it.
    auto definition = sts3215();
    definition.erase("type");
    std::string error;
    CHECK(validate_sensor_definition(catalog, kAllLinked, "hakoniwa", "joint", definition, "sts.json", &error));

    // A definition that has a type must agree with the sensor instance.
    error.clear();
    CHECK(!validate_sensor_definition(catalog, kAllLinked, "hakoniwa", "force_torque", sts3215(), "sts.json", &error));
    CHECK(contains(error, "type does not match the sensor type \"force_torque\""));
}

void test_catalog_entry_not_linked_fails(const SensorCatalog& catalog)
{
    std::string error;
    CHECK(!validate_sensor_definition(catalog, {"gazebo"}, "hakoniwa", "joint", sts3215(), "sts.json", &error));
    CHECK(contains(error, "\"hakoniwa\" is in the catalog but not linked"));
}

void test_unsupported_keys_are_reported_before_linking(const SensorCatalog& catalog)
{
    auto definition = sts3215();
    definition["encodr"] = 1;
    std::string error;
    CHECK(!validate_sensor_definition(catalog, {}, "hakoniwa", "joint", definition, "sts.json", &error));
    CHECK(contains(error, "does not support: encodr"));
}

void test_key_path_rules()
{
    const auto definition = json::parse(R"({
        "type": "joint",
        "noise": { "a": 1, "b": { "c": 2 } },
        "noise_counts": 3,
        "encoder": { "bits": 12, "kind": "absolute", "detail": { "x": 1, "y": 2 } },
        "list": [ { "z": 1 } ]
    })");
    // "noise" covers its subtree but not the sibling "noise_counts".
    // "encoder.bits" and "encoder.detail.x" select single keys; arrays are leaves.
    const auto unsupported = find_unsupported_keys(
        {"noise", "encoder.bits", "encoder.detail.x", "list"}, definition);
    const std::vector<std::string> expected {"encoder.detail.y", "encoder.kind", "noise_counts"};
    CHECK(unsupported == expected);
    if (unsupported != expected) {
        for (const auto& key : unsupported) {
            std::cerr << "  unsupported: " << key << std::endl;
        }
    }
}

void test_invalid_catalogs_are_rejected()
{
    SensorCatalog catalog;
    std::string error;
    CHECK(!parse_sensor_catalog(json::object(), catalog, &error));
    CHECK(contains(error, "\"implementations\" object"));

    CHECK(!parse_sensor_catalog(
        json::parse(R"({"implementations": {"x": {}}})"), catalog, &error));
    CHECK(contains(error, "\"x\" requires a \"sensors\" object"));

    CHECK(!parse_sensor_catalog(
        json::parse(R"({"implementations": {"x": {"sensors": {"joint": {}}}}})"), catalog, &error));
    CHECK(contains(error, "\"x/joint\" requires a \"keys\" array"));

    for (const auto* bad : {"", ".a", "a.", "a..b"}) {
        json value = json::parse(R"({"implementations": {"x": {"sensors": {"joint": {"keys": []}}}}})");
        value["implementations"]["x"]["sensors"]["joint"]["keys"].push_back(bad);
        error.clear();
        CHECK(!parse_sensor_catalog(value, catalog, &error));
        CHECK(contains(error, "invalid key path"));
    }

    CHECK(!load_sensor_catalog("/nonexistent/sensor-implementations.json", catalog, &error));
    CHECK(contains(error, "cannot open sensor catalog"));
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: test_sensor_catalog <catalog/sensor-implementations.json>" << std::endl;
        return 2;
    }
    const auto catalog = load_shipped(argv[1]);
    test_shipped_catalog_lists_the_three_implementations(catalog);
    test_supported_definitions_pass(catalog);
    test_unknown_implementation_fails(catalog);
    test_unsupported_sensor_type_fails(catalog);
    test_unsupported_keys_fail_and_are_all_named(catalog);
    test_misspelt_key_fails(catalog);
    test_definition_type_is_optional_but_must_match(catalog);
    test_catalog_entry_not_linked_fails(catalog);
    test_unsupported_keys_are_reported_before_linking(catalog);
    test_key_path_rules();
    test_invalid_catalogs_are_rejected();
    if (failures != 0) {
        std::cerr << failures << " check(s) failed" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "test_sensor_catalog: all checks passed" << std::endl;
    return EXIT_SUCCESS;
}
