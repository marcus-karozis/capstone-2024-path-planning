#include "ConfigLoader.h"
#include <iostream>
#include <stdexcept>

ConfigLoader::ConfigLoader(const std::string &configPath)
{
    try
    {
        auto config = toml::parse(configPath);

        // Load config values
        loop_closure_MOE = toml::find<double>(config, "loop_closure_MOE");
        margin_of_error = toml::find<double>(config, "margin_of_error");
        cone_detection_MOE_rate = toml::find<double>(config, "cone_detection_MOE_rate");
        pred_future_cone_range = toml::find<double>(config, "pred_future_cone_range");
        pred_future_cone_arc = toml::find<double>(config, "pred_future_cone_arc");
        car_start_pos_x = toml::find<double>(config, "car_start_pos_x");
        car_start_pos_y = toml::find<double>(config, "car_start_pos_y");
        track = toml::find<std::string>(config, "track");
        max_wile_loop_iterations = toml::find<int>(config, "max_wile_loop_iterations");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to load config file: " << e.what() << std::endl;
        throw;
    }
}
