#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <string>
#include <toml.hpp>

class ConfigLoader
{
public:
    double loop_closure_MOE;
    double margin_of_error;
    double cone_detection_MOE_rate;
    double pred_future_cone_range;
    double pred_future_cone_arc;
    double car_start_pos_x;
    double car_start_pos_y;
    std::string track;
    int max_while_loop_iterations;
    double cone_pos_variance_multiplier;
    int reload_iterations;

    ConfigLoader(const std::string &configPath);
};

#endif // CONFIG_LOADER_H
