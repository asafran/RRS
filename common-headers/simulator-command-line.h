#ifndef		SIMULATOR_COMMAND_LINE
#define		SIMULATOR_COMMAND_LINE

#include    "command-line.h"
#include    <optional>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct simulator_init_t
{
    /// Train configuration file name
    std::optional<QString>   train_config;
    /// Route directory
    std::optional<QString>   route_dir;
    /// Clear simulator log file
    std::optional<bool>      clear_log;
    /// Debug print in simulation loop
    std::optional<bool>      debug_print;
    /// Initial railway coordinate
    std::optional<double>    init_coord;
    /// Initial direction
    std::optional<int>       direction;
    /// Initial trajectory name
    std::optional<QString>   traj_name;
    /// Initial trajectory coordinate
    std::optional<double>   traj_coord;
};

#endif // SIMULATOR_COMMAND_LINE
