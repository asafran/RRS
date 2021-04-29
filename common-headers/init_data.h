//------------------------------------------------------------------------------
//
//
//
//
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief
 * \copyright
 * \author
 * \date
 */

#ifndef     INIT_DATA_H
#define     INIT_DATA_H

#include    <QString>

#include    "solver-config.h"

/*!
 * \struct
 * \brief
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct init_data_t
{
    int     direction;
    QString profile_path;
    double  prof_step;
    QString route_dir;
    QString coupling_module;
    QString brakepipe_conf;
    int     integration_time_interval;
//    int     control_time_interval;
    bool    debug_print;
//    double charging_pressure;
//    double init_main_res_pressure;
//    bool no_air;
    solver_config_t solver_config;

    init_data_t()
//        : init_coord(0)
//        , init_velocity(0)
        : direction(1)
//        , profile_path("")
        , prof_step(100.0)
//        , route_dir("")
        , coupling_module("default_coupling")
//        , brakepipe_conf("")
        , integration_time_interval(100)
//        , control_time_interval(50)
        , debug_print(false)
//        , charging_pressure(0.5)
//        , init_main_res_pressure(0.9)
//        , no_air(false)
    {

    }
};

#endif // INIT_DATA_H

