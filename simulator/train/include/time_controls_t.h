#ifndef TIME_CONTROLS_T_H
#define TIME_CONTROLS_T_H
struct time_controls_t
{
    /// Current simulation time
    double      t;
    /// Current simulation time step
    double      dt;
    /// Simulation start time
    double      start_time;
    /// Simulation stop time
    double      stop_time;
    /// Flag of integration step is correct
    bool        is_step_correct;
    /// Flag is simulation thread started
    bool        is_simulation_started;
    /// Delay for realtime simulation
    int         realtime_delay;
    /// Minimal intergation interval
    int         integration_time_interval;
    /// Flag, which allow debug print
    bool        is_debug_print;

    double      control_time;
    double      control_delay;

    time_controls_t()
        : t(0.0)
        , dt(0.001)
        , start_time(0.0)
        , stop_time(1000.0)
        , is_step_correct(true)
        , is_simulation_started(false)
        , realtime_delay(0)
        , integration_time_interval(100)
        , is_debug_print(false)
        , control_time(0)
        , control_delay(0.05)
    {

    }
};

#endif // TIME_CONTROLS_T_H
