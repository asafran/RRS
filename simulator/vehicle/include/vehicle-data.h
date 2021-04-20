#ifndef VEHICLEDATA_H
#define VEHICLEDATA_H

#include    "solver-types.h"
#include    "vehicle-controller.h"
#include    <vector>

struct vehicle_data_t
{
    /// Forward coupling force
    double  R1;
    /// Backward coupling force
    double  R2;
    /// Railway coordinate
    double railway_coord;
    /// Body velocity
    double velocity;

    /// Wheels rotation angles
    std::vector<double> wheel_rotation_angle;
    /// Wheels angular velocities
    std::vector<double> wheel_omega;

    /// Active common forces
    state_vector_t  Q_a;
    /// Reactive common forces
    state_vector_t  Q_r;
    /// Vehicle common acceleration
    state_vector_t  a;

    vec3d position;

    vehicle_data_t()
        : R1(0.0)
        , R2(0.0)
        , railway_coord(0.0)
        , velocity(0.0)
    {

    }

};

#endif // VEHICLEDATA_H
