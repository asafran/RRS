#ifndef VEHICLEDATA_H
#define VEHICLEDATA_H

#include    "solver-types.h"

class VehicleData
{
public:
    VehicleData();

    double getRailwayCoord() const;

    double getVelocity() const;

    double getWheelAngle(size_t i);

    double getWheelOmega(size_t i);

protected:

    /// Railway coordinate
    double railway_coord;
    /// Body velocity
    double velocity;

    /// Wheels rotation angles
    std::vector<double> wheel_rotation_angle;
    /// Wheels angular velocities
    std::vector<double> wheel_omega;


};

#endif // VEHICLEDATA_H
