#include "vehicle-data.h"

VehicleData::VehicleData()
    : R1(0.0)
    , R2(0.0)
    , railway_coord(0.0)
    , velocity(0.0)
{

}

VehicleData::VehicleData(const VehicleData &vehicle)
    : R1(vehicle.R1)
    , R2(vehicle.R2)
    , railway_coord(vehicle.railway_coord)
    , velocity(vehicle.railway_coord)
    , wheel_rotation_angle(vehicle.wheel_rotation_angle)
    , wheel_omega(vehicle.wheel_omega)
    , Q_a(vehicle.Q_a)
    , Q_r(vehicle.Q_r)
    , a(vehicle.a)
    , position(vehicle.position)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double VehicleData::getWheelAngle(size_t i) const
{
    if (i < wheel_rotation_angle.size())
        return wheel_rotation_angle[i];
    else
        return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double VehicleData::getWheelOmega(size_t i) const
{
    if (i < wheel_omega.size())
        return wheel_omega[i];
    else
        return 0;
} 
