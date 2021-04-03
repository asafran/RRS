#include "vehicle-data.h"

VehicleData::VehicleData()
    : R1(0.0)
    , R2(0.0)
    , railway_coord(0.0)
    , velocity(0.0)
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
