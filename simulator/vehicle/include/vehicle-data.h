#ifndef VEHICLEDATA_H
#define VEHICLEDATA_H

#include    "solver-types.h"
#include    "vehicle-controller.h"
//#include    "global-const.h"
#include    <vector>

class VehicleData
{
public:
    VehicleData();

    VehicleData(const VehicleData &vehicle);

    double getRailwayCoord() const { return railway_coord; }

    double getVelocity()  const { return velocity; }

    double getFwdCouplingForce() const { return R1; }

    double getBwdCouplingForce() const { return R2; }

    double getWheelAngle(size_t i) const;

    double getWheelOmega(size_t i) const;

    transfer_vector_t getCommonForces() const { return Q_a; }

    transfer_vector_t getReactiveForces() const { return Q_r; }

    transfer_vector_t getAcceleration() const { return a; }

protected:

    /// Forward coupling force
    double  R1;
    /// Backward coupling force
    double  R2;
    /// Railway coordinate
    double railway_coord;
    /// Body velocity
    double velocity;

    /// Wheels rotation angles
    transfer_vector_t wheel_rotation_angle;
    /// Wheels angular velocities
    transfer_vector_t wheel_omega;

    /// Active common forces
    transfer_vector_t Q_a;
    /// Reactive common forces
    transfer_vector_t Q_r;
    /// Vehicle common acceleration
    transfer_vector_t a;

    vec3d position;

};

#endif // VEHICLEDATA_H
