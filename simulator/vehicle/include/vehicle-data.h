#ifndef VEHICLEDATA_H
#define VEHICLEDATA_H

#include    "solver-types.h"
#include    "vehicle-controller.h"
#include    <vector>

class VehicleData
{
public:
    VehicleData();

    double getRailwayCoord() const { return railway_coord; }

    double getVelocity()  const { return velocity; }

    double getFwdCouplingForce() const { return R1; }

    double getBwdCouplingForce() const { return R2; }

    double getWheelAngle(size_t i) const;

    double getWheelOmega(size_t i) const;

    state_vector_t getCommonForces() const { return Q_a; }

    state_vector_t getReactiveForces() const { return Q_r; }

    state_vector_t getAcceleration() const { return a; }

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
    std::vector<double> wheel_rotation_angle;
    /// Wheels angular velocities
    std::vector<double> wheel_omega;

    /// Active common forces
    state_vector_t  Q_a;
    /// Reactive common forces
    state_vector_t  Q_r;
    /// Vehicle common acceleration
    state_vector_t  a;

//    vec3d position;

};

#endif // VEHICLEDATA_H
