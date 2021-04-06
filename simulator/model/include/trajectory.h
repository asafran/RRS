#ifndef     TRAJECTORY_H
#define     TRAJECTORY_H

#include    <QObject>
#include    <QSet>

#include    "track.h"
#include    "vehicle.h"

class       Connector;

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Trajectory : public QObject
{
    Q_OBJECT

public:

    Trajectory(QObject *parent = Q_NULLPTR);

    ~Trajectory();

    bool load(QString route_dir, QString traj_name);

    QString getName() const { return name; }

    vec3d getPosition(double x, vec3d &attitude);

    void setFwdConnector(Connector *conn) { this->fwdConnector = conn; }

    void setBwdConnector(Connector *conn) { this->bwdConnector = conn; }

    double getLength() const { return length; }

    Connector *getFwdConnector() const { return fwdConnector; }

    Connector *getBwdConnector() const { return bwdConnector; }

    void setBusy(Vehicle *vehicle) { vehicles_on_traj.insert(vehicle); }

    void removeBusy(Vehicle *vehicle) { vehicles_on_traj.remove(vehicle); }

    bool isBusy() const { return !vehicles_on_traj.isEmpty(); }

protected:

    QString         name;

    double          length;

    Connector       *fwdConnector;

    Connector       *bwdConnector;

    QSet<Vehicle *> vehicles_on_traj;

    std::vector<track_t>    tracks;

    track_t findTrack(double x, track_t &next_track);
};

#endif // TRAJECTORY_H
