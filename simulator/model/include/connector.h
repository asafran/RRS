#ifndef     CONNECTOR_H
#define     CONNECTOR_H

#include    <QObject>
#include    "CfgReader.h"

#include    "topology-types.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Connector : public QObject
{
    Q_OBJECT

public:

    Connector(QObject *parent = Q_NULLPTR);

    virtual ~Connector();

    virtual Trajectory *getFwdTraj() const { return fwdTraj; }

    virtual Trajectory *getBwdTraj() const { return bwdTraj; }

    virtual void setState(int state) { this->state = state; }

    virtual void configure(CfgReader &cfg, QDomNode secNode, traj_list_t &traj_list);

    QString getName() const { return this->name; }

    bool isReversed() const { return reverser; }

protected:

    Trajectory *fwdTraj;

    Trajectory *bwdTraj;

    int state;

    bool reverser;

    QString name;
};

#endif // CONNECTOR_H
