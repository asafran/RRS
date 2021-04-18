#ifndef     TOPOLOGY_H
#define     TOPOLOGY_H

#include    <QObject>
#include    <QSet>
#include    <QMap>

#include    "topology-types.h"
#include    "vehicle-controller.h"

#include    "vehicle.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Topology : public QObject
{
    Q_OBJECT

public:

    Topology(QObject *parent = Q_NULLPTR);

    virtual ~Topology();

    /// Загрузка топологии ж/д полигона
    bool load(QString route_dir);

    traj_list_t getTrajList() const { return traj_list; }

    conn_list_t getJointsList() const { return joints; }

    conn_list_t getSwithesList() const { return switches; }

    /// Общая инициализация
//    bool init(const topology_pos_t &tp, QVarLengthArray<Vehicle *> *vehicles);

    /// Вернуть контроллер конкретной ПЕ
//    VehicleController *getVehicleController(size_t idx) const;

protected:

    /// Контейнер данных по всем траекториям на полигоне
    traj_list_t     traj_list;

    /// Контейнер изостыков
    conn_list_t     joints;

    /// Контейнер стрелок
    conn_list_t     switches;

    /// Контейнер ПЕ
//    QSet<Vehicle *> vehicle_control;

    /// Получить список имен всех имеющихся траекторий
    QStringList getTrajNamesList(QString route_dir);

    /// Загрузка топологии
    bool load_topology(QString route_dir);
};

#endif // TOPOLOGY_H
