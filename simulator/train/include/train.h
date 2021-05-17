//------------------------------------------------------------------------------
//
//      Common train's model dynamics
//      (c) maisvendoo, 04/09/2018
//      Developer: Dmitry Pritykin
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Common train's model dynamics
 * \copyright maisvendoo
 * \author Dmitry Pritykin
 * \date 04/09/2018
 */

#ifndef     TRAIN_H
#define     TRAIN_H

#include    "filesystem.h"
#include    "init_data.h"
#include    "ode-system.h"
#include    "vehicle.h"
#include    "coupling.h"
#include    "solver.h"
#include    "solver-config.h"
#include    "brakepipe.h"
#include    "profile.h"
#include    "sound-manager.h"
#include    "topology.h"
#include    "global-const.h"

#include    <QVector>

#if defined(TRAIN_LIB)
    #define TRAIN_EXPORT    Q_DECL_EXPORT
#else
    #define TRAIN_EXPORT    Q_DECL_IMPORT
#endif

/*!
 * \class
 * \brief Common train model
 */



//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TRAIN_EXPORT Train : public OdeSystem
{
    Q_OBJECT

public:

    /// Constructor
    explicit Train(Profile *profile, Topology *topology, QObject *parent = Q_NULLPTR);

    explicit Train(Profile *profile, Topology *topology, init_data_t init_data, QObject *parent = Q_NULLPTR);

    /// Destructor
    virtual ~Train();

    /// Train initialization
    bool init();

    bool addVehiclesBack(QVector<Vehicle *> vehicles_add, bool reversed);

    bool addVehiclesFront(QVector<Vehicle *> vehicles_add);

    void updatePos();

    void updatePos(double coord);

    void setSpeed(double V, const double coeff);

//    bool updatePos(double speed, double coord);

    void updateBrakepipe();

    void placeTrain(const topology_pos_t &tp);

    /// Calculation of right part motion ODE's
    void calcDerivative(state_vector_t &Y, state_vector_t &dYdt, double t);

    /// Action before time step
    void preStep(double t);

    /// Integration step
    bool step(double t, double &dt);

    void connectToTrain(Vehicle *vehicle);

    /// Integration step for vehicles ODE's
    void vehiclesStep(double t, double dt);

    void inputProcess();

    /// Action after integration step
    QVarLengthArray<VehicleData, MAX_NUM_VEHICLES> postStep(double t);

    /// Get first vehicle
    QVector<Vehicle *>::const_iterator getFirstVehicle();

    /// Get last vehicle
    QVector<Vehicle *>::const_iterator getLastVehicle();

    QVector<Vehicle *>::const_iterator getEndVehicle();

    double getVelocity(size_t i) const;

    /// Get train mass
    double getMass() const;
    /// Get train length
    double getLength() const;

    size_t getVehiclesNumber() const;
    /// Initialization of vehicles brakes
    void initVehiclesBrakes(double charging_pressure, double main_res_pressure);

//    QString getClientName();

//    QString getTrainID();

    int getDirection() const;

    QVector<Vehicle *> getVehicles();

signals:

    void logMessage(QString msg);
//    void sendDataToVehicle(QByteArray data);

public slots:

//    void process();

private:    

    /// Train mass
    double          trainMass;
    /// Train length
    double          trainLength;

    /// Order of system ODE motion
    size_t          ode_order;

    /// Direction of motion on railway
//    int             dir;

    /// Profile manager
    Profile     *profile;
/*
    double init_velocity;

    double init_coord;

    /// Charging pressure
    double      charging_pressure;

    /// No air flag (for empty air system on start)
    bool        no_air;

    /// Initial main reservoir pressure
    double      init_main_res_pressure;
*/


    /// Motion ODE's solver
    Solver      *train_motion_solver;

    /// Brakepipe model
    BrakePipe   *brakepipe;    

    /// Sound manager
    SoundManager *soundMan;
    /// Pointer to topology database
    Topology *topo;
/*
    QString train_config_path;

    /// Имя сетевого клиента для ВЖД
    QString     client_name;

    /// Идентификатор поезда для ВЖД
    QString     train_id;
*/
    /// All train's vehicles
    QVector<Vehicle *> vehicles;

    /// All train's couplings
    QVector<Coupling *> couplings;

    /// Solver's configuration
    solver_config_t solver_config;

    const init_data_t init_data;

    /// Train's loading
//    bool loadTrain(const init_data_t &init_data, const QVarLengthArray<Vehicle *> vehicles_t);
    /// Couplings loading
    bool loadCouplings();

    void appendVehicle(Vehicle *vehicle);

    void topologyStep();

    /// Set initial conditions
//    void setInitConditions(const init_data_t &init_data);


//    void timerEvent(QTimerEvent *event) override;
};

#endif // TRAIN_H
