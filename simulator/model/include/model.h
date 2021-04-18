//------------------------------------------------------------------------------
//
//      Train motion model simulation control
//      (c) maisvendoo, 02/09/2018
//      Developer: Dmitry Pritykin
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Train motion model simulation control
 * \copyright maisvendoo
 * \author Dmitry Pritykin
 * \date 02/09/2018
 */

#ifndef     MODEL_H
#define     MODEL_H

#include    <QtGlobal>
#include    <QObject>
#include    <QThread>
#include    <QSharedMemory>
#include    <QTimer>

#include    "simulator-command-line.h"
#include    "filesystem.h"
#include    "train.h"
#include    "elapsed-timer.h"

#include    "server.h"

#include    "profile.h"

#include    "keys-control.h"

#include    "virtual-interface-device.h"

#include    "sim-client.h"

#include    "topology.h"

#if defined(MODEL_LIB)
    #define MODEL_EXPORT Q_DECL_EXPORT
#else
    #define MODEL_EXPORT Q_DECL_IMPORT
#endif

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class MODEL_EXPORT Model : public QObject
{
    Q_OBJECT

public:

    /// Constructor
    explicit Model(QObject *parent = Q_NULLPTR);
    /// Destructor
    virtual ~Model();

    /// Model initialization
    bool init(const simulator_command_line_t &command_line);

    QVarLengthArray<Vehicle *> load(const simulator_command_line_t &command_line);

    /// Check is simulation started
    bool isStarted() const;

signals:

    void logMessage(QString msg);

//    void sendDataToServer(QByteArray data);

//    void sendDataToTrain(QByteArray data);

//    void getRecvData(sim_dispatcher_data_t &disp_data);

public slots:

    /// Messages output
    void outMessage(QString msg);

    ///
    void controlProcess();

    /// Обмен данными с ВЖД
//    void virtualRailwayFeedback();

    /// Start simulation thread
    void start();

private:

    /// Current simulation time
    double      t;
    /// Current simulation time step
    double      dt;
    /// Simulation start time
    double      start_time;
    /// Simulation stop time
    double      stop_time;
    /// Flag of integration step is correct
    bool        is_step_correct;
    /// Flag is simulation thread started
    bool        is_simulation_started;
    /// Delay for realtime simulation
    int         realtime_delay;
    /// Minimal intergation interval
    int         integration_time_interval;
    /// Flag, which allow debug print
    bool        is_debug_print;    

    double      control_time;
    double      control_delay;

    /// Train model
    QVarLengthArray<Train *>       train;

    /// Profile
    Profile     *profile;

    /// Виртуальное устройство для сопряжения с внешним пультом
    VirtualInterfaceDevice  *control_panel;

    Topology topology;

    int timerid;

//    vsg::ref_ptr<vsg::Group> trainExterior;

    /// Actions, which prerare integration step
    void preStep(double t);
    /// Simulation step
    bool step(double t, double &dt);
    /// Actions after integration step
    void postStep(double t);

    /// Debug print to stdout
    void debugPrint();

    /// Initial data loading
    void loadInitData(init_data_t &init_data);

    /// Override of initial data by command line
    void overrideByCommandLine(init_data_t &init_data, const simulator_command_line_t &command_line);

    /// Solver configuration loading
    void configSolver(solver_config_t &solver_config);

    void initControlPanel(QString cfg_path);


    void controlStep(double &control_time, const double control_delay);

    void timerEvent(QTimerEvent *event) override;

private slots:

    void process();
};

#endif // MODEL_H
