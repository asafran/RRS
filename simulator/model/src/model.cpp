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

#include    "model.h"

#include    <QTimerEvent>

#include    "CfgReader.h"
#include    "Journal.h"
#include    "JournalFile.h"

#include    "topology.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Model::Model(QObject *parent) : QObject(parent)
  , t(0.0)
  , dt(0.001)
  , start_time(0.0)
  , stop_time(1000.0)
  , is_step_correct(true)
  , is_simulation_started(false)
  , realtime_delay(0)
  , integration_time_interval(100)
  , is_debug_print(false)
  , control_time(0)
  , control_delay(0.05)
  , profile(nullptr)
  , control_panel(nullptr)
  , topology(this)
  , timer_id(0)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Model::~Model()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Model::init(const simulator_init_t &command_line)
{
    // Check is debug print allowed
    is_debug_print = command_line.debug_print.value();

    init_data_t init_data;

    // Load initial data configuration
    Journal::instance()->info("==== Init data loading ====");
    loadInitData(init_data);

    // Override init data by command line
    Journal::instance()->info("==== Command line processing ====");
    overrideByCommandLine(init_data, command_line);

    // Read solver configuration
    Journal::instance()->info("==== Solver configurating ====");
    configSolver(init_data.solver_config);

    start_time = init_data.solver_config.start_time;
    stop_time = init_data.solver_config.stop_time;
    dt = init_data.solver_config.step;
    integration_time_interval = init_data.integration_time_interval;

    // Load profile
    Journal::instance()->info("==== Profile data loading ====");
    profile = new Profile(init_data.direction, init_data.route_dir.toStdString());

    Journal::instance()->info(QString("State Profile object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(profile), 0, 16));

    Journal::instance()->info("==== Topology data loading ====");
    if(topology.load(init_data.route_dir))
        Journal::instance()->info("Topology loaded successfully");
    else
        Journal::instance()->info("Topology is't loaded successfully");

    if (profile->isReady())
        Journal::instance()->info("Profile loaded successfully");
    else
    {
        Journal::instance()->warning("Profile is't loaded. Using flat profile");
    }

    // Train creation and initialization
    Journal::instance()->info("==== Train initialization ====");

    trains.insert(loadTrain(init_data, command_line));


/*
    if (!train->init() || !train->addVehicles(vehilces))
        return false;    
*/
    initControlPanel("control-panel");

//    initSimClient("virtual-railway");

    Journal::instance()->info("Train is initialized successfully");

    return true;
}

Train *Model::loadTrain(const init_data_t &init_data, const simulator_init_t &command_line)
{
    // Loading train config XML-file
    FileSystem &fs = FileSystem::getInstance();
    QString path = fs.combinePath(fs.getTrainsDir(), command_line.train_config.value() + ".xml");
    QVector<Vehicle *> vehicles;

    CfgReader cfg;
    // Check train config name
    if (!command_line.train_config.has_value())
    {
        Journal::instance()->error("Train config is't referenced");
        return nullptr;
    }
    if (!cfg.load(path))
    {
        Journal::instance()->error("Train's config file" + path + "is't opened");
        return nullptr;
    }    

    QDomNode vehicle_node = cfg.getFirstSection("Vehicle");
    // Parsing of train config file

    while (vehicle_node.isNull())
    {
        QString module_name = "";
        if (!cfg.getString(vehicle_node, "Module", module_name))
        {
            Journal::instance()->error("Module section is not found");
            break;
        }

        QString q_module_cfg_name = "";
        if (!cfg.getString(vehicle_node, "ModuleConfig", q_module_cfg_name))
        {
            Journal::instance()->error("Module config file name is not found");
            break;
        }

        std::string module_cfg_name(q_module_cfg_name.toStdString());

        // Calculate module library path
        QString relModulePath = QString(fs.combinePath(module_name, module_name));

        int n_vehicles = 0;

        if (!cfg.getInt(vehicle_node, "Count", n_vehicles))
        {
            n_vehicles = 0;
//            Journal::instance()->warning("Count of vehicles " + module_name + " is not found. Vehicle will't loaded");
        }

        // Payload coefficient of vehicles group
        double payload_coeff = 0;
        if (!cfg.getDouble(vehicle_node, "PayloadCoeff", payload_coeff))
        {
            payload_coeff = 0;
        }

        for (int i = 0; i < n_vehicles; ++i)
        {
/*
            vsg::ref_ptr<vsg::Group> vehicle_model = loadVehicleModel(module_cfg_name);

            if (!vehicle_model.valid())
            {
                std::cerr << "Vehicle model " << module_cfg_name << " is't loaded" << std::endl;
                continue;
            }

            // Load cabine model
            osg::ref_ptr<osg::Node> cabine;
            loadCabine(vehicle_model.get(), module_cfg_name, cabine);

            float length = getLength(module_cfg_name);

            osg::Vec3 driver_pos = getDirverPosition(module_cfg_name);

            vehicle_exterior_t vehicle_ext;
            vehicle_ext.transform = new osg::MatrixTransform;
            vehicle_ext.transform->addChild(vehicle_model.get());
            vehicle_ext.length = length;
            vehicle_ext.cabine = cabine;
            vehicle_ext.driver_pos = driver_pos;

            vehicle_ext.anims = new animations_t();
            vehicle_ext.displays = new displays_t();

            loadModelAnimations(module_cfg_name, vehicle_model.get(), *vehicle_ext.anims);
            loadAnimations(module_cfg_name, vehicle_model.get(), *vehicle_ext.anims);
            loadAnimations(module_cfg_name, cabine.get(), *vehicle_ext.anims);

            anim_managers.push_back(new AnimationManager(vehicle_ext.anims));

            loadDisplays(cfg, child, cabine.get(), *vehicle_ext.displays);

            vehicles_ext.push_back(vehicle_ext);
            trainExterior->addChild(vehicle_ext.transform.get());

            */
            Vehicle *vehicle = loadVehicle(fs.getModulesDir() +
                                           fs.separator() +
                                           relModulePath);

            if (vehicle == Q_NULLPTR)
            {
                    Journal::instance()->error("Vehicle " + module_name + " is't loaded");
                break;
            }

                Journal::instance()->info(QString("Created Vehicle object at address: 0x%1")
                                          .arg(reinterpret_cast<quint64>(vehicle), 0, 16));


            QString relConfigPath = fs.combinePath(q_module_cfg_name, q_module_cfg_name);


            QString config_dir(fs.combinePath(fs.getVehiclesDir(), q_module_cfg_name));
            vehicle->setConfigDir(config_dir);

            vehicle->init(fs.getVehiclesDir() + fs.separator() + relConfigPath + ".xml");

            vehicle->setPayloadCoeff(payload_coeff);
//            vehicle->setVelocity(0);

            double charging_pressure = 0.5;
            bool no_air = false;

            cfg.getDouble("Common", "ChargingPressure", charging_pressure);
            cfg.getBool("Common","NoAir", no_air);

            if(!no_air)
                vehicle->setBrakepipePressure(charging_pressure);

            vehicles.push_back(vehicle);

        }
        vehicle_node = cfg.getNextSection();

    }

    Train *train = new Train(profile, &topology, init_data, this);
    train->init();

    train->addVehiclesBack(vehicles);

    Journal::instance()->info(QString("Created Train object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(train), 0, 16));

    connect(train, &Train::logMessage, this, &Model::logMessage);

    double charging_pressure = 0.5;
    double main_res_pressure = 0.9;

    if (cfg.getDouble("Common", "ChargingPressure", charging_pressure)
            && cfg.getDouble("Common", "InitMainResPressure", main_res_pressure))
    {
        train->initVehiclesBrakes(charging_pressure, main_res_pressure);
    }
    if(command_line.init_coord)
        train->updatePos(*command_line.init_coord);
    if(command_line.init_velocity)
        train->setSpeed(*command_line.init_velocity, Physics::kmh);

    topology_pos_t tp;

    if(command_line.traj_name)
        tp.traj_name = *command_line.traj_name;
    if(command_line.traj_coord)
        tp.traj_coord = *command_line.traj_coord;
    if(command_line.direction)
        tp.dir = *command_line.direction;

    train->placeTrain(tp);

    return train;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::start()
{
    if (!isStarted())
    {
        is_simulation_started = true;
        t = start_time;

        timer_id = this->startTimer(integration_time_interval);

    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Model::isStarted() const
{
    return is_simulation_started;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::outMessage(QString msg)
{
    fputs(qPrintable(msg + "\n"), stdout);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::controlProcess()
{
    control_panel->process();
}
/*
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::preStep(double t)
{
    train->preStep(t);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Model::step(double t, double &dt)
{
    if (!train->step(t, dt))
        return false;

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::postStep(double t)
{
    train->postStep(t);
}
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::debugPrint()
{
/*
    QString debug_info = QString("t = %1 realtime_delay = %2 time_step = %3 x = %10 v[first] = %4 v[last] = %5 trac = %6 pos = %7 eq_press = %8 bp_press = %9 pos = %11\n")
            .arg(t)
            .arg(realtime_delay)
            .arg(dt)
            .arg(train->getFirstVehicle()->getVelocity() * 3.6)
            .arg(train->getLastVehicle()->getVelocity() * 3.6)
            .arg(static_cast<double>(train->getFirstVehicle()->getAnalogSignal(0)))
            .arg(static_cast<int>(train->getFirstVehicle()->getAnalogSignal(3)))
            .arg(static_cast<double>(train->getFirstVehicle()->getAnalogSignal(2)))
            .arg(static_cast<double>(train->getFirstVehicle()->getAnalogSignal(4)))
            .arg(train->getFirstVehicle()->getRailwayCoord())
            .arg(static_cast<double>(train->getFirstVehicle()->getAnalogSignal(20)));

    fputs(qPrintable(debug_info), stdout);
*/
}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::loadInitData(init_data_t &init_data)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString cfg_path = fs.getConfigDir() + fs.separator() + "init-data.xml";

    if (cfg.load(cfg_path))
    {
        QString secName = "InitData";
/*
        if (!cfg.getDouble(secName, "InitCoord", init_data.init_coord))
        {
            init_data.init_coord = 1.0;
        }

        if (!cfg.getDouble(secName, "InitVelocity", init_data.init_velocity))
        {
            init_data.init_velocity = 0.0;
        }
*/
        if (!cfg.getString(secName, "Profile", init_data.profile_path))
        {
            init_data.profile_path = "default";
        }

        if (!cfg.getDouble(secName, "ProfileStep", init_data.prof_step))
        {
            init_data.prof_step = 100.0;
        }

        if (!cfg.getInt(secName, "IntegrationTimeInterval", init_data.integration_time_interval))
        {
            init_data.integration_time_interval = 100;
        }
/*
        if (!cfg.getInt(secName, "ControlTimeInterval", init_data.control_time_interval))
        {
            init_data.control_time_interval = 50;
        }

        control_delay = static_cast<double>(init_data.control_time_interval) / 1000.0;
*/
        if (!cfg.getBool(secName, "DebugPrint", init_data.debug_print))
        {
            init_data.debug_print = false;
        }

        Journal::instance()->info("Loaded settings from: " + cfg_path);
    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " not found");
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::overrideByCommandLine(init_data_t &init_data,
                                  const simulator_init_t &command_line)
{
    if (command_line.route_dir)
        init_data.route_dir = *command_line.route_dir;

    if (command_line.debug_print)
        init_data.debug_print = *command_line.debug_print;
/*
    if (command_line.init_coord)
    {
        init_data.init_coord = *command_line.init_coord;
    }
*/
    if (command_line.direction)
        init_data.direction = *command_line.direction;

    Journal::instance()->info("Apply command line settinds");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::configSolver(solver_config_t &solver_config)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString cfg_path = fs.getConfigDir() + fs.separator() + "solver.xml";

    if (cfg.load(cfg_path))
    {
        QString secName = "Solver";

        if (!cfg.getString(secName, "Method", solver_config.method))
        {
            solver_config.method = "rkf5";
        }

        Journal::instance()->info("Integration method: " + solver_config.method);

        if (!cfg.getDouble(secName, "StartTime", solver_config.start_time))
        {
            solver_config.start_time = 0;
        }

        Journal::instance()->info("Start time: " + QString("%1").arg(solver_config.start_time));

        if (!cfg.getDouble(secName, "StopTime", solver_config.stop_time))
        {
            solver_config.stop_time = 10.0;
        }

        Journal::instance()->info("Stop time: " + QString("%1").arg(solver_config.stop_time));

        if (!cfg.getDouble(secName, "InitStep", solver_config.step))
        {
            solver_config.step = 1e-4;
        }

        Journal::instance()->info("Initial integration step: " + QString("%1").arg(solver_config.step));

        if (!cfg.getDouble(secName, "MaxStep", solver_config.max_step))
        {
            solver_config.max_step = 1e-2;
        }

        Journal::instance()->info("Maximal integration step: " + QString("%1").arg(solver_config.max_step));
    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " not found");
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::initControlPanel(QString cfg_path)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString full_path = fs.getConfigDir() + fs.separator() + cfg_path + ".xml";

    if (cfg.load(full_path))
    {
        QString secName = "ControlPanel";
        QString module_name = "";

        if (!cfg.getString(secName, "Plugin", module_name))
            return;

        control_panel = Q_NULLPTR;
        QString module_path = fs.getPluginsDir() + fs.separator() + module_name;
        control_panel = loadInterfaceDevice(module_path);

        if (control_panel == Q_NULLPTR)
            return;

        QString config_dir = "";

        if (!cfg.getString(secName, "ConfigDir", config_dir))
            return;

        config_dir = fs.toNativeSeparators(config_dir);

        if (!control_panel->init(fs.getConfigDir() + fs.separator() + config_dir))
            return;

        int request_interval = 0;

        if (!cfg.getInt(secName, "RequestInterval", request_interval))
            request_interval = 100;

        int v_idx = 0;

        if (!cfg.getInt(secName, "Vehicle", v_idx))
            v_idx = 0;

        Vehicle *vehicle = trains[0]->getVehicles().at(static_cast<size_t>(v_idx));

        connect(vehicle, &Vehicle::sendFeedBackSignals,
                control_panel, &VirtualInterfaceDevice::receiveFeedback);

        connect(control_panel, &VirtualInterfaceDevice::sendControlSignals,
                vehicle, &Vehicle::getControlSignals);

    }
}

//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
void Model::initSimClient(QString cfg_path)
{
    if (train->getTrainID().isEmpty())
        return;

    if (train->getClientName().isEmpty())
        return;

    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString full_path = QString(fs.getConfigDir().c_str()) + fs.separator() + cfg_path + ".xml";

    if (cfg.load(full_path))
    {
        QString secName = "VRServer";
        tcp_config_t tcp_config;

        cfg.getString(secName, "HostAddr", tcp_config.host_addr);
        int port = 0;

        if (!cfg.getInt(secName, "Port", port))
        {
            port = 1993;
        }

        tcp_config.port = static_cast<quint16>(port);
        tcp_config.name = train->getClientName();

        sim_client = new SimTcpClient();
        connect(this, &Model::getRecvData, sim_client, &SimTcpClient::getRecvData);
        sim_client->init(tcp_config);
        sim_client->start();

        Journal::instance()->info("Started virtual railway TCP-client...");

        connect(&networkTimer, &QTimer::timeout, this, &Model::virtualRailwayFeedback);
        networkTimer.start(100);
    }
    else
    {
        Journal::instance()->error("There is no virtual railway configuration in file " + full_path);
    }
}
*/
//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
void Model::virtualRailwayFeedback()
{
    if (sim_client == Q_NULLPTR)
        return;

    if (!sim_client->isConnected())
        return;

    sim_dispatcher_data_t disp_data;
    emit getRecvData(disp_data);

    alsn_info_t alsn_info;
    alsn_info.code_alsn = disp_data.code_alsn;
    alsn_info.num_free_block = disp_data.num_free_block;
    alsn_info.response_code = disp_data.response_code;
    alsn_info.signal_dist = disp_data.signal_dist;
    strcpy(alsn_info.current_time, disp_data.current_time);

    train->getFirstVehicle()->setASLN(alsn_info);

    sim_train_data_t train_data;
    strcpy(train_data.train_id, train->getTrainID().toStdString().c_str());
    train_data.direction = train->getDirection();
    train_data.coord = train->getFirstVehicle()->getRailwayCoord();
    train_data.speed = train->getFirstVehicle()->getVelocity();

    sim_client->sendTrainData(train_data);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::controlStep(double &control_time, const double control_delay)
{
    if (control_time >= control_delay)
    {
        control_time = 0;

        if (keys_data.lock())
        {            
            data.resize(keys_data.size());
            memcpy(data.data(), keys_data.data(), static_cast<size_t>(keys_data.size()));

            if (keys_data.size() != 0)
                emit sendDataToTrain(data);

            keys_data.unlock();
        }
    }

    control_time += dt;
}
*/
//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
void Model::topologyStep()
{
    for (size_t i = 0; i < train->getVehicles()->size(); ++i)
    {
        VehicleController *vc = topology.getVehicleController(i);
        vc[i].setRailwayCoord(train->getVehicles()->at(i)->getRailwayCoord());
    }
}
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::timerEvent(QTimerEvent *event)
{
    double tau = 0;
    double integration_time = static_cast<double>(integration_time_interval) / 1000.0;    

    // Integrate all ODE in train motion model
    while ( (tau <= integration_time) &&
            is_step_correct)
    {
        for( auto train = trains.begin(); train != trains.end(); ++train )
        {
            (*train)->preStep(t);

            is_step_correct = (*train)->step(t, dt);

            //topologyStep();

            tau += dt;
            t += dt;

            emit sendDataToViewer((*train)->postStep(t));
        }
    }

//    train->inputProcess();

    // Debug print, is allowed
    if (is_debug_print)
        debugPrint();

    event->accept();
}

