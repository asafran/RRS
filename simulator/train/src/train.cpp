#include    "train.h"

#include    "CfgReader.h"
#include    "physics.h"
#include    "Journal.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::Train(Profile *profile, time_controls_t controls, QObject *parent) : OdeSystem(parent)
  , trainMass(0.0)
  , trainLength(0.0)
  , ode_order(0)
  , dir(1)
  , profile(profile)
  , charging_pressure(0.0)
  , no_air(false)
  , init_main_res_pressure(0.0)
  , train_motion_solver(nullptr)
  , brakepipe(nullptr)
  , soundMan(nullptr)
  , time_controls(controls)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::Train(Profile *profile, const QVarLengthArray<Vehicle *> vehicles, time_controls_t controls,
             init_data_t init_data, QObject *parent) : OdeSystem(parent)
  , trainMass(0.0)
  , trainLength(0.0)
  , ode_order(0)
  , dir(init_data.direction)
  , profile(profile)
  , init_velocity(init_data.init_velocity)
  , init_coord(init_data.init_coord)
  , charging_pressure(init_data.charging_pressure)
  , no_air(init_data.no_air)
  , init_main_res_pressure(init_data.init_main_res_pressure)
  , train_motion_solver(nullptr)
  , brakepipe(nullptr)
  , soundMan(nullptr)
  , time_controls(controls)
{
    init(init_data, vehicles);
}
//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
Train::Train(const Train *train, size_t decouple, QObject *parent) : OdeSystem(parent)
//  , trainMass(0.0)
//  , trainLength(0.0)
  , ode_order(train->ode_order)
  , dir(train->dir)
  , profile(train->profile)
  , charging_pressure(train->brakepipe->getPressure(decouple))
  , no_air(false)
  , init_main_res_pressure(train->init_main_res_pressure)
  , train_motion_solver(train->train_motion_solver)
  , brakepipe(nullptr)
  , soundMan(nullptr)
  , time_controls(train->time_controls)
  , solver_config(train->solver_config)
  , vehicles(train->vehicles.begin() + decouple, train->vehicles.end())
  , couplings(train->couplings.begin() + decouple, train->couplings.end() - 1)
{
    FileSystem &fs = FileSystem::getInstance();
    try
    {
        soundMan = new SoundManager();

        Journal::instance()->info(QString("Created SoundManager at address: 0x%1")
                                      .arg(reinterpret_cast<quint64>(soundMan), 0, 16));

    } catch (const std::bad_alloc &)
    {
        Journal::instance()->error("Sound manager is;t created");
    }
    auto it = vehicles.begin();
    for (; it != vehicles.end(); ++it)
    {
        soundMan->loadSounds((*it)->getSoundsDir());

        connect(*it, &Vehicle::soundPlay, soundMan, &SoundManager::play, Qt::DirectConnection);
        connect(*it, &Vehicle::soundStop, soundMan, &SoundManager::stop, Qt::DirectConnection);
        connect(*it, &Vehicle::soundSetVolume, soundMan, &SoundManager::setVolume, Qt::DirectConnection);
        connect(*it, &Vehicle::soundSetPitch, soundMan, &SoundManager::setPitch, Qt::DirectConnection);
        connect(*it, &Vehicle::volumeCurveStep, soundMan, &SoundManager::volumeCurveStep, Qt::DirectConnection);

        trainMass += (*it)->getMass();
        trainLength += (*it)->getLength();

    }
    it = vehicles.begin() + 1;
    vehicles.first()->setNextVehicle(*it);
    vehicles.first()->setPrevVehicle(Q_NULLPTR);
    for (; it != vehicles.end() - 1; ++it)
    {
        (*it)->setNextVehicle(*(it+1));
        (*it)->setPrevVehicle(*(it-1));
    }
    vehicles.last()->setNextVehicle(Q_NULLPTR);
    vehicles.last()->setPrevVehicle(*(it-1));

    y.resize(ode_order);
    dydt.resize(ode_order);

    for (size_t i = 0; i < y.size(); i++)
        y[i] = dydt[i] = 0;

//    init_data_t init_data;
//    init_data.init_velocity = train->getVelocity(decouple - 1);
//    init_data.init_coord = train->vehicles[decouple ]

    brakepipe = new BrakePipe();

    brakepipe->setLength(trainLength);
    brakepipe->setNodesNum(vehicles.size());

    if (!no_air)
        brakepipe->setBeginPressure(charging_pressure * Physics::MPa + Physics::pA);

    brakepipe->init(QString(fs.getConfigDir().c_str()) + fs.separator() + "brakepipe.xml");

    initVehiclesBrakes();
}
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::~Train()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::init(const init_data_t &init_data, const QVarLengthArray<Vehicle *> vehicles_t)
{
    solver_config = init_data.solver_config;    

//    dir = init_data.direction;

    // Solver loading
    FileSystem &fs = FileSystem::getInstance();
    QString solver_path = QString(fs.getLibraryDir().c_str()) + fs.separator() + solver_config.method;

    train_motion_solver = loadSolver(solver_path);

    Journal::instance()->info(QString("Created Solver object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(train_motion_solver), 0, 16));

    if (train_motion_solver == Q_NULLPTR)
    {
        Journal::instance()->error("Solver " + solver_path + " is't found");
        return false;
    }

    Journal::instance()->info("Loaded solver: " + solver_path);

    full_config_path = QString(fs.getTrainsDir().c_str()) +
            fs.separator() +
            init_data.train_config + ".xml";

    Journal::instance()->info("Train config from file: " + full_config_path);

    try
    {
        soundMan = new SoundManager();

        Journal::instance()->info(QString("Created SoundManager at address: 0x%1")
                                      .arg(reinterpret_cast<quint64>(soundMan), 0, 16));

    } catch (const std::bad_alloc &)
    {
        Journal::instance()->error("Sound manager is;t created");
    }

    // Loading of train
    if (!addVehiclesBack(vehicles_t))
    {
        Journal::instance()->error("Train is't loaded");
        return false;
    }

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::calcDerivative(state_vector_t &Y, state_vector_t &dYdt, double t)
{
    size_t num_vehicles = vehicles.size();
    auto end = vehicles.end();
    auto coup_it = couplings.begin();

    for (auto it = vehicles.begin(); it != end; ++it)
    {
        Vehicle *vehicle = *it;
        size_t idx = vehicle->getIndex();
        size_t s = vehicle->getDegressOfFreedom();

        if ( (num_vehicles > 1) && ( it != end - 1) )
        {
            Vehicle *vehicle1 = *(it+1);
            size_t idx1 = vehicle1->getIndex();
            size_t s1 = vehicle1->getDegressOfFreedom();

            double ds = Y[idx] - Y[idx1] -
                    dir * vehicle->getLength() / 2 -
                    dir * vehicle1->getLength() / 2;

            double dv = Y[idx + s] - Y[idx1 + s1];

            Coupling *coup = *coup_it;
            double R = dir * coup->getForce(ds, dv);
            ++coup_it;

            vehicle->setBackwardForce(R);
            vehicle1->setForwardForce(R);
        }

        profile_element_t pe = profile->getElement(vehicle->getRailwayCoord());

        vehicle->setInclination(pe.inclination);
        vehicle->setCurvature(pe.curvature);

        state_vector_t a = vehicle->getAcceleration(Y, t);

        memcpy(dYdt.data() + idx, Y.data() + idx + s, sizeof(double) * s);
        memcpy(dYdt.data() + idx + s, a.data(), sizeof(double) * s);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::preStep(double t)
{
    (void) t;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::step(double t, double &dt)
{
    // Train dynamics simulation
    bool done = train_motion_solver->step(this, y, dydt, t, dt,
                                          solver_config.max_step,
                                          solver_config.local_error);    
    // Brakepipe simulation
    brakepipe->step(t, dt);

    vehiclesStep(t, dt);

    return done;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::vehiclesStep(double t, double dt)
{
    auto end = vehicles.end();
    auto begin = vehicles.begin();

    brakepipe->setBeginPressure((*begin)->getBrakepipeBeginPressure());
    size_t j = 1;

    for (auto i = begin; i != end; ++i)
    {
        Vehicle *vehicle = *i;
        VehicleController *vc = static_cast<VehicleController *>(*i);

        brakepipe->setAuxRate(j, vehicle->getBrakepipeAuxRate());
        vehicle->setBrakepipePressure(brakepipe->getPressure(j));
        vehicle->integrationStep(y, t, dt);

        vc->setRailwayCoord(vehicle->getRailwayCoord());

        ++j;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::inputProcess()
{
    auto end = vehicles.end();
    auto begin = vehicles.begin();

    for (auto i = begin; i != end; ++i)
    {
        Vehicle *vehicle = *i;
        vehicle->keyProcess();
        vehicle->hardwareProcess();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::postStep(double t)
{
    (void) t;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Vehicle *Train::getFirstVehicle() const
{
    return *vehicles.begin();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Vehicle *Train::getLastVehicle() const
{
    return *(vehicles.end() - 1);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double Train::getVelocity(size_t i) const
{
    if (i < vehicles.size())
    {
        size_t idx = vehicles[i]->getIndex();
        size_t s = vehicles[i]->getDegressOfFreedom();
        return y[idx + s];
    }

    return 0.0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double Train::getMass() const
{
    return trainMass;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double Train::getLength() const
{
    return trainLength;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
size_t Train::getVehiclesNumber() const
{
    return vehicles.size();
}

//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
QString Train::getClientName()
{
    return client_name;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString Train::getTrainID()
{
    return train_id;
}
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int Train::getDirection() const
{
    return dir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QVarLengthArray<Vehicle *> *Train::getVehicles()
{
    return &vehicles;
}

//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
bool Train::loadTrain(const init_data_t &init_data, const QVarLengthArray<Vehicle *> vehicles_t)
{
    Journal::instance()->info("Train loading is started...");

    if (cfg.load(cfg_path))
    {
        // Get charging pressure and no air flag
        if (!cfg.getDouble("Common", "ChargingPressure", charging_pressure))
        {
            charging_pressure = 0.5;
        }

        if (!cfg.getDouble("Common", "InitMainResPressure", init_main_res_pressure))
        {
            init_main_res_pressure = 0.9;
        }

        if (!cfg.getBool("Common", "NoAir", no_air))
        {
            no_air = false;
        }

        if (!cfg.getString("Common", "ClientName", client_name))
        {
            client_name = "";
        }

        if (!cfg.getString("Common", "TrainID", train_id))
        {
            train_id = "";
        }        

        QDomNode vehicle_node = cfg.getFirstSection("Vehicle");

        if (vehicle_node.isNull())
            Journal::instance()->error("There are not Vehicle sections in train config");

        size_t index = 0;

        while (!vehicle_node.isNull())
        {
            QString module_name = "";
            if (!cfg.getString(vehicle_node, "Module", module_name))
            {
                Journal::instance()->error("Module section is not found");
                break;
            }

            QString module_cfg_name = "";
            if (!cfg.getString(vehicle_node, "ModuleConfig", module_cfg_name))
            {
                Journal::instance()->error("Module config file name is not found");
                break;
            }

            // Calculate module library path
            QString relModulePath = QString(fs.combinePath(module_name.toStdString(), module_name.toStdString()).c_str());

            // Vehicles count
            int n_vehicles = 0;
            if (!cfg.getInt(vehicle_node, "Count", n_vehicles))
            {
                n_vehicles = 0;
                Journal::instance()->warning("Count of vehicles " + module_name + " is not found. Vehicle will't loaded");
            }

            // Payload coefficient of vehicles group
            double payload_coeff = 0;
            if (!cfg.getDouble(vehicle_node, "PayloadCoeff", payload_coeff))
            {
                payload_coeff = 0;
            }
    
            for (int i = 0; i < n_vehicles; i++)
            {
                Vehicle *vehicle = loadVehicle(QString(fs.getModulesDir().c_str()) +
                                               fs.separator() +
                                               relModulePath);

                if (vehicle == Q_NULLPTR)
                {
                    Journal::instance()->error("Vehicle " + module_name + " is't loaded");
                    break;
                }

                Journal::instance()->info(QString("Created Vehicle object at address: 0x%1")
                                          .arg(reinterpret_cast<quint64>(vehicle), 0, 16));

                connect(vehicle, &Vehicle::logMessage, this, &Train::logMessage);

                QString relConfigPath = QString(fs.combinePath(module_cfg_name.toStdString(), module_cfg_name.toStdString()).c_str());


                QString config_dir(fs.combinePath(fs.getVehiclesDir(), module_cfg_name.toStdString()).c_str());
                vehicle->setConfigDir(config_dir);

                vehicle->init(QString(fs.getVehiclesDir().c_str()) + fs.separator() + relConfigPath + ".xml");                

                vehicle->setPayloadCoeff(payload_coeff);

                vehicle->setDirection(dir);

                trainMass += vehicle->getMass();
                trainLength += vehicle->getLength();

                size_t s = vehicle->getDegressOfFreedom();

                ode_order += 2 * s;

                vehicle->setIndex(index);
                index = ode_order;

                // Loading sounds
                soundMan->loadSounds(vehicle->getSoundsDir());

                connect(vehicle, &Vehicle::soundPlay, soundMan, &SoundManager::play, Qt::DirectConnection);
                connect(vehicle, &Vehicle::soundStop, soundMan, &SoundManager::stop, Qt::DirectConnection);
                connect(vehicle, &Vehicle::soundSetVolume, soundMan, &SoundManager::setVolume, Qt::DirectConnection);
                connect(vehicle, &Vehicle::soundSetPitch, soundMan, &SoundManager::setPitch, Qt::DirectConnection);
                connect(vehicle, &Vehicle::volumeCurveStep, soundMan, &SoundManager::volumeCurveStep, Qt::DirectConnection);

                if (vehicles.size() !=0)
                {
                    Vehicle *prev =  *(vehicles.end() - 1);
                    prev->setNextVehicle(vehicle);
                    vehicle->setPrevVehicle(prev);
                }                

                vehicles.push_back(vehicle);                
            }            

            vehicle_node = cfg.getNextSection();            
        }        

        for (auto it = vehicles.begin(); it != vehicles.end(); ++it)
        {
            Vehicle *vehicle = *it;
            connect(this, &Train::sendDataToVehicle,v ehicle, &Vehicle::receiveData, Qt::DirectConnection);
        }

    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " is't found");
    }

    // Check train is't empty and return
    return !vehicles.isEmpty();
}
*/
bool Train::addVehiclesBack(const QVarLengthArray<Vehicle *> vehicles_t)
{
    FileSystem &fs = FileSystem::getInstance();

    size_t index = ode_order;
    bool was_empty = vehicles.isEmpty();

    for (auto it = vehicles_t.begin(); it != vehicles_t.end(); ++it)
    {
        connect(*it, &Vehicle::logMessage, this, &Train::logMessage);

        (*it)->setDirection(dir);

        trainMass += (*it)->getMass();
        trainLength += (*it)->getLength();

        size_t s = (*it)->getDegressOfFreedom();

        ode_order += 2 * s;

        (*it)->setIndex(index);
        index = ode_order;

        // Loading sounds
        soundMan->loadSounds((*it)->getSoundsDir());

        connect(*it, &Vehicle::soundPlay, soundMan, &SoundManager::play, Qt::DirectConnection);
        connect(*it, &Vehicle::soundStop, soundMan, &SoundManager::stop, Qt::DirectConnection);
        connect(*it, &Vehicle::soundSetVolume, soundMan, &SoundManager::setVolume, Qt::DirectConnection);
        connect(*it, &Vehicle::soundSetPitch, soundMan, &SoundManager::setPitch, Qt::DirectConnection);
        connect(*it, &Vehicle::volumeCurveStep, soundMan, &SoundManager::volumeCurveStep, Qt::DirectConnection);

        if (vehicles.size() !=0)
        {
            Vehicle *prev =  *(vehicles.end() - 1);
            prev->setNextVehicle(*it);
            (*it)->setPrevVehicle(prev);
        }

        y.resize(ode_order);
        dydt.resize(ode_order);

        if(vehicles.isEmpty())
            y[index + s] = init_velocity / Physics::kmh;
        else
            y[index + s] = getVelocity(vehicles.size() - 1) / Physics::kmh;

        double wheel_radius = (*it)->getWheelDiameter() / 2.0;

        for (size_t j = 1; j < static_cast<size_t>(s); j++)
        {
            y[index + s + j] = y[index + s] / wheel_radius;
        }

        this->vehicles.push_back(*it);

    }


    Journal::instance()->info(QString("Allocated memory for %1 ODE's").arg(ode_order));

    Journal::instance()->info(QString("State vector address: 0x%1")
                              .arg(reinterpret_cast<quint64>(y.data()), 0, 16));

    Journal::instance()->info(QString("State vector derivative address: 0x%1")
                              .arg(reinterpret_cast<quint64>(dydt.data()), 0, 16));

    // Reload of couplings
    if (!loadCouplings(full_config_path))
    {
        Journal::instance()->error("Coupling model is't loaded");
        return false;
    }

    if(was_empty)
    {
        double x0 = init_coord * 1000.0 - dir * this->getFirstVehicle()->getLength() / 2.0;
        y[0] = x0;
    }

    Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[0]).arg(0, 3));

    for (size_t i = 1; i < vehicles.size(); i++)
    {
        double Li_1 = vehicles[i-1]->getLength();
        size_t idxi_1 = vehicles[i-1]->getIndex();

        double Li = vehicles[i]->getLength();
        size_t idxi = vehicles[i]->getIndex();

        y[idxi] = y[idxi_1] - dir *(Li + Li_1) / 2;

        Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[idxi]).arg(i, 3));
    }

    // Set initial conditions
//    Journal::instance()->info("Setting up of initial conditions");
//    setInitConditions(init_data);

    // Brakepipe initialization
    brakepipe = new BrakePipe();

    Journal::instance()->info(QString("Created brakepipe object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(brakepipe), 0, 16));

    brakepipe->setLength(trainLength);
    brakepipe->setNodesNum(vehicles.size());

    if (!no_air)
        brakepipe->setBeginPressure(charging_pressure * Physics::MPa + Physics::pA);

    brakepipe->init(QString(fs.getConfigDir().c_str()) + fs.separator() + "brakepipe.xml");

    initVehiclesBrakes();

    return true;

}

bool Train::addVehiclesFront(const QVarLengthArray<Vehicle *> vehicles_t)
{
/*
    if(vehicles.isEmpty())
        return false;

    size_t index = ode_order;
    for (auto it = vehicles_t.begin(); it != vehicles_t.end(); ++it)
    {
        (*it)->setDirection(dir);

        trainMass += (*it)->getMass();
        trainLength += (*it)->getLength();

        size_t s = (*it)->getDegressOfFreedom();

        ode_order += 2 * s;

        (*it)->setIndex(index);
        index = ode_order;

        // Loading sounds
        soundMan->loadSounds((*it)->getSoundsDir());

        connect(*it, &Vehicle::soundPlay, soundMan, &SoundManager::play, Qt::DirectConnection);
        connect(*it, &Vehicle::soundStop, soundMan, &SoundManager::stop, Qt::DirectConnection);
        connect(*it, &Vehicle::soundSetVolume, soundMan, &SoundManager::setVolume, Qt::DirectConnection);
        connect(*it, &Vehicle::soundSetPitch, soundMan, &SoundManager::setPitch, Qt::DirectConnection);
        connect(*it, &Vehicle::volumeCurveStep, soundMan, &SoundManager::volumeCurveStep, Qt::DirectConnection);

        if (vehicles.size() !=0)
        {
            Vehicle *prev =  *(vehicles.end() - 1);
            prev->setNextVehicle(*it);
            (*it)->setPrevVehicle(prev);
        }

        this->vehicles.push_back(*it);
    }
*/
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::loadCouplings(QString cfg_path)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();

    if (cfg.load(cfg_path))
    {
        QString coupling_module = "";
        if (!cfg.getString("Common", "CouplingModule", coupling_module))
        {
            coupling_module = "default-coupling";
        }

        int num_couplings = static_cast<int>(vehicles.size() - 1);

        if (num_couplings == 0)
            return true;

        for (int i = 0; i < num_couplings; i++)
        {
            Coupling *coupling = loadCoupling(QString(fs.getModulesDir().c_str()) +
                                              fs.separator() +
                                              coupling_module);

            if (coupling == Q_NULLPTR)
            {
                return false;
            }

            Journal::instance()->info(QString("Created Coupling object at address: 0x%1")
                                      .arg(reinterpret_cast<quint64>(coupling), 0, 16));

            Journal::instance()->info("Loaded coupling model from: " + coupling_module);

            coupling->loadConfiguration(QString(fs.getCouplingsDir().c_str()) +
                                        fs.separator() +
                                        coupling_module + ".xml");

            coupling->reset();

            couplings.push_back(coupling);
        }
    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " is't found");
    }

    return couplings.size() != 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::setInitConditions(const init_data_t &init_data)
{
    for (size_t i = 0; i < vehicles.size(); i++)
    {
        Vehicle *vehicle = vehicles[i];

        size_t s = vehicle->getDegressOfFreedom();
        size_t idx = vehicle->getIndex();

        y[idx + s] = init_data.init_velocity / Physics::kmh;

        double wheel_radius = vehicle->getWheelDiameter() / 2.0;

        for (size_t j = 1; j < static_cast<size_t>(s); j++)
        {
            y[idx + s + j] = y[idx + s] / wheel_radius;
        }
    }

    double x0 = init_data.init_coord * 1000.0 - dir * this->getFirstVehicle()->getLength() / 2.0;
    y[0] = x0;    

    Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[0]).arg(0, 3));

    for (size_t i = 1; i < vehicles.size(); i++)
    {
        double Li_1 = vehicles[i-1]->getLength();
        size_t idxi_1 = vehicles[i-1]->getIndex();

        double Li = vehicles[i]->getLength();
        size_t idxi = vehicles[i]->getIndex();

        y[idxi] = y[idxi_1] - dir *(Li + Li_1) / 2;

        Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[idxi]).arg(i, 3));
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::initVehiclesBrakes()
{
    Journal::instance()->info("Initialization of vehicles brake devices...");

    for (size_t i = 0; i < vehicles.size(); ++i)
    {
        double pTM = brakepipe->getPressure(i);
        vehicles[i]->initBrakeDevices(charging_pressure, pTM, init_main_res_pressure);
    }
}


//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
void Train::topologyStep()
{
    for (size_t i = 0; i < getVehicles()->size(); ++i)
    {
        VehicleController *vc = topology.getVehicleController(i);
        vc[i].setRailwayCoord(train->getVehicles()->at(i)->getRailwayCoord());
    }
}
*/
void Train::process()
{
    double tau = 0;
    double integration_time = static_cast<double>(time_controls.integration_time_interval) / 1000.0;
    bool is_step_correct = true;
    // Integrate all ODE in train motion model
    while ( (tau <= integration_time) &&
            is_step_correct)
    {
        preStep(time_controls.t);

        is_step_correct = step(time_controls.t, time_controls.dt);

        //topologyStep();

        tau += time_controls.dt;
        time_controls.t += time_controls.dt;

        postStep(time_controls.t);

        QVarLengthArray<VehicleData> viewer_copy;

        for (auto it = vehicles.begin(); it != vehicles.end(); ++it)
        {
            VehicleData copy = static_cast<VehicleData>(**it);
            viewer_copy.push_back(copy);
        }

        emit posDataReady(viewer_copy);
    }
}
