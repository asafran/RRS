#include    "train.h"

#include    "CfgReader.h"
#include    "physics.h"
#include    "Journal.h"
#include    "connector.h"


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::Train(Topology *topology, QObject *parent) : OdeSystem(parent)
  , trainMass(0.0)
  , trainLength(0.0)
  , ode_order(0)
//  , profile(profile)
  , train_motion_solver(nullptr)
  , brakepipe(nullptr)
  , soundMan(nullptr)
  , topo(topology)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::Train(Topology *topology, init_data_t init_data, QObject *parent) : OdeSystem(parent)
  , trainMass(0.0)
  , trainLength(0.0)
  , ode_order(0)
//  , profile(profile)
  , train_motion_solver(nullptr)
  , brakepipe(nullptr)
  , soundMan(nullptr)
  , topo(topology)
  , init_data(init_data)
{

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
bool Train::init()
{
    solver_config = init_data.solver_config;
    // Solver loading
    FileSystem &fs = FileSystem::getInstance();
    QString solver_path = fs.getLibraryDir() + fs.separator() + solver_config.method;

    train_motion_solver = loadSolver(solver_path);

    Journal::instance()->info(QString("Created Solver object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(train_motion_solver), 0, 16));

    if (train_motion_solver == Q_NULLPTR)
    {
        Journal::instance()->error("Solver " + solver_path + " is't found");
        return false;
    }

    Journal::instance()->info("Loaded solver: " + solver_path);

    try
    {
        soundMan = new SoundManager();

        Journal::instance()->info(QString("Created SoundManager at address: 0x%1")
                                      .arg(reinterpret_cast<quint64>(soundMan), 0, 16));

    } catch (const std::bad_alloc &)
    {
        Journal::instance()->error("Sound manager is;t created");
    }

    // Brakepipe initialization
    brakepipe = new BrakePipe();

    Journal::instance()->info(QString("Created brakepipe object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(brakepipe), 0, 16));

}

void Train::updatePos(const topology_pos_t &tp)
{
    if (vehicles.isEmpty())
    {
        return;
    }

    vehicles[0]->setTrajCoord(tp.traj_coord);
    vehicles[0]->setInitCurrentTraj(topo->getTrajList()[tp.traj_name]);
    vehicles[0]->setTopologyDirection(tp.dir);
    vehicles[0]->setInitRailwayCoord(0.0);

    double traj_coord = tp.traj_coord;
    topo->getTrajList()[tp.traj_name]->setBusy(vehicles[0]);

    updatePos();

    Journal::instance()->info("Train placed on topology_pos_t");

}

void Train::updatePos()
{
    int direction = vehicles[0]->getTopologyDirection();
    double traj_coord = vehicles[0]->getTrajCoord();
    Trajectory *cur_traj = vehicles[0]->getCurrentTraj();

    for (auto i = vehicles.begin() + 1; i != vehicles.end(); ++i)
    {
        double L = (*(i-1))->getLength() + (*i)->getLength() / 2.0;

        traj_coord = traj_coord - direction * L;

        while (traj_coord < 0)
        {
            Connector *conn = cur_traj->getBwdConnector();

            if (conn == Q_NULLPTR)
            {
                Journal::instance()->error("Error placing train on topology_pos_t");
                return;
            }

            cur_traj = conn->getBwdTraj();

            if (cur_traj == Q_NULLPTR)
            {
                Journal::instance()->error("Error placing train on topology_pos_t");
                return;
            }

            traj_coord = cur_traj->getLength() + traj_coord;
        }

        while (traj_coord > cur_traj->getLength())
        {
            traj_coord = traj_coord - cur_traj->getLength();

            Connector *conn = cur_traj->getFwdConnector();

            if (conn == Q_NULLPTR)
            {
                Journal::instance()->error("Error placing train on topology_pos_t");
                return;
            }

            cur_traj = conn->getFwdTraj();

            if (cur_traj == Q_NULLPTR)
            {
                Journal::instance()->error("Error placing train on topology_pos_t");
                return;
            }

            if (conn->isReversed())
            {
                traj_coord = cur_traj->getLength() - traj_coord;
                direction *= -1;
            }
        }

        (*i)->setTrajCoord(traj_coord);
        (*i)->setInitCurrentTraj(cur_traj);
        (*i)->setTopologyDirection(direction);
        (*i)->setInitRailwayCoord(0.0);

        cur_traj->setBusy(*i);
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
                    vehicle->getLength() / 2 -
                    vehicle1->getLength() / 2;

            double dv = Y[idx + s] - Y[idx1 + s1];

            Coupling *coup = *coup_it;
            double R = coup->getForce(ds, dv);
            ++coup_it;

            vehicle->setBackwardForce(R);
            vehicle1->setForwardForce(R);
        }

        profile_element_t pe = vehicle->getCurrentTraj()->getProfileElement(vehicle->getTrajCoord());

        pe.inclination *= vehicle->getTopologyDirection();

        vehicle->setInclination(pe.inclination);
        vehicle->setCurvature(pe.curvature);

        transfer_vector_t a = vehicle->getAcceleration(Y, t);

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

        brakepipe->setAuxRate(j, vehicle->getBrakepipeAuxRate());
        vehicle->setBrakepipePressure(brakepipe->getPressure(j));
        vehicle->integrationStep(y, t, dt);

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
QVarLengthArray<VehicleData, MAX_NUM_VEHICLES> Train::postStep(double t)
{
    (void) t;
    auto end = vehicles.end();
    auto begin = vehicles.begin();

    QVarLengthArray<VehicleData, MAX_NUM_VEHICLES> data;

    for (auto i = begin; i != end; ++i)
    {
         data.append(**i);
    }
    data.squeeze();
    return data;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QVector<Vehicle *>::const_iterator Train::getFirstVehicle()
{
    return vehicles.cbegin();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QVector<Vehicle *>::const_iterator Train::getLastVehicle()
{
    return (vehicles.cend() - 1);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QVector<Vehicle *>::const_iterator Train::getEndVehicle()
{
    return vehicles.cend();
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
/*------------------------------------------------------------------------------
int Train::getDirection() const
{
    return dir;
}
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QVector<Vehicle *> Train::getVehicles()
{
    return vehicles;
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

bool Train::addVehiclesBack(QVector<Vehicle *> vehicles_add, bool reversed)
{
    Journal::instance()->info(QString("Pushing back vehicles, current index %1").arg(ode_order));

    if(reversed)
    {
        for (auto vehicle = vehicles_add.rbegin(); vehicle != vehicles_add.rend(); ++vehicle)
        {
            appendVehicle(*vehicle);

            if (*vehicle != *vehicles.begin())
            {
                Journal::instance()->info(QString("Set prev/next pointers"));
                Vehicle *prev =  *(vehicle - 1);
                prev->setNextVehicle(*vehicle);
                (*vehicle)->setPrevVehicle(prev);
                Journal::instance()->info(QString("Vehicle 0x%1: next 0x%2, vehicle 0x%2: prev 0x%1")
                                          .arg(reinterpret_cast<quint64>(prev), 0, 16)
                                          .arg(reinterpret_cast<quint64>(*vehicle), 0, 16));
            }
        }
    }
    else
    {
        for (auto vehicle = vehicles_add.begin(); vehicle != vehicles_add.end(); ++vehicle)
        {
            appendVehicle(*vehicle);

            if (*vehicle != *vehicles.begin())
            {
                Journal::instance()->info(QString("Set prev/next pointers"));
                Vehicle *prev =  *(vehicle - 1);
                prev->setNextVehicle(*vehicle);
                (*vehicle)->setPrevVehicle(prev);
                Journal::instance()->info(QString("Vehicle 0x%1: next 0x%2, vehicle 0x%2: prev 0x%1")
                                          .arg(reinterpret_cast<quint64>(prev), 0, 16)
                                          .arg(reinterpret_cast<quint64>(*vehicle), 0, 16));
            }
        }
    }
    Journal::instance()->info(QString("State vector address: 0x%1")
                              .arg(reinterpret_cast<quint64>(y.data()), 0, 16));

    Journal::instance()->info(QString("State vector derivative address: 0x%1")
                              .arg(reinterpret_cast<quint64>(dydt.data()), 0, 16));

    // Reload of couplings
    if (!loadCouplings())
    {
        Journal::instance()->error("Coupling model is't loaded");
        return false;
    }

    y[0] = 0;
    updateTrainCoords();
    updatePos();

    brakepipe->setLength(trainLength);
    brakepipe->setNodesNum(vehicles.size());
//    brakepipe->setBeginPressure(vehicles.first()->getBrakepipeBeginPressure());
//    brakepipe->init(fs.getConfigDir() + fs.separator() + "brakepipe.xml");
    brakepipe->init(init_data.brakepipe_conf);

    updateBrakepipe();
}

void Train::appendVehicle(Vehicle *vehicle)
{
    connectToTrain(vehicle);

    size_t s = vehicle->getDegressOfFreedom();

//    Journal::instance()->info(QString("Set index %1").arg(index));
    size_t index = ode_order;

    ode_order += 2 * s;

    vehicle->setIndex(index);
//    index = ode_order;

    Journal::instance()->info(QString("Update index %1").arg(index));

    Journal::instance()->info(QString("Resize ODE vectors, order %1").arg(ode_order));
    y.resize(ode_order);
    dydt.resize(ode_order);

    y[index + s] = vehicle->getVelocity();

    Journal::instance()->info(QString("Update velocity %1, %2 kmh").arg(vehicle->getVelocity())
                              .arg(vehicle->getVelocity() * Physics::kmh));

    double wheel_radius = vehicle->getWheelDiameter() / 2.0;

    for (size_t j = 1; j < static_cast<size_t>(s); j++)
    {
        y[index + s + j] = y[index + s] / wheel_radius;
    }

    vehicles.append(vehicle);
}

bool Train::addVehiclesFront(QVector<Vehicle *> vehicles_add)
{
    if (vehicles.isEmpty())
        return false;

    size_t index = 0;

    Journal::instance()->info(QString("Pushing back vehicles, current index %1").arg(index));
    QVector<Vehicle *> tmp;

    for (auto vehicle = vehicles_add.rbegin(); vehicle != vehicles_add.rend(); ++vehicle)
    {
        connectToTrain(*vehicle);

        size_t s = (*vehicle)->getDegressOfFreedom();

        ode_order += 2 * s;

//        Journal::instance()->info(QString("Set index %1").arg(index));

        (*vehicle)->setIndex(index);
//        index = ode_order;
        index += 2 * s;

        Journal::instance()->info(QString("Update index %1").arg(index));

        Journal::instance()->info(QString("Set prev/next pointers"));

        if (vehicle != vehicles_add.rbegin())
        {
            Vehicle *prev =  *(vehicle - 1);
            prev->setNextVehicle(*vehicle);
            (*vehicle)->setPrevVehicle(prev);
            Journal::instance()->info(QString("Vehicle 0x%1: next 0x%2, vehicle 0x%2: prev 0x%1")
                                      .arg(reinterpret_cast<quint64>(prev), 0, 16)
                                      .arg(reinterpret_cast<quint64>(*vehicle), 0, 16));
        }

        y.resize(ode_order);
        dydt.resize(ode_order);

        Journal::instance()->info(QString("Resize ODE vectors, order %1").arg(ode_order));

        tmp.append(*vehicle);
    }

//    tmp.append(vehicles_add);

    for (auto vehicle = vehicles.begin(); vehicle != vehicles.end(); ++vehicle)
    {
        size_t s = (*vehicle)->getDegressOfFreedom();
        (*vehicle)->setIndex(index);

        Journal::instance()->info(QString("Update index %1").arg(index));

        index += 2 * s;
    }

    tmp.append(vehicles);

    for (auto vehicle = tmp.begin(); vehicle != tmp.end(); ++vehicle)
    {
        size_t s = (*vehicle)->getDegressOfFreedom();
        size_t idx = (*vehicle)->getIndex();

        y[idx + s] = (*vehicle)->getVelocity();
        Journal::instance()->info(QString("Update velocity %1, %2 kmh").arg((*vehicle)->getVelocity())
                                  .arg((*vehicle)->getVelocity() * Physics::kmh));

        double wheel_radius = (*vehicle)->getWheelDiameter() / 2.0;

        for (size_t j = 1; j < static_cast<size_t>(s); j++)
        {
            y[idx + s + j] = y[idx + s] / wheel_radius;
        }
    }

    vehicles = tmp;

//    y[0] = vehicles.first()->getRailwayCoord();

    Journal::instance()->info(QString("State vector address: 0x%1")
                              .arg(reinterpret_cast<quint64>(y.data()), 0, 16));

    Journal::instance()->info(QString("State vector derivative address: 0x%1")
                              .arg(reinterpret_cast<quint64>(dydt.data()), 0, 16));



    // Reload of couplings
    if (!loadCouplings())
    {
        Journal::instance()->error("Coupling model is't loaded");
        return false;
    }
    y[0] = 0;
    updateTrainCoords();
    updatePos();

    // Brakepipe initialization
    delete brakepipe;

    brakepipe = new BrakePipe();

    Journal::instance()->info(QString("Created brakepipe object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(brakepipe), 0, 16));

    brakepipe->setLength(trainLength);
    brakepipe->setNodesNum(vehicles.size());
    brakepipe->init(init_data.brakepipe_conf);

    updateBrakepipe();
}

void Train::connectToTrain(Vehicle * vehicle)
{
    vehicle->disconnect();
    vehicle->setParent(this);
    connect(vehicle, &Vehicle::logMessage, this, &Train::logMessage);

//    (*vehicle)->setDirection(dir);

    trainMass += vehicle->getMass();
    trainLength += vehicle->getLength();

    Journal::instance()->info(QString("Update train mass and lenght: %1, %2").arg(trainMass).arg(trainLength));

    // Loading sounds
    soundMan->loadSounds(vehicle->getSoundsDir());

    connect(vehicle, &Vehicle::soundPlay, soundMan, &SoundManager::play, Qt::DirectConnection);
    connect(vehicle, &Vehicle::soundStop, soundMan, &SoundManager::stop, Qt::DirectConnection);
    connect(vehicle, &Vehicle::soundSetVolume, soundMan, &SoundManager::setVolume, Qt::DirectConnection);
    connect(vehicle, &Vehicle::soundSetPitch, soundMan, &SoundManager::setPitch, Qt::DirectConnection);
    connect(vehicle, &Vehicle::volumeCurveStep, soundMan, &SoundManager::volumeCurveStep, Qt::DirectConnection);
}

void Train::updateTrainCoords()
{
    Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[0]).arg(0, 3));

    for (size_t i = 1; i < vehicles.size(); i++)
    {
        double Li_1 = vehicles[i-1]->getLength();
        size_t idxi_1 = vehicles[i-1]->getIndex();

        double Li = vehicles[i]->getLength();
        size_t idxi = vehicles[i]->getIndex();

        y[idxi] = y[idxi_1] - (Li + Li_1) / 2;

        Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[idxi]).arg(i, 3));
    }
}

void Train::updateTrainCoords(double coord)
{
    double x0 = coord * 1000.0 - (*this->getFirstVehicle())->getLength() / 2.0;
    y[0] = x0;
    updatePos();
}

void Train::setSpeed(double V, const double coeff)
{
    for (auto vehicle = vehicles.begin(); vehicle != vehicles.end(); ++vehicle)
    {
        size_t s = (*vehicle)->getDegressOfFreedom();
        size_t idx = (*vehicle)->getIndex();

        y[idx + s] = V / coeff;

        double wheel_radius = (*vehicle)->getWheelDiameter() / 2.0;

        for (size_t j = 1; j < static_cast<size_t>(s); j++)
        {
            y[idx + s + j] = y[idx + s] / wheel_radius;
        }
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::loadCouplings()
{
    FileSystem &fs = FileSystem::getInstance();

    int num_couplings = (vehicles.size() - 1) - couplings.size();

    if (num_couplings == 0)
        return true;

    for (int i = 0; i < num_couplings; i++)
    {
        Coupling *coupling = loadCoupling(fs.getModulesDir() +
                                          fs.separator() +
                                          init_data.coupling_module);

        if (coupling == Q_NULLPTR)
        {
            return false;
        }

        Journal::instance()->info(QString("Created Coupling object at address: 0x%1")
                                  .arg(reinterpret_cast<quint64>(coupling), 0, 16));

        Journal::instance()->info("Loaded coupling model from: " + init_data.coupling_module);

        coupling->loadConfiguration(fs.getCouplingsDir() +
                                    fs.separator() +
                                    init_data.coupling_module + ".xml");

        coupling->reset();

        couplings.push_back(coupling);
    }

    return !couplings.isEmpty();
}

//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
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
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::initVehiclesBrakes(double charging_pressure, double main_res_pressure)
{
    Journal::instance()->info("Initialization of vehicles brake devices...");

    for (int i = 0; i < vehicles.size(); ++i)
    {
        double pTM = brakepipe->getPressure(i);
        vehicles.at(i)->initBrakeDevices(charging_pressure, pTM, main_res_pressure);
    }
}

void Train::updateBrakepipe()
{
    Journal::instance()->info("Update vehicles brake devices...");

    for (int i = 0; i < vehicles.size(); ++i)
    {
        brakepipe->setPressure(vehicles.at(i)->getBrakepipePressure(),i);
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
*/
