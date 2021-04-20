//------------------------------------------------------------------------------
//
//      Video client's window manager
//      (c) maisvendoo
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Video client's window manager
 * \copyright maisvendoo
 * \author maisvendoo
 * \date
 */

#include    "viewer.h"

//#include    <osg/BlendFunc>
//#include    <osg/CullFace>
//#include    <osg/GraphicsContext>
//#include    <osgDB/FileUtils>
//#include    <osgDB/FileNameUtils>
//#include    <osgViewer/ViewerEventHandlers>
//#include    <osg/LightModel>
//#include    <osgViewer/View>

#include    <vsgXchange/all.h>
#include    <vsg/utils/CommandLine.h>
#include    <vsg/viewer/Viewer.h>
#include    <vsg/utils/CommandLine.h>
#include    <vsg/utils/CommandLine.h>
#include    <vsg/utils/CommandLine.h>


#include    "filesystem.h"
//#include    "config-reader.h"
#include    "CfgReader.h"

#include    <sstream>
#include    <fstream>


#include    "notify.h"
#include    "abstract-loader.h"
#include    "lighting.h"
#include    "qt-events.h"
#include    "screen-capture.h"
#include    "hud.h"
#include    "rails-manipulator.h"
#include    "free-manipulator.h"
#include    "stat-manipulator.h"
#include    "train-manipulator.h"
#include    "camera-switcher.h"


#include    <QObject>
#include    <iostream>
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
RouteViewer::RouteViewer(int argc, char *argv[])
  : is_ready(false)
  , keyboard(nullptr)
{
    is_ready = init(argc, argv);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
RouteViewer::~RouteViewer()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool RouteViewer::isReady() const
{
    return is_ready;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int RouteViewer::run()
{
    // Qt signals processing
    viewer.addEventHandler(new QtEventsHandler());

    // Keyboard events handler
    keyboard = new KeyboardHandler();
    viewer.addEventHandler(keyboard);

    QObject::connect(keyboard, &KeyboardHandler::sendKeyBoardState,
                     &client, &NetworkClient::receiveKeysState);


    osg::ref_ptr<osgViewer::StatsHandler> statsHandler = new osgViewer::StatsHandler;
    statsHandler->setKeyEventTogglesOnScreenStats(osgGA::GUIEventAdapter::KEY_F11);

    viewer.addEventHandler(statsHandler.get());

    //viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

    // Cabine camera manipulator
    osg::ref_ptr<RailsManipulator> rm = new RailsManipulator(settings);
    QObject::connect(train_ext_handler, &TrainExteriorHandler::sendCameraPosition,
                     rm, &RailsManipulator::getCameraPosition);

    // Free camera manipulator
    osg::ref_ptr<FreeManipulator> fm = new FreeManipulator(settings);
    QObject::connect(train_ext_handler, &TrainExteriorHandler::sendCameraPosition,
                     fm, &FreeManipulator::getCameraPosition);

    // Static camera manipulator
    osg::ref_ptr<StaticManipulator> sm_right = new StaticManipulator(settings, true);
    QObject::connect(train_ext_handler, &TrainExteriorHandler::sendCameraPosition,
                     sm_right, &StaticManipulator::getCameraPosition);

    // Static camera manipulator
    osg::ref_ptr<StaticManipulator> sm_left = new StaticManipulator(settings, false);
    QObject::connect(train_ext_handler, &TrainExteriorHandler::sendCameraPosition,
                     sm_left, &StaticManipulator::getCameraPosition);

    // Static camera manipulator
    osg::ref_ptr<TrainManipulator> tm = new TrainManipulator(settings);
    QObject::connect(train_ext_handler, &TrainExteriorHandler::sendCameraPosition,
                     tm, &TrainManipulator::getCameraPosition);

    osg::ref_ptr<CameraSwitcher> cs = new CameraSwitcher;
    cs->addMatrixManipulator(osgGA::GUIEventAdapter::KEY_F2, "cabine_view", rm.get());
    cs->addMatrixManipulator(osgGA::GUIEventAdapter::KEY_F3, "train_view", tm.get());
    cs->addMatrixManipulator(osgGA::GUIEventAdapter::KEY_F4, "free_view", fm.get());
    cs->addMatrixManipulator(osgGA::GUIEventAdapter::KEY_F5, "static_view", sm_right.get());
    cs->addMatrixManipulator(osgGA::GUIEventAdapter::KEY_F6, "static_view", sm_left.get());

    viewer.setCameraManipulator(cs.get());

    return viewer.run();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool RouteViewer::init(int argc, char *argv[])
{
/*
    FileSystem &fs = FileSystem::getInstance();

    // Read settings from config file
    settings = loadSettings(fs.getConfigDir() + fs.separator() + "settings.xml");

    // Parse command line
    CommandLineParser parser(argc, argv);
    cmd_line_t cmd_line = parser.getCommadLine();
    overrideSettingsByCommandLine(cmd_line, settings);

    // Notify settings
    osg::NotifySeverity level = osg::INFO;

    if (settings.notify_level == "INFO")
        level = osg::INFO;

    if (settings.notify_level == "WARN")
        level = osg::WARN;

    if (settings.notify_level == "FATAL")
        level = osg::FATAL;

    osg::setNotifyLevel(level);
    std::string logs_path = fs.getLogsDir();
    osg::setNotifyHandler(new LogFileHandler(logs_path + fs.separator() + "viewer.log"));

    // Load selected route
    if (!loadRoute(cmd_line.route_dir.value))
    {
        std::cerr << "Route from " << cmd_line.route_dir.value << " is't loaded" << std::endl;
        return false;
    }

    // Init graphical engine settings
    if (!initEngineSettings(root.get()))
        return false;

    // Init display settings
    if (!initDisplay(&viewer, settings))
        return false;


    osg::ref_ptr<osgViewer::ScreenCaptureHandler::CaptureOperation> writeFile =
            new WriteToFileOperation(fs.getScreenshotsDir());

    osg::ref_ptr<osgViewer::ScreenCaptureHandler> screenCaptureHandler =
            new osgViewer::ScreenCaptureHandler(writeFile.get());

    viewer.addEventHandler(screenCaptureHandler.get());
    viewer.addEventHandler(new ScreenCaptureHandler(screenCaptureHandler.get()));

    HUD *hud = new HUD(settings.width, settings.height);
    root->addChild(hud->getCamera());

    viewer.addEventHandler(new KeyboardHUDHandler(hud->getScene()));

    QObject::connect(train_ext_handler, &TrainExteriorHandler::setStatusBar,
                     hud, &HUD::setStatusBar);

    osgDB::DatabasePager *dp = viewer.getDatabasePager();
    dp->setDoPreCompile(true);
    dp->setTargetMaximumNumberOfPageLOD(1000);
*/

    // set up defaults and read command line arguments to override them

    auto options = vsg::Options::create();
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    options->add(vsgXchange::all::create());

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "RRS";

    FileSystem &fs = FileSystem::getInstance();

    // Read settings from config file

    settings = loadSettings(QString::fromUtf8(fs.getConfigDir().c_str()) + QDir::separator().toLatin1() + "settings.xml");


    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);
    arguments.read(options);
    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
    if (arguments.read("--double-buffer")) windowTraits->swapchainPreferences.imageCount = 2;
    if (arguments.read("--triple-buffer")) windowTraits->swapchainPreferences.imageCount = 3; // default
    if (arguments.read("--IMMEDIATE")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
    if (arguments.read("--FIFO")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    if (arguments.read("--FIFO_RELAXED")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_RELAXED_KHR;
    if (arguments.read("--MAILBOX")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;

    if (arguments.read({"--fullscreen", "--fs"})) windowTraits->fullscreen = true;
    if (arguments.read({"--window", "-w"}, windowTraits->width, windowTraits->height)) { windowTraits->fullscreen = false; }
    if (arguments.read({"--no-frame", "--nf"})) windowTraits->decoration = false;
    if (arguments.read("--or")) windowTraits->overrideRedirect = true;
    if (arguments.read("--d32")) windowTraits->depthFormat = VK_FORMAT_D32_SFLOAT;
    arguments.read("--screen", windowTraits->screenNum);
    arguments.read("--display", windowTraits->display);
    arguments.read("--samples", windowTraits->samples);
/*
    auto numFrames = arguments.value(-1, "-f");
    auto loadLevels = arguments.value(0, "--load-levels");
    auto horizonMountainHeight = arguments.value(0.0, "--hmh");
    auto skyboxFilename = arguments.value(vsg::Path(), "--skybox");
*/

    simulator_init_t command_line;

    if (arguments.read("--train-config", command_line.train_config.value)) command_line.train_config.is_present = true;
    if (arguments.read("--clear-log")) command_line.clear_log.is_present = command_line.clear_log.value = true;


    if (arguments.read("--init-coord"))
    {
        command_line.init_coord.is_present = true;
        QString tmp;
        arguments.read("--init-coord", tmp);
        command_line.init_coord.value = tmp.toDouble();
    }

    if (arguments.read("--direction"))
    {
        command_line.direction.is_present = true;
        QString tmp;
        arguments.read("--direction", tmp);
        command_line.direction.value = tmp.toInt();
    }

    auto routeFilename = arguments.value(vsg::Path(), "--route");

    if (arguments.errors()) return arguments.writeErrorMessages(std::cerr);

    // Load selected route
    if (!loadRoute(routeFilename))
    {
        std::cerr << "Route from " << routeFilename << " is't loaded" << std::endl;
        return false;
    }
    if (!loadTrain(command_line))
    {
        std::cerr << "Failed to load train " << command_line.train_config.value.toStdString() << std::endl;
        return false;
    }

    // Init graphical engine settings
    if (!initEngineSettings(root.get()))
        return false;

    // Init display settings
    if (!initDisplay(&viewer, settings))
        return false;


    auto group = vsg::Group::create();
    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
settings_t RouteViewer::loadSettings(const QString &cfg_path) const
{
    settings_t settings;

    CfgReader cfg;

    if (cfg.load(cfg_path))
    {
        QString secName = "Viewer";

        cfg.getString(secName, "HostAddress", settings.host_addr);
        cfg.getInt(secName, "Port", settings.port);
        cfg.getInt(secName, "Width", settings.width);
        cfg.getInt(secName, "Height", settings.height);
        cfg.getBool(secName, "FullScreen", settings.fullscreen);
        cfg.getBool(secName, "VSync", settings.vsync);
        cfg.getBool(secName, "LocalMode", settings.localmode);
        cfg.getInt(secName, "posX", settings.x);
        cfg.getInt(secName, "posY", settings.y);
        cfg.getDouble(secName, "FovY", settings.fovy);
        cfg.getDouble(secName, "zNear", settings.zNear);
        cfg.getDouble(secName, "zFar", settings.zFar);
        int tmp = 0;
        cfg.getInt(secName, "ScreenNumber", tmp);
        settings.screen_number = tmp;
        cfg.getBool(secName, "WindowDecoration", settings.window_decoration);
        cfg.getBool(secName, "DoubleBuffer", settings.double_buffer);
        cfg.getBool(secName, "Samples", settings.samples);
        cfg.getInt(secName, "RequestInterval", settings.request_interval);
        cfg.getInt(secName, "ReconnectInterval", settings.reconnect_interval);
        cfg.getDouble(secName, "MotionBlur", settings.persistence);
        cfg.getString(secName, "NotifyLevel", settings.notify_level);
        cfg.getFloat(secName, "ViewDistance", settings.view_distance);

        cfg.getFloat(secName, "CabineCamRotCoeff", settings.cabine_cam_rot_coeff);
        cfg.getFloat(secName, "CabineCamFovYStep", settings.cabine_cam_fovy_step);
        cfg.getFloat(secName, "CabineCamSpeed", settings.cabine_cam_speed);

        cfg.getFloat(secName, "ExtCamInitDist", settings.ext_cam_init_dist);
        cfg.getFloat(secName, "ExtCamInitHeight", settings.ext_cam_init_height);
        cfg.getFloat(secName, "ExtCamInitShift", settings.ext_cam_init_shift);
        cfg.getFloat(secName, "ExtCamRotCoeff", settings.ext_cam_rot_coeff);
        cfg.getFloat(secName, "ExtCamSpeed", settings.ext_cam_speed);
        cfg.getFloat(secName, "ExtCamSpeedCoeff", settings.ext_cam_speed_coeff);
        cfg.getFloat(secName, "ExtCamMinDist", settings.ext_cam_min_dist);
        cfg.getFloat(secName, "ExtCamInitAngleH", settings.ext_cam_init_angle_H);
        cfg.getFloat(secName, "ExtCamInitAngleV", settings.ext_cam_init_angle_V);

        QString tmp_str = "";
        cfg.getString(secName, "FreeCamInitPos", tmp_str);
        std::istringstream ss(tmp_str.toStdString());

        ss >> settings.free_cam_init_pos.x()
           >> settings.free_cam_init_pos.y()
           >> settings.free_cam_init_pos.z();

        cfg.getFloat(secName, "FreeCamRotCoeff", settings.free_cam_rot_coeff);
        cfg.getFloat(secName, "FreeCamSpeed", settings.free_cam_speed);
        cfg.getFloat(secName, "FreeCamSpeedCoeff", settings.free_cam_speed_coeff);
        cfg.getFloat(secName, "FreeCamFovY", settings.free_cam_fovy_step);

        cfg.getFloat(secName, "StatCamDist", settings.stat_cam_dist);
        cfg.getFloat(secName, "StatCamHeight", settings.stat_cam_height);
        cfg.getFloat(secName, "StatCamShift", settings.stat_cam_shift);


//------------------------------------------------------------------------------

        secName = "InitData";
        if (!cfg.getDouble(secName, "InitCoord", settings.simulator_init_data.init_coord))
        {
            settings.simulator_init_data.init_coord = 1.0;
        }

        if (!cfg.getDouble(secName, "InitVelocity", settings.simulator_init_data.init_velocity))
        {
            settings.simulator_init_data.init_velocity = 0.0;
        }

        if (!cfg.getString(secName, "Profile", settings.simulator_init_data.profile_path))
        {
            settings.simulator_init_data.profile_path = "default";
        }

        if (!cfg.getDouble(secName, "ProfileStep", settings.simulator_init_data.prof_step))
        {
            settings.simulator_init_data.prof_step = 100.0;
        }

        if (!cfg.getString(secName, "TrainConfig", settings.simulator_init_data.train_config))
        {
            settings.simulator_init_data.train_config = "default-train";
        }

        if (!cfg.getInt(secName, "IntegrationTimeInterval", settings.simulator_init_data.integration_time_interval))
        {
            settings.simulator_init_data.integration_time_interval = 100;
        }

        if (!cfg.getInt(secName, "ControlTimeInterval", settings.simulator_init_data.control_time_interval))
        {
            settings.simulator_init_data.control_time_interval = 50;
        }

//        control_delay = static_cast<double>(settings.simulator_init_data.control_time_interval) / 1000.0;

        settings.simulator_init_data.debug_print = false;

//------------------------------------------------------------------------------

        secName = "Solver";
        if (!cfg.getString(secName, "Method", settings.simulator_init_data.solver_config.method))
        {
            settings.simulator_init_data.solver_config.method = "rkf5";
        }

//        Journal::instance()->info("Integration method: " + settings.simulator_init_data.solver_config.method);

        if (!cfg.getDouble(secName, "StartTime", settings.simulator_init_data.solver_config.start_time))
        {
            settings.simulator_init_data.solver_config.start_time = 0;
        }

//        Journal::instance()->info("Start time: " + QString("%1").arg(settings.simulator_init_data.solver_config.start_time));

        if (!cfg.getDouble(secName, "StopTime", settings.simulator_init_data.solver_config.stop_time))
        {
            settings.simulator_init_data.solver_config.stop_time = 10.0;
        }

//        Journal::instance()->info("Stop time: " + QString("%1").arg(settings.simulator_init_data.solver_config.stop_time));

        if (!cfg.getDouble(secName, "InitStep", settings.simulator_init_data.solver_config.step))
        {
            settings.simulator_init_data.solver_config.step = 1e-4;
        }

//        Journal::instance()->info("Initial integration step: " + QString("%1").arg(settings.simulator_init_data.solver_config.step));

        if (!cfg.getDouble(secName, "MaxStep", settings.simulator_init_data.solver_config.max_step))
        {
            settings.simulator_init_data.solver_config.max_step = 1e-2;
        }

    }

    return settings;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RouteViewer::applySettings(const settings_t &settings, vsg::ref_ptr<vsg::WindowTraits> windowTraits)
{
    if (settings.double_buffer) windowTraits->swapchainPreferences.imageCount = 2;
    else windowTraits->swapchainPreferences.imageCount = 3; // default
    if (!settings.vsync) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
    //windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    if (settings.vsync) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_RELAXED_KHR;
    //windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;

    if (settings.fullscreen) windowTraits->fullscreen = true;
    else {
        windowTraits->width = settings.width;
        windowTraits->height = settings.height;
        windowTraits->fullscreen = false;
    }

    if (!settings.window_decoration) windowTraits->decoration = false;
    //windowTraits->depthFormat = VK_FORMAT_D32_SFLOAT;
    windowTraits->screenNum = settings.screen_number;
    windowTraits->display = settings.screen_number;
    windowTraits->samples = settings.samples;
}

//------------------------------------------------------------------------------
//
/*------------------------------------------------------------------------------
void RouteViewer::overrideSettingsByCommandLine(const cmd_line_t &cmd_line,
                                                settings_t &settings)
{
    if (cmd_line.host_addr.is_present)
        settings.host_addr = cmd_line.host_addr.value;

    if (cmd_line.port.is_present)
        settings.port = cmd_line.port.value;

    if (cmd_line.width.is_present)
        settings.width = cmd_line.width.value;

    if (cmd_line.height.is_present)
        settings.height = cmd_line.height.value;

    if (cmd_line.fullscreen.is_present)
        settings.fullscreen = cmd_line.fullscreen.value;

    if (cmd_line.localmode.is_present)
        settings.localmode = cmd_line.localmode.value;

    if (cmd_line.train_config.is_present)
        settings.train_config = cmd_line.train_config.value;

    if (cmd_line.notify_level.is_present)
        settings.notify_level = cmd_line.notify_level.value;

    if (cmd_line.direction.is_present)
        settings.direction = cmd_line.direction.value;
}
*/
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool RouteViewer::loadRoute(const vsg::Path &routeDir)
{
    if (routeDir.empty())
    {
        std::cerr << "ERROR: Route path is empty" << std::endl;
        std::cerr << "Route path: " << routeDir << std::endl;
        return false;
    }

    FileSystem &fs = FileSystem::getInstance();
    vsg::Path routeType(routeDir + fs.separator() + "route-type");

    if (!vsg::fileExists(routeType))
    {
        std::cerr << "ERROR: File route-type is not found in route directory" << std::endl;
        return false;
    }

    std::ifstream stream(routeType);

    if (!stream.is_open())
    {
        std::cerr << "ERROR: Stream for route-type file is't open" << std::endl;
        return false;
    }

    std::string routeExt = "";
    stream >> routeExt;

    if (routeExt.empty())
    {
        std::cerr << "ERROR: Unknown route type" << std::endl;
        return false;
    }

    std::string routeLoaderPlugin = routeExt + "-route-loader";

    osg::ref_ptr<RouteLoader> loader = loadRouteLoader(fs.getPluginsDir(), routeLoaderPlugin);

    if (!loader.valid())
    {
        std::cerr << "ERROR: Not found route loader for this route" << std::endl;
        return false;
    }

    loader->load(routeDir, settings.view_distance);

//    MotionPath *motionPath = loader->getMotionPath(settings.direction);

    train_ext_handler = new TrainExteriorHandler(settings, motionPath, settings.train_config);
    viewer.addEventHandler(train_ext_handler);


    //viewer.addEventHandler(train_ext_handler->getAnimationManager());
//    std::vector<AnimationManager *> anims_manager = train_ext_handler->getAnimManagers();

    for (auto am : anims_manager)
    {
        viewer.addEventHandler(am);
    }

    root = new vsg::Group;
    //root->addChild(train_ext_handler->getExterior());
    root->addChild(loader->getRoot());

    return true;
}
bool RouteViewer::loadTrain(const simulator_command_line_t &command_line)
{
      train_ext_handler = new TrainExteriorHandler();
      viewer.addEventHandler(train_ext_handler);
      root->addChild(anim)
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool RouteViewer::initEngineSettings(osg::Group *root)
{
    if (root == nullptr)
        return false;

    // Common graphics settings
    osg::StateSet *stateset = root->getOrCreateStateSet();

    stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    stateset->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    osg::ref_ptr<osg::CullFace> cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    stateset->setAttributeAndModes(cull.get(), osg::StateAttribute::ON);

    // Set lighting
    initEnvironmentLight(root,
                         osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f),
                         1.0f,
                         -20.0f,
                         75.0f);

    osg::LightModel *lightmodel = new osg::LightModel;
    float power = 0.4f;
    lightmodel->setAmbientIntensity(osg::Vec4(power, power, power, 1.0));
    lightmodel->setTwoSided(true);
    stateset->setAttributeAndModes(lightmodel, osg::StateAttribute::ON);

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool RouteViewer::initDisplay(osgViewer::Viewer *viewer,
                              const settings_t &settings)
{
    if (viewer == nullptr)
        return false;

    viewer->setSceneData(root.get());

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = settings.x;
    traits->y = settings.y;
    traits->width = settings.width;
    traits->height = settings.height;
    traits->windowName = settings.name;
    traits->windowDecoration = settings.window_decoration;
    traits->doubleBuffer = settings.double_buffer;
    traits->samples = settings.samples;
    traits->vsync = settings.vsync;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    osg::Camera *camera = viewer->getCamera();

    camera->setGraphicsContext(gc.get());
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));

    camera->setClearColor(osg::Vec4(0.63f, 0.80f, 0.97f, 1.0f));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    camera-> setComputeNearFarMode (osg :: CullSettings :: DO_NOT_COMPUTE_NEAR_FAR);
    double aspect = static_cast<double>(traits->width) / static_cast<double>(traits->height);
    camera->setProjectionMatrixAsPerspective(settings.fovy, aspect, settings.zNear, settings.zFar);

    camera->setAllowEventFocus(false);

    if (settings.fullscreen)
        viewer->setUpViewOnSingleScreen(settings.screen_number);

    return true;
}
