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

#ifndef     VIEWER_H
#define     VIEWER_H

#include    <vsg/viewer/Viewer.h>

#include    "settings.h"
#include    "command-line-parser.h"
#include    "client.h"

#include    "keyboard.h"

#include    "train-exterior.h"
#include    "simulator-command-line.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class RouteViewer
{
public:

    /// Constructor
    RouteViewer(int argc, char *argv[]);

    /// Destructor
    virtual ~RouteViewer();

    /// Get ready state
    bool isReady() const;

    /// Start client
    int run();

protected:

    /// Viewer ready flag
    bool                        is_ready;

    KeyboardHandler             *keyboard;

    /// Viewer settings
    settings_t                  settings;

    /// VSG viewer object
    vsg::Viewer           viewer;

    /// OSG scene root node
    vsg::ref_ptr<vsg::Group>    root;



    TrainExteriorHandler *train_ext_handler;

    /// Initialization
    bool init(int argc, char *argv[]);   

    /// Load settings from config
    settings_t loadSettings(const QString &cfg_path) const;

    void applySettings(const settings_t &settings, vsg::ref_ptr<vsg::WindowTraits> windowTraits);

    /// Override settings from command line
//    void overrideSettingsByCommandLine(const cmd_line_t &cmd_line,
//                                       settings_t &settings);

    /// Load route form directory
    bool loadRoute(const std::string &routeDir);

    bool loadTrain(const simulator_command_line_t &command_line);

    /// Init common graphical engine settings
    bool initEngineSettings(osg::Group *root);

    /// Init display
    bool initDisplay(osgViewer::Viewer *viewer, const settings_t &settings);    
};

#endif // VIEWER_H
