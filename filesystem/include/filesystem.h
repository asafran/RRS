#ifndef		FILESYSTEM_H
#define		FILESYSTEM_H

#include    <QDir>

#ifdef FILESYSTEM_LIB
    #define FILESYSTEM_EXPORT   Q_DECL_EXPORT
#else
    #define FILESYSTEM_EXPORT   Q_DECL_IMPORT
#endif

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class FILESYSTEM_EXPORT FileSystem
{
public:    

    /// Get instance byt filesystem singleton
    static FileSystem &getInstance()
    {
        static FileSystem instance;

        QString workDir = QDir::currentPath();
        QString tmp = instance.getLevelUpDirectory(workDir, 1);
        instance.setBinaryDir(workDir);
        instance.setRouteRootDir(tmp + "routes");
        instance.setConfigDir(tmp + "cfg");
        instance.setLogsDir(tmp + "logs");
        instance.setLibraryDir(tmp + "lib");
        instance.setTrainsDir(instance.getConfigDir() + instance.separator() + "trains");
        instance.setModulesDir(tmp + "modules");
        instance.setVehiclesDir(instance.getConfigDir() + instance.separator() + "vehicles");
        instance.setCouplingsDir(instance.getConfigDir()+ instance.separator() + "couplings");
        instance.setDevicesDir(instance.getConfigDir()+ instance.separator() + "devices");
        instance.setDataDir(tmp + "data");
        instance.setVehicleModelsDir(instance.combinePath(instance.getDataDir(), "models"));
        instance.setVehicleTexturesDir(instance.combinePath(instance.getDataDir(), "textures"));
        instance.setPluginsDir(tmp + "plugins");
        instance.setScreenshotsDir(tmp + "screenshots");
        instance.setFontsDir(tmp + "fonts");
        instance.setSoundsDir(instance.combinePath(instance.getDataDir(), "sounds"));
        instance.setThemeDir(tmp + "themes");

        return instance;
    }    

    /// Get directory by num_levels levels up
    QString getLevelUpDirectory(QString path, int num_levels);

    QString getNativePath(const QString &path);

    /// Get route directory path
    QString getRouteRootDir() const;

    QString getConfigDir() const;

    QString getLogsDir() const;

    QString getLibraryDir() const;

    QString getTrainsDir() const;

    QString getModulesDir() const;

    QString getVehiclesDir() const;

    QString getCouplingsDir() const;

    QString getDevicesDir() const;

    QString getBinaryDir() const;

    QString getPluginsDir() const;

    QString getDataDir() const;

    QString getVehicleModelsDir() const;

    QString getVehicleTexturesDir() const;

    QString getScreenshotsDir() const;

    QString getFontsDir() const;

    QString getSoundsDir() const;

    QString getThemeDir() const;

    QString combinePath(const QString &path1, const QString &path2);

    QString toNativeSeparators(const QString &path);

    /// Get native path separator
    char separator() const;

private:

    QString routeRootDir;
    QString configDir;
    QString logsDir;
    QString libraryDir;
    QString trainsDir;
    QString modulesDir;
    QString vehiclesDir;
    QString couplingsDir;
    QString devicesDir;
    QString binDir;
    QString pluginsDir;

    QString dataDir;
    QString vehicleModelsDir;
    QString vehicleTexturesDir;

    QString screenshotsDir;
    QString fontsDir;

    QString soundsDir;

    QString themeDir;

    FileSystem() {}
    FileSystem(const FileSystem &) = delete;
    FileSystem &operator=(FileSystem &) = delete;

    /// Set route direcory path in paltform native format
    void setRouteRootDir(const QString &path);

    /// Set config directory path
    void setConfigDir(const QString &path);

    void setLogsDir(const QString &path);

    void setLibraryDir(const QString &path);

    void setTrainsDir(const QString &path);

    void setModulesDir(const QString &path);

    void setVehiclesDir(const QString &path);

    void setCouplingsDir(const QString &path);

    void setDevicesDir(const QString &path);

    void setBinaryDir(const QString &path);

    void setPluginsDir(const QString &path);

    void setDataDir(const QString &path);

    void setVehicleModelsDir(const QString &path);

    void setVehicleTexturesDir(const QString &path);

    void setScreenshotsDir(const QString &path);

    void setFontsDir(const QString &path);

    void setSoundsDir(const QString &path);

    void setThemeDir(const QString &path);
};

#endif
