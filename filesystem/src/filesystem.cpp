#include    "filesystem.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FileSystem::setRouteRootDir(const QString &path)
{
    routeRootDir = getNativePath(path);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FileSystem::setConfigDir(const QString &path)
{
    configDir = getNativePath(path);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FileSystem::setLogsDir(const QString &path)
{
    logsDir = getNativePath(path);
}

void FileSystem::setLibraryDir(const QString &path)
{
    libraryDir = getNativePath(path);
}

void FileSystem::setTrainsDir(const QString &path)
{
    trainsDir = getNativePath(path);
}

void FileSystem::setModulesDir(const QString &path)
{
    modulesDir = getNativePath(path);
}

void FileSystem::setVehiclesDir(const QString &path)
{
    vehiclesDir = getNativePath(path);
}

void FileSystem::setCouplingsDir(const QString &path)
{
    couplingsDir = getNativePath(path);
}

void FileSystem::setDevicesDir(const QString &path)
{
    devicesDir = getNativePath(path);
}

void FileSystem::setBinaryDir(const QString &path)
{
    binDir = getNativePath(path);
}

void FileSystem::setPluginsDir(const QString &path)
{
    pluginsDir = getNativePath(path);
}

void FileSystem::setDataDir(const QString &path)
{
    dataDir = getNativePath(path);
}

void FileSystem::setVehicleModelsDir(const QString &path)
{
    vehicleModelsDir = getNativePath(path);
}

void FileSystem::setVehicleTexturesDir(const QString &path)
{
    vehicleTexturesDir = getNativePath(path);
}

void FileSystem::setScreenshotsDir(const QString &path)
{
    screenshotsDir = getNativePath(path);
}

void FileSystem::setFontsDir(const QString &path)
{
    fontsDir = getNativePath(path);
}

void FileSystem::setSoundsDir(const QString &path)
{
    soundsDir = getNativePath(path);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void FileSystem::setThemeDir(const QString &path)
{
    themeDir = getNativePath(path);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getRouteRootDir() const
{
    return routeRootDir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getConfigDir() const
{
    return configDir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getLogsDir() const
{
    return logsDir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getLibraryDir() const
{
    return libraryDir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getTrainsDir() const
{
    return trainsDir;
}

QString FileSystem::getModulesDir() const
{
    return modulesDir;
}

QString FileSystem::getVehiclesDir() const
{
    return vehiclesDir;
}

QString FileSystem::getCouplingsDir() const
{
    return couplingsDir;
}

QString FileSystem::getDevicesDir() const
{
    return devicesDir;
}

QString FileSystem::getBinaryDir() const
{
    return binDir;
}

QString FileSystem::getPluginsDir() const
{
    return  pluginsDir;
}

QString FileSystem::getDataDir() const
{
    return dataDir;
}

QString FileSystem::getVehicleModelsDir() const
{
    return vehicleModelsDir;
}

QString FileSystem::getVehicleTexturesDir() const
{
    return vehicleTexturesDir;
}

QString FileSystem::getScreenshotsDir() const
{
    return screenshotsDir;
}

QString FileSystem::getFontsDir() const
{
    return fontsDir;
}

QString FileSystem::getSoundsDir() const
{
    return soundsDir;
}

QString FileSystem::getThemeDir() const
{
    return themeDir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::combinePath(const QString &path1, const QString &path2)
{
    if (*(path1.end() - 1) != separator())
        return getNativePath(path1 + separator() + path2);
    else
        return getNativePath(path1 + path2);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::toNativeSeparators(const QString &path)
{
    QString tmp = path;

#if __unix__
    std::replace(tmp.begin(), tmp.end(), '\\', '/');
#else
    std::replace(tmp.begin(), tmp.end(), '/', '\\');
#endif

    return tmp;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getNativePath(const QString &path)
{
    return QDir::toNativeSeparators(path);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
char FileSystem::separator() const
{
    return QDir::separator().toLatin1();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString FileSystem::getLevelUpDirectory(QString path, int num_levels)
{
    QDir dir(path);

    for (int i = 0; i < num_levels; ++i)
        dir.cdUp();

    QString tmp = dir.path() + QDir::separator();

    return tmp;
}
