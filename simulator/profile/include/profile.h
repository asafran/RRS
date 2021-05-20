//------------------------------------------------------------------------------
//
//
//
//
//------------------------------------------------------------------------------
#ifndef     PROFILE_H
#define     PROFILE_H

#include    <QString>
#include    <fstream>
#include    <QVector>

#include    "profile-element.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Profile
{
public:

    Profile()
        : is_ready(false)
        , dir(1)
    {

    }

    Profile(int dir, const QString &routeDir);

    ~Profile();

    bool isReady() const;

    profile_element_t getElement(double railway_coord);

private:

    bool    is_ready;

    int     dir;

    QVector<profile_element_t> profile_data;

    bool load(const QString &path);

    bool load(std::ifstream &stream);
};

#endif // PROFILE_H
