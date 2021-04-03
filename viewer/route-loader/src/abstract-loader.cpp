//------------------------------------------------------------------------------
//
//      Abstract loader or railway route
//      (c) maisvendoo, 03/12/2018
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Abstract loader or railway route
 * \copyright maisvendoo
 * \author maisvendoo
 * \date 03/12/2018
 */

#include    "abstract-loader.h"
#include    <vsg/nodes/Group.h>

#include    "library.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
RouteLoader::RouteLoader()
    : routeDir("")
    , root(new vsg::Group)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
vsg::ref_ptr<vsg::Group> RouteLoader::getRoot()
{
    if (root.valid())
        return root;
    else
        return vsg::ref_ptr<vsg::Group>();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
RouteLoader *loadRouteLoader(const std::string &path, const std::string &name)
{
    vsg::ref_ptr<RouteLoader> loader;

    Library lib(path, name);

    if (lib.load())
    {
        GetRouteLoader getRouteLoader = (GetRouteLoader) lib.resolve("getRouteLoader");

        if (getRouteLoader)
        {
            loader = getRouteLoader();
        }
    }

    return loader.release();
}
