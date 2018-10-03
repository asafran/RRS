#include    "ReaderWriterDMD.h"

#include    <osg/Array>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ReaderWriterDMD::ReaderWriterDMD()
{
    supportsExtension("dmd", "DGLEngine model format");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ReaderWriterDMD::~ReaderWriterDMD()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
const char *ReaderWriterDMD::className() const
{
    return "DGLEngine DMD-models Reader/Writer";
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osgDB::ReaderWriter::ReadResult ReaderWriterDMD::readNode(const std::string &file,
                                                         const osgDB::ReaderWriter::Options *options) const
{
    std::string ext = osgDB::getLowerCaseFileExtension(file);

    if (!acceptsExtension(ext))
        return ReadResult::FILE_NOT_HANDLED;

    std::string fileName = osgDB::findDataFile(file, options);

    if (fileName.empty())
        return ReadResult::FILE_NOT_FOUND;

    osgDB::ifstream fin(fileName.c_str(), std::ios_base::in);

    if (!fin.good())
        return ReadResult::ERROR_IN_READING_FILE;

    return doReadNode(fin, options, fileName);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osgDB::ReaderWriter::ReadResult ReaderWriterDMD::readNode(std::ifstream &fin,
                                                         const osgDB::ReaderWriter::Options *options) const
{
    return doReadNode(fin, options, "");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osgDB::ReaderWriter::ReadResult ReaderWriterDMD::doReadNode(std::ifstream &fin,
                                                           const osgDB::ReaderWriter::Options *options,
                                                           const std::string &fileName) const
{
    (void) fileName;
    (void) options;

    DMDObject dmdObj;

    if (dmdObj.load(fin))
    {
        osg::Node *node = convertModelToSceneGraph(dmdObj, options);
        return node;
    }

    return ReadResult::FILE_NOT_HANDLED;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osgDB::ReaderWriter::WriteResult ReaderWriterDMD::writeNode(const osg::Node &node,
                                                           const std::string &file,
                                                           const osgDB::ReaderWriter::Options *options) const
{
    (void) node;
    (void) file;
    (void) options;

    return WriteResult::FILE_NOT_HANDLED;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osgDB::ReaderWriter::WriteResult ReaderWriterDMD::writeNode(const osg::Node &node,
                                                           const std::ofstream &fout,
                                                           const osgDB::ReaderWriter::Options *options) const
{
    (void) node;
    (void) fout;
    (void) options;

    return WriteResult::FILE_NOT_HANDLED;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osgDB::ReaderWriter::WriteResult ReaderWriterDMD::doWriteNode(const osg::Node &node,
                                                             const std::ofstream &fout,
                                                             const osgDB::ReaderWriter::Options *options,
                                                             const std::string &fileName) const
{
    (void) node;
    (void) fileName;
    (void) fout;
    (void) options;

    return WriteResult::FILE_NOT_HANDLED;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osg::Node *ReaderWriterDMD::convertModelToSceneGraph(DMDObject &dmdObj,
                                                     const osgDB::ReaderWriter::Options *options) const
{
    dmd_multymesh_t multyMesh = dmdObj.getMultyMesh();

    if (multyMesh.meshes.empty())
        return nullptr;

    osg::Group  *group = new osg::Group;

    for (auto it = multyMesh.meshes.begin(); it != multyMesh.meshes.end(); ++it)
    {
        osg::Geometry *geometry = convertMeshToGeometry(*it, multyMesh, options);

        osg::Vec2Array *texvertices = new osg::Vec2Array;

        for (size_t i = 0; i < multyMesh.texture_vertices->size(); i++)
        {
            osg::Vec3f tex_ver = multyMesh.texture_vertices->at(i);
            texvertices->push_back(osg::Vec2(tex_ver.x(), tex_ver.y()));
        }

        geometry->setTexCoordArray(0, texvertices);

        osg::Geode *geode = new osg::Geode;
        geode->addDrawable(geometry);

        group->addChild(geode);
    }

    return group;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osg::Geometry *ReaderWriterDMD::convertMeshToGeometry(dmd_mesh_t *mesh,
                                                      dmd_multymesh_t &multyMesh,
                                                      const osgDB::ReaderWriter::Options *options) const
{
    osg::Geometry *geometry = new osg::Geometry;


    osg::Vec3Array *vertexArray = new osg::Vec3Array;
    osg::Vec3Array *normalArray = new osg::Vec3Array;

    vertexArray->resize(multyMesh.tex_v_count);
    normalArray->resize(multyMesh.tex_v_count);

    for (size_t i = 0; i < mesh->faces.size(); i++)
    {
        face_t face = mesh->faces[i];
        face_t tex_face = multyMesh.texture_faces[i];

        for (size_t j = 0; j < face.indices.size(); j++)
        {
            size_t v_idx = face.indices[j];
            size_t tv_idx = tex_face.indices[j];

            vertexArray->at(tv_idx).set(mesh->vertices->at(v_idx));
            normalArray->at(tv_idx).set(mesh->smooth_normals->at(v_idx));
        }
    }

    geometry->setVertexArray(vertexArray);
    geometry->setNormalArray(normalArray, osg::Array::BIND_PER_VERTEX);

    std::vector<osg::DrawElementsUInt *> meshPrimitiveSets;

    for (size_t i = 0; i < mesh->faces.size(); i++)
    {
        osg::DrawElementsUInt *primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
        meshPrimitiveSets.push_back(primitive);

        osg::DrawElementsUInt *face = meshPrimitiveSets.back();

        for (size_t j = 0; j < mesh->faces[i].indices.size(); j++)
            face->push_back(multyMesh.texture_faces[i].indices[j]);

        geometry->addPrimitiveSet(face);
    }

    return geometry;
}


REGISTER_OSGPLUGIN(dmd, ReaderWriterDMD)
