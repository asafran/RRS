#include    "proc-animation.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ProcAnimation::ProcAnimation(vsg::MatrixTransform *transform)
    : pos(0.0f)
    , duration(0.0f)
    , signal_id(0)
    , transform(transform)
    , name("")
    , is_fixed_signal(false)
    , fixed_signal(0.0f)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ProcAnimation::~ProcAnimation()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ProcAnimation::step(float t, float dt)
{
    anim_step(t, dt);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ProcAnimation::setName(const QString &name)
{
    this->name = name;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString ProcAnimation::getName() const
{
    return name;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ProcAnimation::load(const QString &path)
{
    CfgReader cfg;

    if (!cfg.load(path))
    {
        Journal::instance()->warning("Animation file " + path + " is't found");
        return false;
    }

    return load_config(cfg);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ProcAnimation::load(CfgReader &cfg)
{
    loadKeyPoints(cfg);

    return load_config(cfg);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ProcAnimation::setPosition(float pos)
{
    if (is_fixed_signal)
        this->pos = fixed_signal;
    else
        this->pos = pos;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
size_t ProcAnimation::getSignalID() const
{
    return signal_id;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ProcAnimation::loadKeyPoints(CfgReader &cfg)
{
    QDomElement config_node = cfg.getFirstSection("KeyPoint");

    if (config_node.isNull())
    {
        Journal::instance()->warning("There is no Config node in file");
        return false;
    }

     while (!config_node.isNull())
    {
         key_point_t key_point;

         if(!cfg.getFloat(config_node, "Param", key_point.param))
         {
             Journal::instance()->error("Param of keypoint is missing");
             continue;
         }
         if(!cfg.getFloat(config_node, "Value", key_point.value))
         {
             Journal::instance()->error("Value of keypoint is missing");
             continue;
         }

         keypoints.push_back(key_point);

         config_node = config_node.nextSiblingElement("KeyPoint");
    }

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float ProcAnimation::interpolate(float param)
{
    if (keypoints.size() <= 1)
        return 0.0f;

    size_t next_idx = 0;
    key_point_t begin_point = findBeginKeyPoint(param, next_idx);
    key_point_t next_point = keypoints.at(next_idx);

    float range = next_point.param - begin_point.param;

    if (range < 1e-6f)
        return 0.0f;

    float value = begin_point.value + (param - begin_point.param) * (next_point.value - begin_point.value) / range;

    return value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ProcAnimation::key_point_t ProcAnimation::findBeginKeyPoint(float param, size_t &next_idx)
{
    key_point_t key_point;

    if (keypoints.size() == 0)
        return key_point;

    size_t left_idx = 0;
    size_t right_idx = keypoints.size() - 1;
    size_t idx = (left_idx + right_idx) / 2;

    while (idx != left_idx)
    {
        key_point = keypoints.at(idx);

        if (param <= key_point.param)
            right_idx = idx;
        else
            left_idx = idx;

        idx = (left_idx + right_idx) / 2;
    }

    key_point = keypoints.at(idx);
    next_idx = idx + 1;

    return key_point;
}
