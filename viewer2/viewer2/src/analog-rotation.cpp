#include    "analog-rotation.h"
#include    <vsg/maths/transform.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AnalogRotation::AnalogRotation(vsg::MatrixTransform *transform) : ProcAnimation (transform)
    , min_angle(0.0f)
    , max_angle(vsg::PIf)
    , angle(0.0f)
    , cur_pos(0.0f)
    , infinity(false)
    , axis(vsg::vec3(0.0, 0.0, 1.0))
    , matrix(transform->getMatrix())
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AnalogRotation::~AnalogRotation()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AnalogRotation::anim_step(float t, float dt)
{
    (void) t;

    cur_pos += (pos - cur_pos) * duration * dt;

    angle = interpolate(cur_pos);

    update();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool AnalogRotation::load_config(CfgReader &cfg)
{
    QString secName = "AnalogRotation";

    int tmp = 0;

    cfg.getInt(secName, "SignalID", tmp);

    signal_id = (tmp < 0) ? __SIZE_MAX__ : (size_t)((unsigned)tmp);

    is_fixed_signal = cfg.getFloat(secName, "FixedSignal", fixed_signal);
    cfg.getFloat(secName, "Duration", duration);

    cfg.getBool(secName, "Infinity", infinity);

    QString tmp_str;
    cfg.getString(secName, "Axis", tmp_str);

    std::istringstream ss(tmp_str.toStdString());

    ss >> axis.x >> axis.y >> axis.z;

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AnalogRotation::update()
{
    if (keypoints.size() == 0)
        return;

    if (!infinity)
        angle = cut(angle, (*keypoints.begin()).value, (*(keypoints.end() - 1)).value);

    vsg::dmat4 rotate = vsg::rotate(angle * vsg::PIf / 180.0f, axis);
    transform->setMatrix(rotate * matrix);
}
