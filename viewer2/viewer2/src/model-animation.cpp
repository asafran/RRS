#include    "model-animation.h"

#include    "model-animation-visitor.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ModelAnimation::ModelAnimation(vsg::Node *model, const QString &name)
    : ProcAnimation(name)
{
    ModelAnimationVisitor mav(&parts, name);
    mav.setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);

    model->accept(mav);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
ModelAnimation::~ModelAnimation()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ModelAnimation::anim_step(float t, float dt)
{
    for (auto part : parts)
    {
        part->setRefPosition(static_cast<double>(pos));
        part->step(static_cast<double>(t), static_cast<double>(dt));
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ModelAnimation::load_config(CfgReader &cfg)
{
    QString secName = "ModelAnimation";
    int tmp = 0;

    cfg.getInt(secName, "SignalID", tmp);

    signal_id = (tmp < 0) ? __SIZE_MAX__ : (size_t)((unsigned)tmp);

    is_fixed_signal = cfg.getFloat(secName, "FixedSignal", fixed_signal);

    double lastTime = 0.0;

    if (cfg.getDouble(secName, "Time", lastTime))
    {
        if (parts.size() > 0)
        {
            for (auto part: parts)
            {
                part->setLastTime(lastTime);
            }
        }
    }

    return true;
}
