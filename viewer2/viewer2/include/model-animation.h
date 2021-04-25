#ifndef     MODEL_ANIMATION_H
#define     MODEL_ANIMATION_H

#include    "proc-animation.h"
#include    "model-parts-list.h"
#include    "model-part-animation.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class ModelAnimation : public ProcAnimation
{
public:

    ModelAnimation(vsg::Node *model, const QString &name);

    ~ModelAnimation();

private:

    model_parts_list_t  parts;

    void anim_step(float t, float dt);

    bool load_config(CfgReader &cfg);
};

#endif // MODELANIMATION_H
