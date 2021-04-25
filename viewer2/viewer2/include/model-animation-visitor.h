#ifndef     MODEL_ANIMATION_VISITOR_H
#define     MODEL_ANIMATION_VISITOR_H

#include    <vsg/core/Visitor.h>
#include    <vsg/core/Inherit.h>
#include    "model-parts-list.h"

class ModelAnimationVisitor : public Inherit<vsg::Visitor, ModelAnimationVisitor>
{
public:

    ModelAnimationVisitor(model_parts_list_t *parts, const std::string &anim_name);

    virtual void apply(osg::Transform &transform);

private:

    model_parts_list_t  *parts;
    std::string         anim_name;
};


#endif // MODEL_ANIMATION_VISITOR_H
