#ifndef     ANIM_TRANSFORM_VISITOR_H
#define     ANIM_TRANSFORM_VISITOR_H

#include    <vsg/core/Visitor.h>
#include    "animations-list.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class AnimTransformVisitor : public vsg::Visitor
{
public:

    AnimTransformVisitor(animations_t *animations, const QString &vehicle_config);

    virtual void apply(vsg::MatrixTransform &transform);

private:

    animations_t *animations;
    QString vehicle_config;

    ProcAnimation *create_animation(const QString &name, vsg::MatrixTransform *transform);
};

#endif // ANIM_TRANSFORM_VISITOR_H
