#ifndef     MATERIAL_ANIMATION_VISITOR_H
#define     MATERIAL_ANIMATION_VISITOR_H

#include    <vsg/core/Visitor.h>
#include    "animations-list.h"

class MaterialAnimationVisitor : public vsg::Visitor
{
public:

    MaterialAnimationVisitor(animations_t *animations, CfgReader *cfg);

    virtual void apply(vsg::Geometry &node);

private:

    animations_t *animations;
    CfgReader *cfg;
};

#endif // MATERIAL_ANIMATION_VISITOR_H
