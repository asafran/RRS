#ifndef     MATERIAL_ANIMATION_H
#define     MATERIAL_ANIMATION_H

#include    "proc-animation.h"

#include    <vsg/state/material.h>
#include    <vsg/StateSet>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class MaterialAnimation : public ProcAnimation
{
public:

    MaterialAnimation(osg::Material *mat, osg::Drawable *drawable);

    ~MaterialAnimation();

private:


    vsg::material   *mat;
    osg::Drawable   *drawable;
    osg::ref_ptr<osg::StateSet>   stateset;    

    float           cur_pos;

    osg::Vec4       color;
    osg::Vec4       emission_color;

    void anim_step(float t, float dt);

    bool load_config(ConfigReader &cfg);

    void update();
};

#endif // MATERIAL_ANIMATION_H
