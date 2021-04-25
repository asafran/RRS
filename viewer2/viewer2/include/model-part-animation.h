#ifndef     MODEL_PART_ANIMATION_H
#define     MODEL_PART_ANIMATION_H

#include    <vsg/nodes/MatrixTransform.h>
#include    <vsg/>



//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class ModelPartAnimation : public osg::Referenced
{
public:

    ModelPartAnimation(vsg::MatrixTransform *transform);

    void step(double t, double dt);

    void setRefPosition(double pos);

    void setLastTime(double lastTime);

private:    

    osg::MatrixTransform    *transform;
    osg::ref_ptr<osg::AnimationPath>      path;
    osg::ref_ptr<osg::AnimationPathCallback> callback;

    double pos;

    double ref_pos;

    double lastTime;

    void update();
};

#endif // MODELPARTANIMATION_H
