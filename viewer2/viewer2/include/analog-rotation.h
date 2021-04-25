#ifndef     ANALOG_ROTATION_H
#define     ANALOG_ROTATION_H

#include    "proc-animation.h"

class AnalogRotation : public ProcAnimation
{
public:

    AnalogRotation(vsg::MatrixTransform *transform);

    ~AnalogRotation();

private:

    float       min_angle;
    float       max_angle;
    float       angle;

    float       cur_pos;

    bool        infinity;

    vsg::vec3   axis;
    vsg::dmat4 matrix;

    void anim_step(float t, float dt);

    bool load_config(CfgReader &cfg);

    void update();
};

#endif // ANALOG_ROTATION_H
