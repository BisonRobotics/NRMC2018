#include "sim_robot/sim_backhoe.h"

SimBackhoe::SimBackhoe(double shouldlerTheta, double wristTheta)
{
    shTh = shouldlerTheta;
    wrTh = wristTheta;

    sh = new SimVesc(16, 0, 1.0);
    wr = new SimVesc(16, 0, 1.0);
}

SimBackhoe::update(double dt)
{
    sh->update(dt);
    wr->update(dt);

    shTh += sh->getLinearVelocity() * dt;
    wrTh += wr->getLinearVelocity() * dt;
}
