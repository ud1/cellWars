#include "MyStrategy.hpp"


const double MAX_ACCEL = 500.0;

double calcForce(Particle &p, Particle &oth, double targetDistance, double t, double w)
{
    P dir = relWrapP(oth.pos - p.pos, w);
    double dist2 = dir.len2();
    if (dist2 > gameParameters.maxActionDistance2)
        return 0.0;

    double dist = sqrt(dist2);
    dir /= dist;

    P dv = oth.vel - p.vel;
    double vel = dot(dir, dv);

    double td = targetDistance - dist;

    double accel = 0.0;
    if (td < 0.0 && vel > 0.0)
    {
        accel = -MAX_ACCEL;
    }
    else if (td > 0.0 && vel < 0.0)
    {
        accel = MAX_ACCEL;
    }
    else
    {
        if (td < 0.0)
        {
            double targetVel = -td < MAX_ACCEL * t ? -td : -sqrt(-2.0 * MAX_ACCEL * td);
            accel = targetVel - vel;
        }
        else if (td > 0.0)
        {
            double targetVel = td < MAX_ACCEL * t ? td : sqrt(2.0 * MAX_ACCEL * td);
            accel = targetVel - vel;
        }

        if (accel < -MAX_ACCEL)
        {
            accel = -MAX_ACCEL;
        }
        else if (accel > MAX_ACCEL)
        {
            accel = MAX_ACCEL;
        }
    }

    return accel*0.1;
}

IStrategy::IStrategy(uint16_t playerId) : playerId(playerId)
{}

MyStrategy::MyStrategy(uint16_t playerId) : IStrategy(playerId)
{}

void MyStrategy::generateActions(const World &w, Actions &actions)
{
    int soulId = -1;
    for (int id = 0; id < (int) w.particles.size(); ++id)
    {
        auto &p = w.particles[id];
        if (p.owner == playerId && p.particleType == ParticleType::SOUL)
        {
            soulId = id;
            break;
        }
    }

    double dist2 = 1e9;
    int closest = -1;

    if (soulId >= 0)
    {
        auto &soul = w.particles[soulId];

        for (int id = 0; id < (int) w.particles.size(); ++id)
        {
            auto &p = w.particles[id];
            if (p.owner == playerId && p.essence > 500)
                continue;

            if ((p.owner == 0 || p.owner == playerId) && id != soulId)
            {
                P dir = relWrapP(p.pos - soul.pos, w.width);
                double d2 = dir.len2();
                if (dist2 > d2)
                {
                   dist2 = d2;
                   closest = id;
                }
            }
        }
    }

    if (closest >= 0)
    {
        auto &soul = w.particles[soulId];
        auto &target = w.particles[closest];

        P dir = relWrapP(target.pos - soul.pos, w.width);
        double dist = dir.len();
        P normDir = dir.norm();
        P targetVel = dist > 10 ? normDir : normDir * 0.1;

        actions.thrust[soulId].acceleration = targetVel * 20.0;
        actions.transferEssence[soulId][closest].targetType = ParticleType::REGENERATOR;

        if (dist > 1.0)
            actions.applyForce[soulId][closest].force = -1.0;

        if (soul.essence > 800)
            actions.transferEssence[soulId][closest].amount = 10.0;
    }
}
