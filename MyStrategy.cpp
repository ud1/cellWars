#include "MyStrategy.hpp"
#include <memory>

const double MAX_ACCEL = 500.0;

double calcForce(const Particle &p, const Particle &oth, double targetDistance, double t, double w)
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

struct GridSquare {
    std::vector<uint16_t> ids;
};

struct ProximityGrid {
    std::vector<GridSquare> squares;
    int size;

    ProximityGrid(int wid)
    {
        size = (wid + 4) / 3.5;
        squares.resize(size * size);
    }

    void add(const Particle &p, uint16_t id)
    {
        IP ip = IP(p.pos / P(3.5, 3.5));
        GridSquare &s = squares[ip.y * size + ip.x];
        s.ids.push_back(id);
    }

    GridSquare& get(IP p)
    {
        p.x = (p.x + size) % size;
        p.y = (p.y + size) % size;
        return squares[p.y * size + p.x];
    }
};

struct IndexOrder {
    std::vector<IP> inds;

    IndexOrder(int w)
    {
        int w05 = w/2;
        for (int y = -w05; y <= w05; ++y)
        {
            for (int x = -w05; x <= w05; ++x)
            {
                inds.push_back(IP(x, y));
            }
        }

        std::sort(inds.begin(), inds.end(), [](auto &a, auto &b){
            return a.toP().len2() < b.toP().len2();
        });
    }
};

static std::unique_ptr<IndexOrder> indexOrder;

void MyStrategy::generateActions(const World &w, Actions &actions)
{
    std::unordered_map<ParticleType, std::vector<uint16_t>> myParticles;

    ProximityGrid grid = ProximityGrid(w.width);
    if (!indexOrder)
        indexOrder = std::make_unique<IndexOrder>(grid.size);

    for (int id = 0; id < (int) w.particles.size(); ++id)
    {
        auto &p = w.particles[id];
        grid.add(p, id);

        if (p.owner == playerId)
        {
            myParticles[p.particleType].push_back(id);
        }
    }

    int closest = -1;
    if (!myParticles[ParticleType::SOUL].empty())
    {
        uint16_t soulId = myParticles[ParticleType::SOUL][0];
        auto &soul = w.particles[soulId];

        IP soulIp = IP(soul.pos / P(3.5, 3.5));
        for (auto &ip : indexOrder->inds)
        {
            GridSquare &sq = grid.get(ip + soulIp);

            for (auto id : sq.ids)
            {
                auto &p = w.particles[id];
                if (p.owner == playerId && p.essence > 500)
                    continue;

                if ((p.owner == 0 || p.owner == playerId) && id != soulId)
                {
                    closest = id;
                    break;
                }
            }

            if (closest >= 0)
                break;
        }
    }

    if (closest >= 0)
    {
        uint16_t soulId = myParticles[ParticleType::SOUL][0];

        auto &soul = w.particles[soulId];
        auto &target = w.particles[closest];

        P dir = relWrapP(target.pos - soul.pos, w.width);
        double dist = dir.len();
        P normDir = dir.norm();
        P targetVel = (dist > 10 ? normDir : normDir * 0.2) + target.vel;

        actions.thrust[soulId].acceleration = targetVel - soul.vel;

        if (dist > 1.0)
            actions.applyForce[soulId][closest].force = -1.0;

        if (soul.essence > 800)
        {
            actions.transferEssence[soulId][closest].amount = 10.0;
            actions.transferEssence[soulId][closest].targetType = ParticleType::REGENERATOR;
        }
    }

    for (auto &pr : myParticles)
    {
        if (pr.first == ParticleType::SOUL)
            continue;

        for (uint16_t id : pr.second)
        {
            auto &p = w.particles[id];
            IP ip = IP(p.pos / P(3.5, 3.5));

            GridSquare &sq = grid.get(ip);

            for (auto othId : sq.ids)
            {
                if (id != othId)
                {
                    auto &oth = w.particles[othId];
                    double dist2 = oth.pos.dist2(p.pos);
                    if (dist2 < sqr(3.5))
                    {
                        actions.applyForce[id][othId].force = calcForce(p, oth, 1.5, 0.016, w.width);

                        if ((p.essence > 600 || p.essence > 300 && (p.essence + oth.essence) > 600) && (oth.owner != playerId || oth.essence < 800))
                        {
                            actions.transferEssence[id][othId].amount = 10.0;
                            actions.transferEssence[id][othId].targetType = ParticleType::REGENERATOR;
                        }
                    }
                }
            }
        }
    }
}
