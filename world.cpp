#include "world.hpp"
#include <iostream>
#include <unordered_set>
#include "explosion.hpp"

double globalT = 0.0;

enum class CollisionStatus {
    None, Intersecting, Colliding
};

struct CollisionInfo {
    CollisionStatus status;
    P wrap;
    double t;
};

CollisionInfo getCollisionTime(const Particle &p1, const Particle &p2, P wrapP) {
    P dp = p2.pos - p1.pos + wrapP;
    double startT;

    if (p1._currentT > p2._currentT)
    {
        dp += p2.vel * (p1._currentT - p2._currentT);
        startT = p1._currentT;
    }
    else if (p1._currentT < p2._currentT)
    {
        dp -= p1.vel * (p2._currentT - p1._currentT);
        startT = p2._currentT;
    }
    else
    {
        startT = p1._currentT;
    }

    double dd = dp.len2() - 1.0;
    if (dd < 0) {
        if (dd > -1e-6)
        {
            P dv = p2.vel - p1.vel;
            double dpdv = dot(dp, dv);
            if (dpdv >= 0) {
                return CollisionInfo{CollisionStatus::None, wrapP, std::numeric_limits<double>::infinity()};
            }
        }
        return CollisionInfo{CollisionStatus::Intersecting, wrapP, startT};
    }

    P dv = p2.vel - p1.vel;
    double dpdv = dot(dp, dv);
    if (dpdv >= -1.0e-9) {
        return CollisionInfo{CollisionStatus::None, wrapP, std::numeric_limits<double>::infinity()};
    }
    double dv2 = dv.len2();
    double Do4 = sqr(dpdv) - dv2*dd;

    if (Do4 > 0)
    {
        double sqrtDo2 = sqrt(Do4);
        return CollisionInfo{CollisionStatus::Colliding, wrapP, startT + (-dpdv - sqrtDo2) / dv2};
    }

    return CollisionInfo{CollisionStatus::None, wrapP, std::numeric_limits<double>::infinity()};
}

CollisionInfo getCollisionTimeWrapped(Particle &p1, Particle &p2, double w) {
    CollisionInfo result = CollisionInfo{CollisionStatus::None, P{0.0, 0.0}, std::numeric_limits<double>::infinity()};

    for (double y : {-w, 0.0, w})
    {
        for (double x : {-w, 0.0, w})
        {
            CollisionInfo c = getCollisionTime(p1, p2, P(x, y));
            if (c.t < result.t)
                result = c;
        }
    }

    return result;
}


struct CollidingParticles {
    double t;
    uint32_t updateId1, updateId2;
    P collisionWrap;
    uint16_t id1, id2;
    CollisionStatus status;

    CollidingParticles(double t, uint32_t updateId1, uint32_t updateId2, const P &collisionWrap, uint16_t id1, uint16_t id2, CollisionStatus status) : t(t), updateId1(updateId1), updateId2(updateId2),
                                                                                                                                                       collisionWrap(collisionWrap), id1(id1), id2(id2),
                                                                                                                                                       status(status)
    {}

    bool operator<(const CollidingParticles &rhs) const
    {
        return t > rhs.t;
    }
};

struct ParticleSet {
    std::vector<uint16_t> ids;

    void clear() {
        ids.clear();
    }

    bool add(uint16_t id) {
        if (std::find(ids.begin(), ids.end(), id) == ids.end())
        {
            ids.push_back(id);
            return true;
        }

        return false;
    }
};

void generateCollisions(Grid &grid, World &w, Particle &p, uint16_t id, P nextPos,
                        double maxT,
                        std::priority_queue<CollidingParticles> &collidingParticlesQueue)
{
    static ParticleSet particleSet;
    particleSet.clear();

    auto visit = [&](IP ip){
        uint32_t x = wrap(ip.x, grid.w);
        uint32_t y = wrap(ip.y, grid.w);
        grid.iterateCell(x, y, [&](uint16_t oth_id) {
            if (id != oth_id && particleSet.add(oth_id))
            {
                Particle &oth = w.particles[oth_id];
                CollisionInfo ci = getCollisionTimeWrapped(p, oth, grid.w);

                if (ci.status == CollisionStatus::None || ci.t > maxT)
                    return;

                collidingParticlesQueue.emplace(ci.t, p._updateId, oth._updateId, ci.wrap, id, oth_id, ci.status);
            }
        });

        return true;
    };

    grid.raytraceSquare(p.pos, nextPos, visit);
}

void updateVelocities(Particle &p1, Particle &p2, P wrapP)
{
    P norm = p2.pos - p1.pos + wrapP;
    norm = norm.norm();
    P tan = norm.conj();
    P avgVel = (p1.vel + p2.vel) * 0.5;

    P relV1 = p1.vel - avgVel;
    P oldVel1 = p1.vel;
    p1.vel = tan * dot(relV1, tan) - norm * dot(relV1, norm) + avgVel;
    p1._totalImpulseChange2 += (oldVel1 - p1.vel).len2();

    P oldVel2 = p2.vel;
    P relV2 = p2.vel - avgVel;
    p2.vel = tan * dot(relV2, tan) - norm * dot(relV2, norm) + avgVel;
    p2._totalImpulseChange2 += (oldVel2 - p2.vel).len2();
}

GameParameters gameParameters;

Particle *getNearbyParticle(World &w, Particle &p, uint16_t othId, double wid)
{
    if (othId >= w.particles.size())
        return nullptr;

    Particle &oth = w.particles[othId];
    if (&oth == &p)
        return nullptr;

    P dir = relWrapP(oth.pos - p.pos, wid);
    double dist2 = dir.len2();
    if (dist2 > gameParameters.maxActionDistance2)
        return nullptr;

    return &p;
}

void applyForcesAndThrust(World &w, double t, std::unordered_map<uint16_t, Actions> &actionsByPlayerId, double wid) {
    for (auto &pl : actionsByPlayerId)
    {
        uint16_t playerId = pl.first;
        Actions& actions = pl.second;

        for (auto &pa: actions.applyForce)
        {
            if (pa.first >= w.particles.size())
                continue;

            Particle &p = w.particles[pa.first];

            if (p.owner != playerId)
                continue;

            for (auto &f: pa.second)
            {
                if (f.first >= w.particles.size() || f.first == pa.first)
                    continue;

                Particle &oth = w.particles[f.first];

                P dir = relWrapP(oth.pos - p.pos, wid);
                double dist2 = dir.len2();
                if (dist2 > gameParameters.maxActionDistance2)
                    continue;

                double dist = sqrt(dist2);
                dir /= dist;

                ParticleParameters &thisParams = gameParameters.particleParameters[p.particleType];

                double accel = f.second.force;
                double maxForce = thisParams.maxForce * p.essence / thisParams.maxEssenceCapacity;
                if (accel > maxForce)
                    accel = maxForce;
                else if (accel < -maxForce)
                    accel = -maxForce;

                p._accel -= dir * accel * 0.5;
                oth._accel += dir * accel * 0.5;
            }
        }

        for (auto &pa: actions.thrust)
        {
            if (pa.first >= w.particles.size())
                continue;

            Particle &p = w.particles[pa.first];

            if (p.owner != playerId)
                continue;

            if (p.particleType == ParticleType::MOVER || p.particleType == ParticleType::SOUL)
            {
                double len2 = pa.second.acceleration.len2();
                ParticleParameters &thisParams = gameParameters.particleParameters[p.particleType];

                double maxThrust = thisParams.maxThrust * p.essence / thisParams.maxEssenceCapacity;
                if (len2 > sqr(maxThrust))
                {
                    p._accel += pa.second.acceleration.norm() * maxThrust;
                }
                else
                {
                    p._accel += pa.second.acceleration;
                }
            }
        }
    }
}

struct PlayerParticleTypeKey {
    uint16_t playerId;
    ParticleType type;

    PlayerParticleTypeKey(uint16_t playerId, ParticleType type) : playerId(playerId), type(type)
    {}

    bool operator==(const PlayerParticleTypeKey &rhs) const
    {
        return playerId == rhs.playerId &&
               type == rhs.type;
    }
};

template <>
struct std::hash<PlayerParticleTypeKey>
{
    std::size_t operator()(const PlayerParticleTypeKey& k) const
    {
        return k.playerId * ((size_t) ParticleType::COUNT) + (size_t) k.type;
    }
};

void applyEssenceTransfer(World &w, double t, std::unordered_map<uint16_t, Actions> &actionsByPlayerId, double wid) {
    std::unordered_map<uint16_t/*particleId*/, std::unordered_map<PlayerParticleTypeKey, double>> essenceTransfer;

    for (auto &pl : actionsByPlayerId)
    {
        uint16_t playerId = pl.first;
        Actions& actions = pl.second;

        for (auto &pa: actions.transferEssence)
        {
            if (pa.first >= w.particles.size())
                continue;

            Particle &p = w.particles[pa.first];

            if (p.owner != playerId)
                continue;

            ParticleParameters &thisParams = gameParameters.particleParameters[p.particleType];

            double totalEssenceTransfer = 0.0;
            for (auto &f: pa.second)
            {
                if (f.second.amount <= 0.0 || f.second.targetType == ParticleType::NEUTRAL)
                    continue;

                Particle *oth = getNearbyParticle(w, p, f.first, wid);
                if (oth)
                {
                    double maxOutcome = thisParams.maxEssenceOutcomePerSecond * t;
                    if (f.second.amount > maxOutcome)
                        f.second.amount = maxOutcome;

                    ParticleParameters &othParams = gameParameters.particleParameters[oth->particleType == ParticleType::NEUTRAL ? f.second.targetType : oth->particleType];
                    double maxIncome = othParams.maxEssenceIncomePerSecond * t;
                    if (f.second.amount > maxIncome)
                        f.second.amount = maxIncome;

                    totalEssenceTransfer += f.second.amount;
                }
            }

            if (totalEssenceTransfer >= 0.0)
            {
                double coef = p.essence >= totalEssenceTransfer ? 1.0 : p.essence / totalEssenceTransfer;

                for (auto &f: pa.second)
                {
                    if (f.second.amount <= 0.0)
                        continue;

                    Particle *oth = getNearbyParticle(w, p, f.first, wid);
                    if (oth)
                    {
                        double amount = f.second.amount * coef;
                        essenceTransfer[pa.first][PlayerParticleTypeKey{playerId, p.particleType}] -= amount;
                        essenceTransfer[f.first][PlayerParticleTypeKey{playerId, f.second.targetType}] += amount;
                    }
                }
            }
        }
    }

    for (auto &v : essenceTransfer)
    {
        Particle &p = w.particles[v.first];

        if (p.owner)
        {
            p.essence += v.second[PlayerParticleTypeKey{p.owner, p.particleType}];
            v.second.erase(PlayerParticleTypeKey{p.owner, p.particleType});

            while (p.essence > 0.0 && !v.second.empty())
            {
                auto minIt = std::min_element(v.second.begin(), v.second.end(), [](const auto &v1, const auto &v2){
                    return v1.second < v2.second;
                });

                double minV = minIt->second;
                if (v.second.size() * minV <= p.essence)
                {
                    p.essence -= v.second.size() * minV;
                    v.second.erase(minIt);

                    for (auto &et : v.second)
                    {
                        et.second -= minV;
                    }
                }
                else
                {
                    minV = p.essence / v.second.size();
                    for (auto &et : v.second)
                    {
                        et.second -= minV;
                    }
                }
            }
        }

        if (p.essence <= 0.0)
        {
            p.essence = 0.0;
            if (!v.second.empty())
            {
                auto maxIt = std::max_element(v.second.begin(), v.second.end(), [](const auto &v1, const auto &v2){
                    return v1.second < v2.second;
                });
                auto maxV = *maxIt;
                if (v.second.size() > 1)
                {
                    v.second.erase(maxIt);
                    auto secondMaxV = *std::max_element(v.second.begin(), v.second.end(), [](const auto &v1, const auto &v2){
                        return v1.second < v2.second;
                    });

                    if (secondMaxV.second < maxV.second)
                    {
                        p.essence = maxV.second - secondMaxV.second;
                        p.owner = maxV.first.playerId;
                        p.particleType = maxV.first.type;
                    }
                }
                else
                {
                    p.essence = maxV.second;
                    p.owner = maxV.first.playerId;
                    p.particleType = maxV.first.type;
                }
            }
        }

        ParticleParameters &params = gameParameters.particleParameters[p.particleType];
        if (p.essence > params.maxEssenceCapacity)
            p.essence = params.maxEssenceCapacity;
    }
}

void applyDevour(World &w, double t, std::unordered_map<uint16_t, Actions> &actionsByPlayerId, double wid)
{
    std::unordered_map<uint16_t, std::unordered_map<uint16_t, double>> devour;

    for (auto &pl : actionsByPlayerId)
    {
        uint16_t playerId = pl.first;
        Actions &actions = pl.second;

        for (auto &pa: actions.devour)
        {
            if (pa.first >= w.particles.size())
                continue;

            Particle &p = w.particles[pa.first];

            if (p.owner != playerId)
                continue;

            ParticleParameters &thisParams = gameParameters.particleParameters[p.particleType];

            for (auto &f: pa.second)
            {
                if (f.second.amount <= 0.0)
                    continue;

                Particle *oth = getNearbyParticle(w, p, f.first, wid);
                if (oth)
                {
                    double amount = f.second.amount;
                    double maxIncome = thisParams.maxEssenceIncomePerSecond * t;
                    if (amount > maxIncome)
                        amount = maxIncome;

                    ParticleParameters &othParams = gameParameters.particleParameters[oth->particleType];
                    double maxOutcome = othParams.maxEssenceOutcomePerSecond * t;
                    if (amount > maxOutcome)
                        amount = maxOutcome;

                    devour[f.first][pa.first] += amount;
                }
            }
        }
    }

    for (auto &f : devour)
    {
        double totalAmount = 0;
        for (auto &oth : f.second)
        {
            totalAmount += oth.second;
        }

        Particle &p = w.particles[f.first];

        double coef = p.essence >= totalAmount ? 1.0 : p.essence / totalAmount;
        for (auto &othId : f.second)
        {
            double amount = coef * othId.second;
            p.essence -= amount;

            Particle &oth = w.particles[othId.first];
            oth.essence += amount;

            ParticleParameters &params = gameParameters.particleParameters[oth.particleType];
            if (oth.essence > params.maxEssenceCapacity)
                oth.essence = params.maxEssenceCapacity;
        }
    }
}

struct Explosion {
    ExplosionCircle explosionCircle;
    double impulsePerRadian;

    Explosion(const ExplosionCircle &explosionCircle, double impulsePerRadian) : explosionCircle(explosionCircle), impulsePerRadian(impulsePerRadian)
    {}
};

void applyExplosions(World &w, double t, std::unordered_map<uint16_t, Actions> &actionsByPlayerId, double wid)
{
    std::unordered_map<uint16_t, Explosion> explodingParticles;

    for (auto &pl : actionsByPlayerId)
    {
        uint16_t playerId = pl.first;
        Actions &actions = pl.second;

        for (auto id: actions.explode)
        {
            if (id >= w.particles.size())
                continue;

            Particle &p = w.particles[id];

            if (p.owner != playerId)
                continue;

            if (p.particleType == ParticleType::EXPLOSIVE)
            {
                ParticleParameters &thisParams = gameParameters.particleParameters[p.particleType];
                double r = thisParams.explosionRadiusPerEssence * p.essence;
                double impulsePerRadian = thisParams.explosionImpulsePerEssencePerRadian * p.essence;
                explodingParticles.insert(std::make_pair(id, Explosion{ExplosionCircle{r}, impulsePerRadian}));
            }
        }
    }

    {
        uint16_t id = 0;
        for (auto &p: w.particles)
        {
            if (p.particleType == ParticleType::EXPLOSIVE && p.detonated)
            {
                ParticleParameters &thisParams = gameParameters.particleParameters[p.particleType];
                double r = thisParams.explosionRadiusPerEssence * p.essence;
                double impulsePerRadian = thisParams.explosionImpulsePerEssencePerRadian * p.essence;
                explodingParticles.insert(std::make_pair(id, Explosion{ExplosionCircle{r}, impulsePerRadian}));
            }
            ++id;
        }
    }

    if (!explodingParticles.empty())
    {
        uint16_t id = 0;
        for (Particle &p : w.particles)
        {
            for (auto &exp : explodingParticles)
            {
                if (exp.first != id)
                {
                    P relP = relWrapP(p.pos - w.particles[exp.first].pos, wid);
                    exp.second.explosionCircle.addSector(relP, id);
                }
            }

            ++id;
        }

        for (auto &exp : explodingParticles)
        {
            Particle &thisP = w.particles[exp.first];
            double expRad = exp.second.explosionCircle.rad;

            for (auto &s : exp.second.explosionCircle.sectors)
            {
                Particle &p = w.particles[s.id];
                P relP = relWrapP(p.pos - thisP.pos, wid);
                double dist = relP.len();
                P dir = relP / dist;

                double impulse = s.getAngle() * exp.second.impulsePerRadian * (1.0 - dist / expRad);
                p.vel += dir * impulse;
                p._totalImpulseChange2 += sqr(impulse);
                thisP.vel -= dir * impulse * gameParameters.explosionImpulseReturn;
                thisP._totalImpulseChange2 += sqr(impulse * gameParameters.explosionImpulseReturn);
            }

            thisP.essence = 0.0;
            thisP.owner = 0;
            thisP.particleType = ParticleType::NEUTRAL;
            thisP.detonated = false;
        }
    }
}

void simulate(Grid &grid, World &w, double t, std::unordered_map<uint16_t, Actions> &actionsByPlayerId)
{
    for (Particle &p : w.particles)
    {
        p._accel = P(0, 0);
        p._totalImpulseChange2 = 0.0;
    }

    applyForcesAndThrust(w, t, actionsByPlayerId, grid.w);
    applyEssenceTransfer(w, t, actionsByPlayerId, grid.w);
    applyDevour(w, t, actionsByPlayerId, grid.w);
    applyExplosions(w, t, actionsByPlayerId, grid.w);

    grid.reset();

    uint16_t id = 0;
    for (Particle &p : w.particles)
    {
        P impulseDelta = p._accel * t;
        p.vel += impulseDelta;
        p._totalImpulseChange2 += impulseDelta.len2();
        P nextPos = p.pos + p.vel * t;
        p._currentT = 0.0;
        p._updateId = 0;
        grid.raytraceAddToGrid(p.pos, nextPos, id);
        ++id;
    }

    std::priority_queue<CollidingParticles> collidingParticlesQueue;

    id = 0;
    for (Particle &p : w.particles)
    {
        P nextPos = p.pos + p.vel * t;
        generateCollisions(grid, w, p, id, nextPos,
                           t, collidingParticlesQueue);
        ++id;
    }

    while (!collidingParticlesQueue.empty())
    {
        CollidingParticles cp = collidingParticlesQueue.top();
        collidingParticlesQueue.pop();

        Particle &p1 = w.particles[cp.id1];
        Particle &p2 = w.particles[cp.id2];

        if (p1._updateId > cp.updateId1 || p2._updateId > cp.updateId2)
            continue;

        p1.pos = p1.pos + p1.vel * (cp.t - p1._currentT);
        p1._currentT = cp.t;
        p2.pos = p2.pos + p2.vel * (cp.t - p2._currentT);
        p2._currentT = cp.t;

        if (cp.status == CollisionStatus::Intersecting)
        {
            P dp = p2.pos - p1.pos + cp.collisionWrap;
            double dist2 = dp.len2();
            std::cout << globalT << ": " << "INTERS " << p1.pos.x << " " << p1.pos.y << " | " << p2.pos.x << " " << p2.pos.y << " | " << dist2 << std::endl;
            if (dist2 < 1.0 && dist2 > 0)
            {
                dp /= (2.0 - 1e-3)*sqrt(dist2);
                P center = (p1.pos + p2.pos + cp.collisionWrap) * 0.5;
                p2.pos = center + dp;
                p1.pos = center - dp;

                P dv = p2.vel - p1.vel;
                if (dot(dp, dv) < 0)
                {
                    updateVelocities(p1, p2, cp.collisionWrap);
                }
            }
        }
        else
        {
            updateVelocities(p1, p2, cp.collisionWrap);
        }

        p1.pos = wrapP(p1.pos, grid.w);
        p2.pos = wrapP(p2.pos, grid.w);
        p1._updateId++;
        p2._updateId++;

        P p1NextPos = p1.pos + p1.vel * (t - cp.t);
        P p2NextPos = p2.pos + p2.vel * (t - cp.t);

        grid.raytraceAddToGrid(p1.pos, p1NextPos, cp.id1);
        grid.raytraceAddToGrid(p2.pos, p2NextPos, cp.id2);

        generateCollisions(grid, w, p1, cp.id1, p1NextPos, t, collidingParticlesQueue);
        generateCollisions(grid, w, p2, cp.id2, p2NextPos, t, collidingParticlesQueue);
    }

    for (Particle &p : w.particles)
    {
        p.pos = wrapP(p.pos + p.vel * (t - p._currentT), grid.w);

        if (p.owner) {
            ParticleParameters &params = gameParameters.particleParameters[p.particleType];
            if (p._totalImpulseChange2 > sqr(params.essenceLossMinImpulse))
            {
                if (p.particleType != ParticleType::EXPLOSIVE)
                {
                    double deltaImpulse = sqrt(p._totalImpulseChange2) - params.essenceLossMinImpulse;
                    p.essence -= deltaImpulse * params.essenceLossPerImpulse;
                }
                else
                {
                    p.detonated = true;
                }
            }

            if (p.particleType != ParticleType::SOUL)
                p.essence -= gameParameters.essenceDeclinePerSecond * t;

            if (p.particleType == ParticleType::SOUL || p.particleType == ParticleType::REGENERATOR)
            {
                double regen = 1.0 + params.essenceRegenerationSpeedPerSecond * t;
                p.essence *= regen;

                if (p.essence > params.maxEssenceCapacity)
                    p.essence = params.maxEssenceCapacity;
            }

            if (p.essence <= 0.0)
            {
                p.essence = 0.0;
                p.owner = 0;
                p.particleType = ParticleType::NEUTRAL;
                p.detonated = false;
            }
        }
    }

    globalT += t;
}