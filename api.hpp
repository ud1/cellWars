#ifndef CELLS_API_HPP
#define CELLS_API_HPP

#include <unordered_map>
#include <unordered_set>

enum class ParticleType {
    NEUTRAL,
    SOUL,
    MOVER,
    ACCUMULATOR,
    SHELL,
    EXPLOSIVE,
    REGENERATOR,
    DEVOUR,

    COUNT
};

struct Force {
    double force = 0.0;
};

struct TransferEssence {
    double amount = 0.0;
    ParticleType targetType = ParticleType::NEUTRAL;
};

struct Thrust {
    P acceleration = P{0.0, 0.0};
};

struct Devour {
    double amount;
};

struct Actions {
    std::unordered_map<uint16_t/*myParticleId*/, std::unordered_map<uint16_t/*targetParticleId*/, Force>> applyForce;
    std::unordered_map<uint16_t/*myParticleId*/, Thrust> thrust;
    std::unordered_map<uint16_t/*myParticleId*/, std::unordered_map<uint16_t/*targetParticleId*/, TransferEssence>> transferEssence;
    std::unordered_map<uint16_t/*myParticleId*/, std::unordered_map<uint16_t/*targetParticleId*/, Devour>> devour;
    std::unordered_set<uint16_t/*myParticleId*/> explode;
};

struct ParticleParameters {
    double essenceRegenerationSpeedPerSecond = 0.0;
    double maxEssenceCapacity = 1000.0;
    double maxEssenceIncomePerSecond = 100.0;
    double maxEssenceOutcomePerSecond = 100.0;
    double maxForce = 10.0;
    double maxThrust = 0.0;
    double explosionRadiusPerEssence = 0.0;
    double explosionImpulsePerEssencePerRadian = 0.0;
    double essenceLossMinImpulse = 10.0;
    double essenceLossPerImpulse = 50.0;
};

struct GameParameters {
    double maxActionDistance2 = sqr(3.5);
    double tick = 0.016;
    double essenceDeclinePerSecond = 1.0;
    double explosionImpulseReturn = 0.1;
    std::unordered_map<ParticleType, ParticleParameters> particleParameters;

    GameParameters()
    {
        {
            ParticleParameters &p = particleParameters[ParticleType::SOUL];
            p.essenceRegenerationSpeedPerSecond = 0.1;
            p.maxThrust = 10.0;
        }

        {
            ParticleParameters &p = particleParameters[ParticleType::MOVER];
            p.maxThrust = 10.0;
        }

        {
            ParticleParameters &p = particleParameters[ParticleType::ACCUMULATOR];
            p.maxEssenceCapacity = 10000.0;
            p.maxEssenceIncomePerSecond = 1000.0;
            p.maxEssenceOutcomePerSecond = 1000.0;
        }

        {
            ParticleParameters &p = particleParameters[ParticleType::SHELL];
            p.maxForce = 50.0;
            p.essenceLossMinImpulse = 20.0;
            p.essenceLossPerImpulse = 10.0;
        }

        {
            ParticleParameters &p = particleParameters[ParticleType::EXPLOSIVE];
            p.explosionRadiusPerEssence = 0.05;
            p.explosionImpulsePerEssencePerRadian = 0.2;
        }

        {
            ParticleParameters &p = particleParameters[ParticleType::REGENERATOR];
            p.essenceRegenerationSpeedPerSecond = 0.05;
        }

        {
            ParticleParameters &p = particleParameters[ParticleType::DEVOUR];
        }
    }
};

extern GameParameters gameParameters;

#endif
