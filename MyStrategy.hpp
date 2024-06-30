#ifndef CELLS_MYSTRATEGY_HPP
#define CELLS_MYSTRATEGY_HPP
#include "world.hpp"

struct IStrategy
{
    uint16_t playerId;

    IStrategy(uint16_t playerId);

    virtual void generateActions(const World &w, Actions &actions) = 0;
};

struct EmptyStrategy : public IStrategy {
    EmptyStrategy(uint16_t playerId) : IStrategy(playerId) {};
    virtual void generateActions(const World &w, Actions &actions) {}
};

struct MyStrategy : public IStrategy {
    MyStrategy(uint16_t playerId);
    virtual void generateActions(const World &w, Actions &actions);
};

#endif //CELLS_MYSTRATEGY_HPP
