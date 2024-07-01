#ifndef CELLS_WORLD_HPP
#define CELLS_WORLD_HPP

#include <vector>
#include <queue>
#include "myutils.hpp"
#include "api.hpp"

struct Particle {
    P pos, vel, _accel;
    uint32_t _updateId;
    ParticleType particleType = ParticleType::NEUTRAL;
    uint8_t owner = 0;
    double essence = 0.0;
    double _currentT;
    double _totalImpulseChange2;
    bool detonated = false;
};

struct World {
    uint32_t width = 512;
    std::vector<Particle> particles;
};

const size_t MAX_PARTICLES_IN_CELL = 7;

struct GridCell {
    uint8_t version = 0;
    uint8_t size = 0;
    uint16_t particleIds[MAX_PARTICLES_IN_CELL];
};

inline uint32_t wrap(int c, uint32_t w)
{
    while (c < 0)
        c += (int) w;

    while (c >= (int) w)
        c -= (int) w;

    return c;
}

inline double wrapF(double c, double w)
{
    while (c < 0)
        c += w;

    while (c >= w)
        c -= w;

    return c;
}

inline double relWrapF(double c, double w)
{
    double w05 = w * 0.5;
    while (c < -w05)
        c += w;

    while (c > w05)
        c -= w;

    return c;
}

inline P wrapP(P p, double w)
{
    return P(wrapF(p.x, w), wrapF(p.y, w));
}

inline P relWrapP(P p, double w)
{
    return P(relWrapF(p.x, w), relWrapF(p.y, w));
}

struct Grid {
    std::vector<GridCell> cells;
    std::unordered_map<uint32_t, std::vector<uint16_t>> extraParticles;
    uint32_t w;
    uint8_t version = 0;

    Grid(uint32_t w) : w(w)
    {
        cells.resize(w*w);
    }

    void reset()
    {
        ++version;
        if (version == 0)
        {
            for (GridCell &g : cells)
            {
                g.version = 0;
                g.size = 0;
            }
            ++version;
        }

        extraParticles.clear();
    }

    void add(uint32_t x, uint32_t y, uint16_t id)
    {
        assert(x >= 0 && x < w);
        assert(y >= 0 && y < w);

//        std::cout << x << " " << y << std::endl;

        uint32_t  ind = x + y*w;
        GridCell &g = cells[ind];

        if (g.version != version)
        {
            g.version = version;
            g.size = 1;
            g.particleIds[0] = id;
        }
        else if (g.size < MAX_PARTICLES_IN_CELL)
        {
            for (uint8_t i = 0; i < g.size; ++i)
            {
                if (g.particleIds[i] == id)
                    return;
            }
            g.particleIds[g.size] = id;
            g.size++;
        }
        else
        {
            std::vector<uint16_t> &v = extraParticles[ind];
            if (std::find(v.begin(), v.end(), id) != v.end())
                return;
            v.push_back(id);
        }
    }

    void addWrapped(int x, int y, uint16_t id)
    {
        add(wrap(x, w), wrap(y, w), id);
    }

    void raytraceAddToGrid(P a, P b, uint16_t id)
    {
        raytraceSquare(a, b, [this, id](IP p) {
            addWrapped(p.x, p.y, id);
            return true;
        });
    }

    template<typename VisitCell>
    void raytracePoint(P a, P b, VisitCell visitCell)
    {
        IP ip = a;
        IP gridB = b;
        P dir = b - a;
        if (!visitCell(ip))
            return;

        if (gridB.x >= ip.x)
        {
            if (gridB.y >= ip.y)
            {
                while (ip != gridB)
                {
                    double c = cross(dir, (ip + IP(1, 1)).toP() - a);
//                    std::cout << c << std::endl;
                    if (c > 0.0)
                    {
                        ip.x++;
                    }
                    else if (c < 0.0)
                    {
                        ip.y++;
                    }
                    else
                    {
                        ip.x++;
                        ip.y++;
                    }
                    if (!visitCell(ip))
                        return;
                }
            }
            else
            {
                while (ip != gridB)
                {
                    double c = cross(dir, (ip + IP(1, 0)).toP() - a);
//                    std::cout << c << std::endl;
                    if (c < 0.0)
                    {
                        ip.x++;
                    }
                    else if (c > 0.0)
                    {
                        ip.y--;
                    }
                    else
                    {
                        ip.x++;
                        ip.y--;
                    }
                    if (!visitCell(ip))
                        return;
                }
            }
        }
        else
        {
            if (gridB.y >= ip.y)
            {
                while (ip != gridB)
                {
                    double c = cross(dir, (ip + IP(0, 1)).toP() - a);
//                    std::cout << c << std::endl;
                    if (c < 0.0)
                    {
                        ip.x--;
                    }
                    else if (c > 0.0)
                    {
                        ip.y++;
                    }
                    else
                    {
                        ip.x--;
                        ip.y++;
                    }
                    if (!visitCell(ip))
                        return;
                }
            }
            else
            {
                while (ip != gridB)
                {
                    double c = cross(dir, ip.toP() - a);
//                    std::cout << c << std::endl;
                    if (c > 0.0)
                    {
                        ip.x--;
                    }
                    else if (c < 0.0)
                    {
                        ip.y--;
                    }
                    else
                    {
                        ip.x--;
                        ip.y--;
                    }
                    if (!visitCell(ip))
                        return;
                }
            }
        }
    }

    template<typename VisitCell>
    void raytraceSquare(P a, P b, VisitCell visitCell)
    {
        IP ip = a;
        P dir = b - a;
        visitCell(ip);

        P frac = a - a.floor();
        if (frac.x > 0.5)
        {
            if (frac.y > 0.5)
            {
                visitCell(ip + IP(1, 0));
                visitCell(ip + IP(0, 1));
                visitCell(ip + IP(1, 1));
            }
            else if (frac.y < 0.5)
            {
                visitCell(ip + IP(1, 0));
                visitCell(ip + IP(0, -1));
                visitCell(ip + IP(1, -1));
            }
            else
            {
                visitCell(ip + IP(1, 0));
                visitCell(ip + IP(0, -1));
                visitCell(ip + IP(1, -1));
                visitCell(ip + IP(1, 1));
                visitCell(ip + IP(0, 1));
            }
        }
        else if (frac.x < 0.5)
        {
            if (frac.y > 0.5)
            {
                visitCell(ip + IP(-1, 0));
                visitCell(ip + IP(0, 1));
                visitCell(ip + IP(-1, 1));
            }
            else if (frac.y < 0.5)
            {
                visitCell(ip + IP(-1, 0));
                visitCell(ip + IP(0, -1));
                visitCell(ip + IP(-1, -1));
            }
            else
            {
                visitCell(ip + IP(-1, 0));
                visitCell(ip + IP(0, -1));
                visitCell(ip + IP(-1, -1));
                visitCell(ip + IP(-1, 1));
                visitCell(ip + IP(0, 1));
            }
        }
        else
        {
            if (frac.y > 0.5)
            {
                visitCell(ip + IP(0, 1));
                visitCell(ip + IP(-1, 0));
                visitCell(ip + IP(1, 0));
                visitCell(ip + IP(-1, 1));
                visitCell(ip + IP(1, 1));
            }
            else if (frac.y < 0.5)
            {
                visitCell(ip + IP(0, -1));
                visitCell(ip + IP(-1, 0));
                visitCell(ip + IP(1, 0));
                visitCell(ip + IP(-1, -1));
                visitCell(ip + IP(1, -1));
            }
            else
            {
                visitCell(ip + IP(-1, -1));
                visitCell(ip + IP(-1, 0));
                visitCell(ip + IP(-1, 1));
                visitCell(ip + IP(0, -1));
                visitCell(ip + IP(0, 1));
                visitCell(ip + IP(1, -1));
                visitCell(ip + IP(1, 0));
                visitCell(ip + IP(1, 1));
            }
        }

        bool dirXPos = dir.x >= 0;
        bool dirYPos = dir.y >= 0;

        if (dirXPos)
        {
            a.x += 0.5;
            b.x += 0.5;
        }
        else
        {
            a.x -= 0.5;
            b.x -= 0.5;
        }

        if (dirYPos)
        {
            a.y += 0.5;
            b.y += 0.5;
        }
        else
        {
            a.y -= 0.5;
            b.y -= 0.5;
        }

        ip = a;
        IP gridB = b;

        if (dirXPos)
        {
            if (dirYPos)
            {
                while (ip != gridB)
                {
                    double c = cross(dir, (ip + IP(1, 1)).toP() - a);
                    if (c > 0.0)
                    {
                        ip.x++;
                        if (!visitCell(ip) + !visitCell(ip + IP(0, -1)))
                            return;
                    }
                    else if (c < 0.0)
                    {
                        ip.y++;
                        if (!visitCell(ip) + !visitCell(ip + IP(-1, 0)))
                            return;
                    }
                    else
                    {
                        ip.x++;
                        ip.y++;
                        if (!visitCell(ip) + !visitCell(ip + IP(-1, 0)) + !visitCell(ip + IP(0, -1)))
                            return;
                    }
                }
            }
            else
            {
                while (ip != gridB)
                {
                    double c = cross(dir, (ip + IP(1, 0)).toP() - a);
                    if (c < 0.0)
                    {
                        ip.x++;
                        if (!visitCell(ip) + !visitCell(ip + IP(0, 1)))
                            return;
                    }
                    else if (c > 0.0)
                    {
                        ip.y--;
                        if (!visitCell(ip) + !visitCell(ip + IP(-1, 0)))
                            return;
                    }
                    else
                    {
                        ip.x++;
                        ip.y--;
                        if (!visitCell(ip) + !visitCell(ip + IP(-1, 0)) + !visitCell(ip + IP(0, 1)))
                            return;
                    }
                }
            }
        }
        else
        {
            if (dirYPos)
            {
                while (ip != gridB)
                {
                    double c = cross(dir, (ip + IP(0, 1)).toP() - a);
                    if (c < 0.0)
                    {
                        ip.x--;
                        if (!visitCell(ip) + !visitCell(ip + IP(0, -1)))
                            return;
                    }
                    else if (c > 0.0)
                    {
                        ip.y++;
                        if (!visitCell(ip) + !visitCell(ip + IP(1, 0)))
                            return;
                    }
                    else
                    {
                        ip.x--;
                        ip.y++;
                        if (!visitCell(ip) + !visitCell(ip + IP(1, 0)) + !visitCell(ip + IP(0, -1)))
                            return;
                    }
                }
            }
            else
            {
                while (ip != gridB)
                {
                    double c = cross(dir, ip.toP() - a);
//                    std::cout << c << std::endl;
                    if (c > 0.0)
                    {
                        ip.x--;
                        if (!visitCell(ip) + !visitCell(ip + IP(0, 1)))
                            return;
                    }
                    else if (c < 0.0)
                    {
                        ip.y--;
                        if (!visitCell(ip) + !visitCell(ip + IP(1, 0)))
                            return;
                    }
                    else
                    {
                        ip.x--;
                        ip.y--;
                        if (!visitCell(ip) + !visitCell(ip + IP(1, 0)) + !visitCell(ip + IP(0, 1)))
                            return;
                    }
                }
            }
        }
    }

    template<typename IdVisitor>
    void iterateCell(uint32_t x, uint32_t y, IdVisitor idVisitor)
    {
        uint32_t ind = x + y * w;
        GridCell &cell = cells[ind];
        if (cell.version == version)
        {
            uint8_t sz = cell.size;
            if (sz > MAX_PARTICLES_IN_CELL)
                sz = MAX_PARTICLES_IN_CELL;

            for (uint8_t i = 0; i < sz; ++i)
            {
                uint16_t id = cell.particleIds[i];
                idVisitor(id);
            }

            if (cell.size > MAX_PARTICLES_IN_CELL)
            {
                for (uint16_t v : extraParticles[ind])
                    idVisitor(v);
            }
        }
    }
};

void simulate(Grid &grid, World &w, double t, std::unordered_map<uint16_t/*playerId*/, Actions> &actionsByPlayerId);

#endif //CELLS_WORLD_HPP
