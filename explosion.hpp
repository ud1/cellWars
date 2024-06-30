#ifndef CELLS_EXPLOSION_HPP
#define CELLS_EXPLOSION_HPP

#include "myutils.hpp"

struct Sector {
    P start, end;
    double R;
    uint16_t id;

    double getAngle() const {
        return unrotate(start, end).getAngle();
    }
};

inline bool angleLessFromThirdQuadrant(const P &p1, const P &p2)
{
    if (p1.y <= 0.0 && p2.y <= 0.0)
    {
        return p1.x < p2.x;
    }

    if (p1.y >= 0.0 && p2.y >= 0.0)
    {
        return p1.x > p2.x;
    }

    return p1.y < p2.y;
}

inline bool angleLessOrEqFromThirdQuadrant(const P &p1, const P &p2)
{
    if (p1.y <= 0.0 && p2.y <= 0.0)
    {
        return p1.x <= p2.x;
    }

    if (p1.y >= 0.0 && p2.y >= 0.0)
    {
        return p1.x >= p2.x;
    }

    return p1.y <= p2.y;
}

struct ExplosionCircle {
    double rad;
    double rad2;
    std::vector<Sector> sectors;

    ExplosionCircle(double rad) : rad(rad), rad2(sqr(rad))
    {
    }

    void addSector(const P &relP, uint16_t id)
    {
        double dist2 = relP.len2();
        if (dist2 > rad2)
            return;

        double R = sqrt(dist2);
        P dir = relP / R;

        double sinHalfA = 0.5 / R;
        double cosHalfA = sqrt(1 - sqr(sinHalfA));

        Sector newSector;
        P newSectorStart = newSector.start = P(cosHalfA, -sinHalfA);
        P newSectorEnd = newSector.end = P(cosHalfA, sinHalfA);
        newSector.R = R;
        newSector.id = id;
        newSector.start = newSector.start.rotate(dir);
        newSector.end = newSector.end.rotate(dir);

        if (sectors.empty())
        {
            sectors.push_back(newSector);
            return;
        }

        std::vector<Sector> newSectors;

        size_t minAngleInd = 0;
        P minAngle = sectors[0].start.rotateR(dir);
        for (size_t i = 1; i < sectors.size(); ++i)
        {
            P startAngle = sectors[i].start.rotateR(dir);
            if (angleLessFromThirdQuadrant(startAngle, minAngle))
            {
                minAngleInd = i;
                minAngle = startAngle;
            }
        }

        bool inserted = false;
        for (size_t i = 0; i < sectors.size(); ++i)
        {
            unsigned ind = (i + minAngleInd) % sectors.size();
            Sector &s = sectors[ind];
            P start = s.start.rotateR(dir);
            P end = s.end.rotateR(dir);

            if (inserted || angleLessOrEqFromThirdQuadrant(end, newSectorStart))
            {
                newSectors.push_back(s);
            }
            else if (angleLessOrEqFromThirdQuadrant(newSectorEnd, start))
            {
                newSectors.push_back(newSector);
                inserted = true;
                newSectors.push_back(s);
            }
            else if (angleLessOrEqFromThirdQuadrant(start, newSectorStart) && angleLessFromThirdQuadrant(newSectorStart, end))
            {
                assert(!inserted);

                if (s.R < R)
                {
                    newSectors.push_back(s);
                    newSector.start = s.end;
                    newSectorStart = end;
                    if (angleLessOrEqFromThirdQuadrant(newSectorEnd, newSectorStart))
                        inserted = true;
                }
                else
                {
                    Sector copy = s;
                    copy.end = newSector.start;
                    newSectors.push_back(copy);
                }
            }
            else if (angleLessOrEqFromThirdQuadrant(newSectorStart, start) && angleLessFromThirdQuadrant(start, newSectorEnd))
            {
                assert(!inserted);

                if (s.R < R)
                {
                    newSector.end = s.start;
                    newSectorEnd = start;
                    if (angleLessFromThirdQuadrant(newSectorStart, newSectorEnd))
                        newSectors.push_back(newSector);
                    inserted = true;
                    newSectors.push_back(s);
                }
                else if (angleLessFromThirdQuadrant(newSectorStart, end) && angleLessFromThirdQuadrant(end, newSectorEnd))
                {
                    continue;
                }
                else
                {
                    newSectors.push_back(newSector);
                    inserted = true;

                    Sector copy = s;
                    copy.start = newSector.end;
                    newSectors.push_back(copy);
                }
            }
        }

        if (!inserted && angleLessFromThirdQuadrant(newSectorStart, newSectorEnd))
        {
            newSectors.push_back(newSector);
        }

        sectors.swap(newSectors);
    }
};

#endif
