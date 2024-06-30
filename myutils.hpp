
#ifndef MY_UTILS_HPP
#define MY_UTILS_HPP

#include <cassert>
#include <tuple>
#include <optional>
#include <algorithm>

#ifdef MY_IMPLEMENTATION
#define GLOBAL_VAR
#else
#define GLOBAL_VAR extern
#endif // MY_IMPLEMENTATION

#include <iostream>
#include <fstream>
#include <iomanip>

#if ENABLE_LOGGING
#include <algorithm>
//GLOBAL_VAR std::ofstream outputFile("out.txt");
#define LOGC(x)  std::cout << x << std::endl;
//#define LOGF(x) outputFile << "T " << g_world->getTick() << "-" << g_self->getTeammateIndex() << "| " << g_self->getRemainingCooldownTicks() << " " << g_self->getRemainingKnockdownTicks() << "| " << g_self->getX() << " " << g_self->getY() << " " << g_self->getAngle() << "| " << g_self->getSpeedX() << " " << g_self->getSpeedY() << "| " << x << std::endl;
//#define LOG(x) LOGC(g_world->getTickIndex() << " " << x);
#define LOG(x) LOGC(x);
#else
#define LOG(x)

#endif // ENABLE_LOGGING

extern long g_tick;


#define _USE_MATH_DEFINES

#include <cmath>
#include <ctime>
#include <vector>

constexpr double EPS = 1e-9;
constexpr float FEPS = 1e-2;

constexpr inline double sqr ( double v )
{
    return v*v;
}

constexpr double PI = 3.14159265358979323846;
constexpr double PI05 = PI * 0.5;

inline double normalizeAngle(double angle)
{
    while ( angle > PI ) {
        angle -= 2.0 * PI;
    }

    while ( angle < -PI ) {
        angle += 2.0 * PI;
    }

    return angle;
}

inline double atan2_approximation2( double y, double x )
{
    if ( x == 0.0 )
    {
        if ( y > 0.0 )
            return PI05;
        if ( y == 0.0 )
            return 0.0;
        return -PI05;
    }

    float atan;
    float z = y/x;
    if ( std::abs( z ) < 1.0 )
    {
        atan = z/(1.0 + 0.28*z*z);
        if ( x < 0.0 )
        {
            if ( y < 0.0 )
                return atan - PI;
            return atan + PI;
        }
    }
    else
    {
        atan = PI05 - z/(z*z + 0.28);
        if ( y < 0.0 )
            return atan - PI;
    }
    return atan;
}

/*inline double angleDiff(double angle1, double angle2)
{
	return std::abs(normalizeAngle(angle1 - angle2));
}*/

struct P {
    explicit P() {}

    explicit P ( double angle ) : x(cos(angle)), y(sin(angle)) {}
    constexpr explicit P ( double x, double y ) : x ( x ), y ( y ) {}

    double dist ( P const &o ) const {
        return sqrt ( sqr ( x - o.x ) + sqr ( y - o.y ) );
    }

    double maxDist ( P const &o ) const {
        return std::max ( std::abs ( x - o.x ), std::abs ( y - o.y ) );
    }

    constexpr double dist2 ( P const &o ) const {
        return sqr ( x - o.x ) + sqr ( y - o.y );
    }

    constexpr double len2() const {
        return x*x + y*y;
    }

    double len() const {
        return sqrt ( x*x + y*y );
    }

    P norm() const {
        double l = len();
        if ( l == 0.0 )
            return P ( 1.0, 0.0 );
        return P ( x / l, y / l );
    }

    constexpr P conj() const {
        return P(y, -x);
    }

    constexpr P conjR() const {
        return P(-y, x);
    }

    constexpr P &operator += (const P &o)
    {
        x += o.x;
        y += o.y;
        return *this;
    }

    constexpr P &operator -= (const P &o)
    {
        x -= o.x;
        y -= o.y;
        return *this;
    }

    constexpr P &operator *= (double v)
    {
        x *= v;
        y *= v;
        return *this;
    }

    constexpr P &operator /= (double v)
    {
        double r = 1.0 / v;
        x *= r;
        y *= r;
        return *this;
    }

    constexpr bool operator == ( const P &o ) const {
        return ( std::abs ( x - o.x ) + std::abs ( y - o.y ) ) < EPS;
    }

    constexpr bool operator != ( const P &o ) const {
        return ( std::abs ( x - o.x ) + std::abs ( y - o.y ) ) >= EPS;
    }

    P rotate ( double angle ) const {
        double cs = cos ( angle );
        double sn = sin ( angle );
        return P ( x * cs - y * sn, x * sn + y * cs );
    }

    double getAngle () const {
        double absoluteAngleTo = atan2 (this->y, this->x );
        return absoluteAngleTo;
    }

    double getAngleFast () const {
        double absoluteAngleTo = atan2_approximation2(this->y, this->x );
        return absoluteAngleTo;
    }

    constexpr P rotate(const P &sinCos) const
    {
        return P(x * sinCos.x - y * sinCos.y, y * sinCos.x + x * sinCos.y);
    }

    constexpr P rotateR(const P &sinCos) const
    {
        return P(x * sinCos.x + y * sinCos.y, y * sinCos.x - x * sinCos.y);
    }

    constexpr P floor() const
    {
        return P(std::floor(x), std::floor(y));
    }

    template<int ind>
    double comp() {
        if (ind == 0)
            return x;
        return y;
    }

    double x, y;
};

inline P unrotate(const P &fromDir, const P& toDir)
{
    return P(toDir.x * fromDir.x + toDir.y * fromDir.y, toDir.y * fromDir.x - toDir.x * fromDir.y).norm();
}

inline P clampDir(const P &value, const P &maxValue)
{
    if (value.x >= maxValue.x)
        return value;

    if (value.y > 0.0)
        return maxValue;

    return P(maxValue.x, -maxValue.y);
}

inline bool isAngleAbsLessThan(const P &value1, const P &value2)
{
    return value1.x >= value2.x;
}

inline bool isAngleLessThan(const P &value1, const P &value2)
{
    P delta = unrotate(value1, value2);
    return delta.y > 0;
}

inline double clamp ( double v, double vmin, double vmax )
{
    if ( v < vmin )
        return vmin;
    if ( v > vmax )
        return vmax;
    return v;
}

inline int clampI ( int v, int vmin, int vmax )
{
    if ( v < vmin )
        return vmin;
    if ( v > vmax )
        return vmax;
    return v;
}

inline float clampF ( float v, float vmin, float vmax )
{
    if ( v < vmin )
        return vmin;
    if ( v > vmax )
        return vmax;
    return v;
}

inline P clampP(const P &p, const P &minP, const P &maxP)
{
    return P(clamp(p.x, minP.x, maxP.x), clamp(p.y, minP.y, maxP.y));
}

//#ifdef ENABLE_LOGGING
inline std::ostream &operator << ( std::ostream &str, const P&p )
{
    str << "(" << p.x << "," << p.y << ")";
    return str;
}
//#endif

constexpr inline P operator - ( P p1, P p2 )
{
    return P ( p1.x - p2.x, p1.y - p2.y );
}

constexpr inline P operator + ( P p1, P p2 )
{
    return P ( p1.x + p2.x, p1.y + p2.y );
}

constexpr inline P operator * ( P p1, double v )
{
    return P ( p1.x * v, p1.y * v );
}

constexpr inline P operator / ( P p1, double v )
{
    return P ( p1.x / v, p1.y / v );
}

constexpr inline P operator / ( double v, P p1 )
{
    return P ( v / p1.x, v / p1.y );
}

constexpr inline P operator * ( P p1, P p2 )
{
    return P ( p1.x * p2.x, p1.y * p2.y );
}

constexpr inline P operator / ( P p1, P p2 )
{
    return P ( p1.x / p2.x, p1.y / p2.y );
}

constexpr inline double dot ( P p1, P p2 )
{
    return p1.x * p2.x + p1.y * p2.y;
}

constexpr inline double cross ( P p1, P p2 )
{
    return p1.x * p2.y - p1.y * p2.x;
}

inline P clampV(const P &v, double len)
{
    double l = v.len();
    if (l > len)
        return v * (len / l);

    return v;
}

struct IP {
    int x, y;

    explicit IP() {}

    constexpr explicit IP ( int x, int y ) : x ( x ), y ( y ) {}

    IP(const P &p) {
        x = std::floor(p.x);
        y = std::floor(p.y);
    }

    IP incY() const
    {
        return IP(x, y + 1);
    }

    void limitWH(int w, int h)
    {
        x = clampI(x, 0, w - 1);
        y = clampI(y, 0, h - 1);
    }

    int ind(int w) const {
        return y * w + x;
    }

    bool operator == ( const IP &o ) const
    {
        return x == o.x && y == o.y;
    }

    bool operator != ( const IP &o ) const
    {
        return x != o.x || y != o.y;
    }

    friend bool operator<(const IP &lhs, const IP &rhs);

    int dist(const IP &p) const
    {
        return std::abs(p.x - x) + std::abs(p.y - y);
    }

    P toP() const {
        return P(x, y);
    }

    bool isInSquare(int sqSize) const {
        return x >= 0 && y >= 0 && x < sqSize && y < sqSize;
    }

    static IP fromInd(int ind, int mapSize)
    {
        return IP(ind % mapSize, ind / mapSize);
    }
};

struct IPHash
{
    std::size_t operator()(IP const&p) const noexcept
    {
        return p.y * 13441 + p.x;
    }
};


#ifdef ENABLE_LOGGING
inline std::ostream &operator << ( std::ostream &str, const IP&p )
{
    str << "(" << p.x << "," << p.y << ")";
    return str;
}
#endif

inline bool operator<(const IP &lhs, const IP &rhs)
{
    return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
}

constexpr inline IP operator - ( IP p1, IP p2 )
{
    return IP ( p1.x - p2.x, p1.y - p2.y );
}

constexpr inline IP operator + ( IP p1, IP p2 )
{
    return IP ( p1.x + p2.x, p1.y + p2.y );
}

constexpr inline IP operator * ( IP p1, int v )
{
    return IP ( p1.x * v, p1.y * v );
}

constexpr inline IP operator / ( IP p1, int v )
{
    return IP ( p1.x / v, p1.y / v );
}

inline bool angleLess(const P &p1, const P &p2)
{
    return p1.x > p2.x;
}

inline bool angleLessOrEq(const P &p1, const P &p2)
{
    return p1.x >= p2.x;
}

constexpr inline P closestPointToSegment(const P &segP1, const P &segP2, const P &p)
{
    P r = segP2 - segP1;
    double rlen2 = r.len2();
    if (rlen2 == 0.0)
        return segP1;

    P t = p - segP1;

    double d = dot(r, t) / rlen2;
    if (d >= 1.0)
        return segP2;
    if (d <= 0.0)
        return segP1;

    P res = segP1 + r * d;
    return res;
}

constexpr inline bool checkSegsIntersect(const P &start1, const P &end1, const P &start2, const P &end2)
{
    P a = end1 - start1;
    P b = start2 - end2;
    P d = start2 - start1;

    double det = a.x * b.y - a.y * b.x;

    if (det == 0)
        return false;

    double r = (d.x * b.y - d.y * b.x) / det;
    double s = (a.x * d.y - a.y * d.x) / det;

    return !(r < 0 || r > 1 || s < 0 || s > 1);
}

struct BBox {
    P minP, maxP;

    static BBox cubeAt(int x, int y)
    {
        BBox res;
        res.minP = P(x, y);
        res.maxP = P(x + 1, y + 1);
        return res;
    }

    static BBox fromCenterAndHalfSize(const P &center, double h05)
    {
        BBox res;
        res.minP = center - P(h05, h05);
        res.maxP = center + P(h05, h05);
        return res;
    }

    static BBox fromBottomCenterAndSize(const P &bottomCenter, const P &size)
    {
        BBox bBox;
        bBox.minP = bottomCenter - P(size.x * 0.5, 0);
        bBox.maxP = bottomCenter + P(size.x * 0.5, size.y);
        return bBox;
    }

    double calculateXOffset(const BBox &b, double offset) const
    {
        if (b.maxP.y > (minP.y) && b.minP.y < (maxP.y))
        {
            if (offset > 0.0f && b.maxP.x <= (minP.x))
            {
                double d = minP.x - b.maxP.x;
                if (d < offset)
                    offset = d - EPS;
            }

            if (offset < 0.0f && (b.minP.x) >= maxP.x)
            {
                double d = maxP.x - b.minP.x;
                if (d > offset)
                    offset = d + EPS;
            }
        }

        return offset;
    }

    double calculateYOffset(const BBox &b, double offset) const
    {
        if (b.maxP.x > (minP.x) && b.minP.x < (maxP.x))
        {
            if (offset > 0.0f && b.maxP.y <= (minP.y))
            {
                double d = minP.y - b.maxP.y;
                if (d < offset)
                    offset = d - EPS;
            }

            if (offset < 0.0f && (b.minP.y) >= maxP.y)
            {
                double d = maxP.y - b.minP.y;
                if (d > offset)
                    offset = d + EPS;
            }
        }

        return offset;
    }

    bool intersects(const BBox &b) const {
        return (b.maxP.x > minP.x) && (b.minP.x < maxP.x) && (b.maxP.y > minP.y) && (b.minP.y < maxP.y);
    }

    bool contains(const P &p) const {
        return minP.x <= p.x && p.x <= maxP.x && minP.y <= p.y && p.y <= maxP.y;
    }

    std::pair<bool, P> rayIntersection(const P&p, const P &ray) const {
        if (ray.x > 0)
        {
            if (p.x < minP.x)
            {
                double tx = (minP.x - p.x) / ray.x;

                P p1 = p + ray * tx;
                if (p1.y <= maxP.y + EPS && p1.y >= minP.y - EPS)
                    return std::make_pair(true, p1);
            }
        }
        else if (ray.x < 0)
        {
            if (maxP.x < p.x)
            {
                double tx = (maxP.x - p.x) / ray.x;

                P p1 = p + ray * tx;
                if (p1.y <= maxP.y + EPS && p1.y >= minP.y - EPS)
                    return std::make_pair(true, p1);
            }
        }

        if (ray.y > 0)
        {
            if (p.y < minP.y)
            {
                double ty = (minP.y - p.y) / ray.y;

                P p2 = p + ray * ty;
                if (p2.x <= maxP.x + EPS && p2.x >= minP.x - EPS)
                    return std::make_pair(true, p2);
            }
        }
        else if (ray.y < 0)
        {
            if (maxP.y < p.y)
            {
                double ty = (maxP.y - p.y) / ray.y;

                P p2 = p + ray * ty;
                if (p2.x <= maxP.x + EPS && p2.x >= minP.x - EPS)
                    return std::make_pair(true, p2);
            }
        }

        return std::make_pair(false, P(0, 0));
    }

    void expand(double delta)
    {
        minP -= P(delta, delta);
        maxP += P(delta, delta);
    }

    P getCorner(int i) const
    {
        switch (i)
        {
            case 0: return minP;
            case 1: return P(minP.x, maxP.y);
            case 2: return maxP;
            case 3: return P(maxP.x, minP.y);
        }

        assert(false);
        return P(0, 0);
    }

    P getCenter() const {
        return (minP + maxP) * 0.5;
    }
};

inline std::optional<double> rayCircleIntersection(const P &relativeCirclePos, const P &unitRayDir, double circleRad)
{
    double p2 = relativeCirclePos.len2();
    double s = dot(unitRayDir, relativeCirclePos);
    double D = sqr(s) - p2 + sqr(circleRad);
    if (D < 0)
        return {};

    double sqrtD = sqrt(D);
    if (s <= 0) {
        if (s + sqrtD >= 0)
            return s + sqrtD;
        else
            return {};
    }

    return std::max(0.0, s - sqrtD);
}

inline double unitCircleIntersection(double relativeCirclePos, double unitRayX)
{
    double s = unitRayX * relativeCirclePos;

    double D = (sqr(unitRayX) - 1) * sqr(relativeCirclePos) + 1;

    double sqrtD = sqrt(D);
    return std::abs(s + sqrtD);
}

template<class BidirIt, class UnaryPredicate>
BidirIt unstable_remove_if(BidirIt first, BidirIt last, UnaryPredicate p)
{
    while (1) {
        while ((first != last) && !p(*first)) {
            ++first;
        }
        if (first == last--) break;
        while ((first != last) && p(*last)) {
            --last;
        }
        if (first == last) break;
        *first++ = std::move(*last);
    }
    return first;
}

template<class T, class UnaryPredicate>
void filterVector(std::vector<T> &v, UnaryPredicate p)
{
    v.erase(unstable_remove_if(v.begin(), v.end(), p), v.end());
    //v.erase(std::remove_if(v.begin(), v.end(), p), v.end());
}

template<typename T, typename Func>
const std::pair<const T*, double> findMax(const std::vector<T> &v, Func f)
{
    double resS = -1e9;
    const T* resT = nullptr;
    for (const T&t : v)
    {
        double s = f(t);
        if (s > resS)
        {
            resS = s;
            resT = &t;
        }
    }

    return std::make_pair(resT, resS);
}

struct TimeMeasure
{
    TimeMeasure(double &total) : total(total) {
        c_start = std::clock();
    }

    ~TimeMeasure()
    {
        std::clock_t c_end = std::clock();
        total += 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
    }

    double &total;
    std::clock_t c_start;
};

inline double lerp(double a, double b, double coef)
{
    return a + (b - a) * coef;
}

template<typename T>
void sortAndUniqueVector(std::vector<T> &v)
{
    std::sort( v.begin(), v.end() );
    v.erase( std::unique( v.begin(), v.end() ), v.end() );
}

#endif

