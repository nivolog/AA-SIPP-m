//
// Created by zain on 08.12.2021.
//

#ifndef RRTX_DUBINS_H
#define RRTX_DUBINS_H
#include "point.h"
#include <vector>
#include <limits>
#include <cassert>
#include "constants.h"

using namespace std;

enum DubinsPathSegmentType {
    DUBINS_LEFT = 0,
    DUBINS_STRAIGHT = 1,
    DUBINS_RIGHT = 2
};
const DubinsPathSegmentType dubinsPathType[6][3] = {
        {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},
        {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
        {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT},
        {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
        {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},
        {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}
};

inline double mod2pi(double x)
{
    if (x < 0 && x > DUBINS_ZERO)
        return 0;
    double xm = x - twopi * floor(x / twopi);
    if (twopi - xm < .5 * DUBINS_EPS) xm = 0.;
    return xm;
}

class DubinsPath
{
public:
    DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
               double p = std::numeric_limits<double>::max(), double q = 0.)
            : type_(type)
    {
        length_[0] = t;
        length_[1] = p;
        length_[2] = q;
        assert(t >= 0.);
        assert(p >= 0.);
        assert(q >= 0.);
    }
    double length() const
    {
        return length_[0] + length_[1] + length_[2];
    }
//    DubinsPath& operator=(const DubinsPath& other);
//    DubinsPath (const DubinsPath&);
    /** Path segment types */
    const DubinsPathSegmentType *type_;
    /** Path segment lengths */
    double length_[3];
    /** Whether the path should be followed "in reverse" */
    bool reverse_{false};
};



class Dubins {
public:
    Dubins(double turningRadius = 1.0, bool isSymmetric = false)
            : rho_(turningRadius), isSymmetric_(isSymmetric) {
    }
    double distance(const Dub_Point *state1, const Dub_Point *state2) const;
    double distance(const DubinsPath & dp) const;
    void interpolate(const Dub_Point *from, const DubinsPath &path, double t, Dub_Point *point) const;
    void interpolate(const Dub_Point *from, const Dub_Point *to, double t, Dub_Point *point) const;

    void interpolate(const Dub_Point *from, const Dub_Point *to, double t, bool &firstTime,
                             DubinsPath &path, Dub_Point *point) const;
    bool hasSymmetricDistance() const {
        return isSymmetric_;
    }
    DubinsPath dubins(double d, double alpha, double beta) const;
    DubinsPath dubins(const Dub_Point *x, const Dub_Point *y) const;
    void copyState(Dub_Point *to, const Dub_Point *from) const;
    double rho_;

private:
    bool isSymmetric_;
};


#endif //RRTX_DUBINS_H
