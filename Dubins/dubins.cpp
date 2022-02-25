//
// Created by zain on 08.12.2021.
//

#include <cmath>
#include <iostream>
#include <algorithm>
#include "../dubins.h"

DubinsPath Dubins::dubins(const Dub_Point *s1, const Dub_Point *s2) const{

    double x1 = s1->getX(), y1 = s1->getY(), th1 = s1->getTheta();
    double x2 = s2->getX(), y2 = s2->getY(), th2 = s2->getTheta();
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    return dubins(d, alpha, beta);
}


double Dubins::distance(const Dub_Point *state1, const Dub_Point *state2) const {
    if (isSymmetric_)
        return rho_ * std::min(dubins(state1, state2).length(), dubins(state2, state1).length());
    return rho_ * dubins(state1, state2).length();
}

void Dubins::interpolate(const Dub_Point *from, const Dub_Point *to, double t, Dub_Point *point) const {
    bool firstTime = true;
    DubinsPath path;
    interpolate(from, to, t, firstTime, path, point);
}

void Dubins::interpolate(const Dub_Point *from, const Dub_Point *to, double t, bool &firstTime, DubinsPath &path,
                         Dub_Point *point) const {
    {
        if (firstTime)
        {
            if (t >= 1.)
            {
                if (to != point)
                    copyState(point, to);
                return;
            }
            if (t <= 0.)
            {
                if (from != point)
                    copyState(point, from);
                return;
            }

            path = dubins(from, to);
            if (isSymmetric_)
            {
                DubinsPath path2(dubins(to, from));
                if (path2.length() < path.length())
                {
                    path2.reverse_ = true;
                    path = path2;
                }
            }
            firstTime = false;
        }
        interpolate(from, path, t, point);
    }

}
DubinsPath dubinsLSL(double d, double alpha, double beta)
{
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
    if (tmp >= DUBINS_ZERO)
    {
        double theta = atan2(cb - ca, d + sa - sb);
        double t = mod2pi(-alpha + theta);
        double p = sqrt(std::max(tmp, (double)0.));
        double q = mod2pi(beta - theta);
//        assert(fabs(p * cos(alpha + t) - sa + sb - d) < 2 * DUBINS_EPS);
//        assert(fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
//        assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
        return DubinsPath(dubinsPathType[0], t, p, q);
    }
    return {};
}

DubinsPath dubinsRSR(double d, double alpha, double beta)
{
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
    if (tmp >= DUBINS_ZERO)
    {
        double theta = atan2(ca - cb, d - sa + sb);
        double t = mod2pi(alpha - theta);
        double p = sqrt(std::max(tmp, (double)0.));
        double q = mod2pi(-beta + theta);
//        assert(fabs(p * cos(alpha - t) + sa - sb - d) < 2* DUBINS_EPS);
//        assert(fabs(p * sin(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
//        assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
        return DubinsPath(dubinsPathType[1], t, p, q);
    }
    return {};
}

DubinsPath dubinsRSL(double d, double alpha, double beta)
{
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
    if (tmp >= DUBINS_ZERO)
    {
        double p = sqrt(std::max(tmp, (double)0.));
        double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
        double t = mod2pi(alpha - theta);
        double q = mod2pi(beta - theta);
//        assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
//        assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
//        assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
        return DubinsPath(dubinsPathType[2], t, p, q);
    }
    return {};
}

DubinsPath dubinsLSR(double d, double alpha, double beta)
{
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
    if (tmp >= DUBINS_ZERO)
    {
        double p = sqrt(std::max(tmp, (double)0.));
        double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
        double t = mod2pi(-alpha + theta);
        double q = mod2pi(-beta + theta);
//        assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
//        assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
//        assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
        return DubinsPath(dubinsPathType[3], t, p, q);
    }
    return {};
}

DubinsPath dubinsRLR(double d, double alpha, double beta)
{
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
    if (fabs(tmp) < 1.)
    {
        double p = twopi - acos(tmp);
        double theta = atan2(ca - cb, d - sa + sb);
        double t = mod2pi(alpha - theta + .5 * p);
        double q = mod2pi(alpha - beta - t + p);
//        assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
//        assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
//        assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
        return DubinsPath(dubinsPathType[4], t, p, q);
    }
    return {};
}

DubinsPath dubinsLRL(double d, double alpha, double beta)
{
    double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
    double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
    if (fabs(tmp) < 1.)
    {
        double p = twopi - acos(tmp);
        double theta = atan2(-ca + cb, d + sa - sb);
        double t = mod2pi(-alpha + theta + .5 * p);
        double q = mod2pi(beta - alpha - t + p);
//        assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
//        assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
//        assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
        return DubinsPath(dubinsPathType[5], t, p, q);
    }
    return {};
}
DubinsPath Dubins::dubins(double d, double alpha, double beta) const{
//    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
//        return {dubinsPathType[0], 0, d, 0};
//
//    DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
//    double len, minLength = path.length();
//    vector<DubinsPath> v;
//    vector<pair<double, int>> v1;
//    v.push_back(path);v1.push_back({path.length(), 0});
//    v.push_back(tmp);v1.push_back({tmp.length(),1});
//    tmp = dubinsRSL(d, alpha, beta);
//    v.push_back(tmp); v1.push_back({tmp.length(), 2});
//    tmp = dubinsLSR(d, alpha, beta);
//    v.push_back(tmp); v1.push_back({tmp.length(), 3});
//    tmp = dubinsRLR(d, alpha, beta);
//    v.push_back(tmp); v1.push_back({tmp.length(), 4});
//    tmp = dubinsLRL(d, alpha, beta);
//    v.push_back(tmp); v1.push_back({tmp.length(), 5});
//    sort(v1.begin(), v1.end());
////    for(auto it:v1){
////        cout<<fixed<<setprecision(3)<<it.first<<" ";
////    }cout<<fixed<<setprecision(3)<<endl;
//    path = v[v1[0].second];
////    cout<<fixed<<setprecision(3)<<tmp.length()<<" "<<path.length()<<" ";
////    if ((len = tmp.length()) < minLength)
////    {
////        minLength = len;
////        path = tmp;
////    }
////    auto tmp4 = dubinsRSL(d, alpha, beta);
//////    cout<<fixed<<setprecision(3)<<tmp.length()<<" ";
////    if ((len = tmp4.length()) < minLength)
////    {
////        minLength = len;
////        path = tmp4;
////    }
////    auto tmp3 = dubinsLSR(d, alpha, beta);
//////    cout<<fixed<<setprecision(3)<<tmp.length()<<" ";
////    if ((len = tmp3.length()) < minLength)
////    {
////        minLength = len;
////        path = tmp3;
////    }
////    auto tmp2 = dubinsRLR(d, alpha, beta);
//////    cout<<fixed<<setprecision(3)<<tmp.length()<<" ";
////    if ((len = tmp2.length()) < minLength)
////    {
////        minLength = len;
////        path = tmp2;
////    }
////    auto tmp1 = dubinsLRL(d, alpha, beta);
//////    cout<<fixed<<setprecision(3)<<tmp.length()<<endl;
////    if ((len = tmp1.length()) < minLength) {
////        path = tmp1;
////        minLength = len;
////    }
////    cout<<fixed<<setprecision(3)<<path.length()<<endl;
//    return path;

    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
        return {dubinsPathType[0], 0, d, 0};

    DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
    double len, minLength = path.length();

    if ((len = tmp.length()) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsRSL(d, alpha, beta);
    if ((len = tmp.length()) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsLSR(d, alpha, beta);
    if ((len = tmp.length()) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsRLR(d, alpha, beta);
    if ((len = tmp.length()) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsLRL(d, alpha, beta);
    if ((len = tmp.length()) < minLength)
        path = tmp;
    return path;
}

void Dubins::copyState(Dub_Point *to, const Dub_Point *from) const {
    to->x_ = from->x_;
    to->y_ = from->y_;
    to->theta_ = from->theta_;
}




void Dubins::interpolate(const Dub_Point *from, const DubinsPath &path, double t, Dub_Point *point) const {
    auto *s = new Dub_Point(0, 0, from->theta_);
    double seg = t * path.length(), phi, v;

    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            phi = s->getTheta();
            seg -= v;
            switch (path.type_[i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi + v) - sin(phi), s->getY() - cos(phi + v) + cos(phi));
                    s->setTheta_(phi + v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi - v) + sin(phi), s->getY() + cos(phi - v) - cos(phi));
                    s->setTheta_(phi - v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                    break;
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s->getTheta();
            seg -= v;
            switch (path.type_[2 - i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi - v) - sin(phi), s->getY() - cos(phi - v) + cos(phi));
                    s->setTheta_(phi - v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi + v) + sin(phi), s->getY() + cos(phi + v) - cos(phi));
                    s->setTheta_(phi + v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() - v * cos(phi), s->getY() - v * sin(phi));
                    break;
            }
        }
    }
    point->setX(s->getX() * rho_ + from->getX());
    point->setY(s->getY() * rho_ + from->getY());
    point->setTheta_(s->getTheta());
    delete s;
}

double Dubins::distance(const DubinsPath &dp) const {
    return rho_*dp.length();
}
