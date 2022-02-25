//
// Created by zain on 20.11.2021.
//

#ifndef RRTX_POINT_H
#define RRTX_POINT_H

class Dub_Point {
public:
    Dub_Point() {
        x_ = y_ = theta_ = 0;
    }

    Dub_Point(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {
    }

    double x_, y_, theta_;

    Dub_Point(const Dub_Point &other) {
        x_ = other.x_;
        y_ = other.y_;
        theta_ = other.theta_;
    }

    Dub_Point &operator=(const Dub_Point &pt) {
        x_ = pt.x_;
        y_ = pt.y_;
        theta_ = pt.theta_;
        return *this;
    }

    double getX() const{
        return x_;
    }
    double getY() const{
        return y_;
    }
    double getTheta() const{
        return theta_;
    }
    void setXY(double x, double y){
        x_ = x;
        y_ = y;
    }
    void setTheta_(double theta){
        theta_ = theta;
    }
    void setX(double x){
        x_ = x;
    }
    void setY(double y){
        y_ = y;
    }
    friend bool operator < (Dub_Point a, Dub_Point b);
};

#endif //RRTX_POINT_H
