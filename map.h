#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>
#include "tinyxml2.h"
#include "gl_const.h"
#include "lineofsight.h"

class Map
{
private:
    std::vector<std::vector<double>> distances;
    void dt(float *image);
    float *dt1(float *f, int n);
    std::vector<std::vector<int>> GridLow;
    std::vector<std::vector<int>> GridHigh;
public:
    void computeDistances();
    std::vector<std::vector<int>> Grid;
    unsigned int height, width;

public:
    Map();
    ~Map();
    bool getMap(const char* FileName);
    bool CellIsTraversable (int i, int j) const;
    bool CellOnGrid (int i, int j) const;
    bool CellIsObstacle(int i, int j) const;
    int  getValue(int i, int j) const;
    double getDistance(int i, int j) const;

    bool CellCloseToObst (int i, int j) const;
    bool CellFarFromObst (int i, int j) const;
};

#endif
