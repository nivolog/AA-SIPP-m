//
// Created by vlad on 07.06.2022.
//

#ifndef SIPP_MP_H
#define SIPP_MP_H

#include "constraints.h"
#include "lineofsight.h"
#include "config.h"
#include "searchresult.h"
#include "task.h"
#include "dynamicobstacles.h"
#include <math.h>
#include <cmath>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <random>
#include "primitive.h"
#include "dubins.h"
#include <fstream>

#ifdef __linux__
#include <sys/time.h>
#else
#include <windows.h>
#endif

class SIPP_MP {
public:
    SIPP_MP(const Config &config, const char *primitivesName);
    ~SIPP_MP();
    SearchResult startSearch(Map &map, Task &task, DynamicObstacles &obstacles);
    SearchResult sresult;
private:
    bool stopCriterion(const Node &curNode, Node &goalNode);
    std::list<Node> findSuccessors(const Node curNode, const Map &map);

    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
    bool findPath(unsigned int numOfCurAgent, const Map &map);
    double getHValue(int i, int j, double o);
    unsigned int openSize;
    std::vector<TerminalPoint> point_path;
    std::vector<Primitive> primitives_path;
    Agent curagent;
    Constraints *constraints;
    std::shared_ptr<const Config> config;
    Primitives primitives;
    OpenContainer Open;
    ClosedList closed;
    Dubins dubins;
    double intervals_time, cells_time;

    double resolution, angle_step, max_velocity, avg_velocity;
};


#endif //SIPP_MP_H
