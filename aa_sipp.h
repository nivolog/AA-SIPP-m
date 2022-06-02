#ifndef AA_SIPP_H
#define AA_SIPP_H

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
class AA_SIPP
{

public:

    AA_SIPP(const Config &config, const char *primitivesName);
    ~AA_SIPP();
    SearchResult startSearch(Map &map, Task &task, DynamicObstacles &obstacles);
    SearchResult sresult;
private:

    bool stopCriterion(const Node &curNode, Node &goalNode);
    std::list<Node> findSuccessors(const Node curNode, const Map &map);

    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
    bool findPath(unsigned int numOfCurAgent, const Map &map);
    void setPriorities(const Task &task);
    double getHValue(int i, int j, double o);
    bool changePriorities(int bad_i);
    unsigned int openSize;
    std::vector<TerminalPoint> point_path;
    std::vector<Primitive> primitives_path;
    std::vector<std::vector<int>> priorities;
    std::vector<int> current_priorities;
    LineOfSight lineofsight;
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

#endif // AA_SIPP_H
