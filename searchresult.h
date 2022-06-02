#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include <vector>
#include <list>
#include <structs.h>

struct ResultPathInfo
{
    bool pathfound;
    double pathlength;
    double runtime;
    std::vector<Primitive> primitives;
    std::vector<TerminalPoint> points;
    unsigned int closed_size, open_size;
    ResultPathInfo()
    {
        runtime = 0;
        pathfound = false;
        pathlength = 0;
        primitives.clear();
        points.clear();
    }
};

struct SearchResult
{
    bool pathfound;
    double makespan;
    double flowtime;
    double runtime;
    unsigned int agents;
    int agentsSolved;
    int tries;
    std::vector<ResultPathInfo> pathInfo;

    SearchResult() : pathInfo(1)
    {
        pathfound = false;
        runtime = 0;
        flowtime = 0;
        makespan = 0;
        agents = 0;
    }

    ~SearchResult()
    {
        pathInfo.clear();
    }

};

#endif // SEARCHRESULT_H
