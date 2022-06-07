//
// Created by vlad on 07.06.2022.
//

#include "sipp_mp.h"


SIPP_MP::SIPP_MP(const Config &config, const char *primitivesName)
{
    this->config = std::make_shared<const Config> (config);
    openSize = 0;
    constraints = nullptr;
    intervals_time = 0;
    cells_time = 0;
    this->resolution = this->config->resolution;
    primitives.loadPrimitives(primitivesName, this->config->resolution);
    this->angle_step = primitives.angle_step;
    this->max_velocity = primitives.max_velocity;
    this->avg_velocity = primitives.avg_velocity;
    this->dubins = Dubins(2 / this->resolution, false);
}

SIPP_MP::~SIPP_MP()
{
}

bool SIPP_MP::stopCriterion(const Node &curNode, Node &goalNode)
{
    if(Open.isEmpty())
    {
        std::cout << "OPEN list is empty! ";
        return true;
    }
    if( std::abs(curNode.i - curagent.goal_i) <= CN_POS_TOL &&
        std::abs(curNode.j - curagent.goal_j) <= CN_POS_TOL &&
        std::min(360 - std::abs(curNode.heading - curagent.goal_heading), std::abs(curNode.heading - curagent.goal_heading)) <= CN_ANG_TOL &&
        curNode.interval.end == CN_INFINITY &&
        curNode.speed == 0)
    {
        goalNode = curNode;
        return true;
    }
    return false;
}

double SIPP_MP::getHValue(int i, int j, double o)
{
    const Dub_Point curr = Dub_Point(0, 0, fmod(360-o, 360)/360*2*PI);
    const Dub_Point goal = Dub_Point(curagent.goal_j-j, -(curagent.goal_i-i), fmod(360-curagent.goal_heading, 360)/360*2*PI);
    double res = 1.5 * dubins.distance(&curr, &goal) * this->resolution;
    return res;


    double w = 1.0;
    int dx = j - curagent.goal_j;
    int dy = i - curagent.goal_i;

    //!Diagonal distance
//    return (min(abs(dy), abs(dx))*sqrt(2) + abs(dx - dy)) * w * this->resolution;

    //!Euclidean distance
    return sqrt(pow(dy, 2) + pow(dx, 2)) * w * this->resolution;

    //!M-distance
//    if (dx > dy) std::swap(dx,dy);
//    if (3*dx < dy) return (dy - 3*dx + sqrt(10)*dx) * w * this->resolution;
//    else if (2*dx < dy) return (sqrt(10)*(dy - 2*dx) + sqrt(5)*(3*dx - dy)) * w * this->resolution;
//    else if (3*dx < 2*dy) return (sqrt(5)*(2*dy - 3*dx) + sqrt(13)*(2*dx - dy)) * w * this->resolution;
//    else return (sqrt(13)*(dy - dx) + sqrt(2)*(3*dx - 2*dy)) * w * this->resolution;

    return 0;
}

std::list<Node> SIPP_MP::findSuccessors(const Node curNode, const Map &map)
{
    Node newNode;
    std::list<Node> successors;
    std::vector<double> EAT;
    std::vector<SafeInterval> intervals;
    double h_value;
    auto parent = &(*closed.get<0>().find(boost::make_tuple(curNode.i, curNode.j, curNode.interval_id, curNode.angle_id, curNode.speed)));
    std::vector<Primitive> prims = primitives.getPrimitives(curNode.i, curNode.j, curNode.angle_id, curNode.speed, map);
    for(auto p:prims)
    {
        newNode = Node(curNode.i + p.target.i, curNode.j + p.target.j);
        newNode.angle_id = p.target.angle_id;
        newNode.speed = p.target.speed;
        newNode.primitive = p;
        newNode.heading = p.target.angle_id * this->angle_step;
        newNode.primitive.begin = curNode.g;
        newNode.primitive.setSize(curagent.size);
        newNode.primitive.setSource(curNode.i, curNode.j);
        newNode.g = curNode.g + p.duration;
        newNode.Parent = parent;
        h_value = getHValue(newNode.i, newNode.j, newNode.heading);

        if(newNode.i == curNode.i && newNode.j == curNode.j)
        {
            if(curNode.speed == 0 && curNode.interval.end > newNode.g)
            {
                newNode.interval = curNode.interval;
                newNode.interval_id = curNode.interval_id;
                newNode.angle_id = p.target.angle_id;
                newNode.F = newNode.g + h_value;
                if(closed.get<0>().find(boost::make_tuple(newNode.i, newNode.j, newNode.interval_id, newNode.angle_id, newNode.speed)) == closed.get<0>().end())
                    successors.push_front(newNode);
            }
        }
        else
        {
//            if(curNode.g < CN_MAX_OBSTACLE_TIME){
            intervals = constraints->findIntervals(newNode, EAT, closed, Open);
            for(unsigned int k = 0; k < intervals.size(); k++)
            {
                newNode.interval = intervals[k];
                newNode.interval_id = newNode.interval.id;
                newNode.g = EAT[k];
                newNode.F = newNode.g + h_value;
                newNode.angle_id = p.target.angle_id;
                successors.push_front(newNode);
            }
//            }else{
//                SafeInterval _interval(curNode.g, CN_INFINITY);
//                newNode.interval = _interval;
//                newNode.interval_id = 0;
//                newNode.F = newNode.g + h_value;
//                newNode.angle_id = p.target.angle_id;
//                successors.push_front(newNode);
//            }

        }
    }
    return successors;
}

SearchResult SIPP_MP:: startSearch(Map &map, Task &task, DynamicObstacles &obstacles)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    constraints = new Constraints(map.width, map.height);
    constraints->resetSafeIntervals(map.width, map.height);
    constraints->setObstacles(&obstacles);
    for(int k = 0; k < obstacles.getNumberOfObstacles(); k++){
        constraints->addConstraints(obstacles.getPrimitives(k), obstacles.getSize(k), obstacles.getMSpeed(k), map);

    }
    constraints->countCollisions();

    sresult.pathInfo.clear();
    sresult.pathInfo.resize(task.getNumberOfAgents());
    sresult.agents = task.getNumberOfAgents();
    sresult.agentsSolved = 0;
    sresult.flowtime = 0;
    sresult.makespan = 0;
    curagent = task.getAgent(0);
    constraints->setSize(4.25); //TODO
    findPath(0, map);

//    double timespent(0);
//    #ifdef __linux__
//        gettimeofday(&end, NULL);
//        timespent = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
//    #else
//        QueryPerformanceCounter(&end);
//        timespent = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
//    #endif
//    if(timespent > config->timelimit) TODO: MOVE THIS TO FIND PATH FUNCTION
//        break;

    #ifdef __linux__
        gettimeofday(&end, NULL);
        sresult.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
    #else
        QueryPerformanceCounter(&end);
        sresult.runtime = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
    #endif
    sresult.tries = 1;
    return sresult;
}

bool SIPP_MP::findPath(unsigned int numOfCurAgent, const Map &map)
{

    Open.clear();
    closed.clear();
    ResultPathInfo resultPath;
    openSize = 0;
    constraints->resetSafeIntervals(map.width, map.height);
    for(int i=0; i<map.height; i++)
        for(int j=0; j<map.width; j++)
            constraints->countSafeIntervals({i,j});
//            constraints->updateCellSafeIntervals({i,j});
//    constraints->updateCellSafeIntervals({curagent.start_i, curagent.start_j});

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    Node curNode(curagent.start_i, curagent.start_j, 0, 0), goalNode(curagent.goal_i, curagent.goal_j, CN_INFINITY, CN_INFINITY);
//    constraints->countSafeIntervals({curNode.i, curNode.j});
    curNode.interval = constraints->getSafeInterval(curNode.i, curNode.j, 0);
    curNode.interval_id = curNode.interval.id;
    curNode.heading = 0;
    curNode.heading = curagent.start_heading;
    curNode.F = getHValue(curNode.i, curNode.j, curNode.heading);
    curNode.angle_id = int(curNode.heading / this->angle_step);
    curNode.speed = 0;
    curNode.primitive.id = -1;
    curNode.primitive.source.angle_id = 0;
    curNode.primitive.target.angle_id = 0;
    Open.addOpen(curNode);
    bool goalOpened = false;

    while(!stopCriterion(curNode, goalNode) && !goalOpened)
    {
        curNode = Open.findMin();

        closed.insert(curNode);

        auto successors = findSuccessors(curNode, map);
        for(Node s:successors){
            Open.addOpen(s);
        }

    }
    std::cout << "Closed size: " << closed.size() << "\n";
    std::cout << "Opened size: " << Open.open.size() << "\n";
    std::cout << "Nodes expanded: " << closed.size() + Open.open.size() << "\n";

    if(goalNode.g < CN_INFINITY)
    {
        makePrimaryPath(goalNode);
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        makeSecondaryPath();
        std::cout<<"PATH FOUND\n";
        resultPath.points = point_path;
        resultPath.pathfound = true;
        resultPath.primitives = primitives_path;
        resultPath.pathlength = goalNode.g;
        resultPath.closed_size = closed.size();
        resultPath.open_size = Open.open.size();
        sresult.pathfound = true;
        sresult.flowtime += goalNode.g;
        sresult.makespan = std::max(sresult.makespan, goalNode.g);
        sresult.pathInfo[numOfCurAgent] = resultPath;
        sresult.agentsSolved++;
    }
    else
    {
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.runtime = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        std::cout<<"Path for agent "<<curagent.id<<" not found!\n";
        sresult.pathfound = false;
        resultPath.pathfound = false;
        resultPath.points.clear();
        resultPath.primitives.clear();
        resultPath.pathlength = 0;
        sresult.pathInfo[numOfCurAgent] = resultPath;
    }
    return resultPath.pathfound;
}

void SIPP_MP::makePrimaryPath(Node curNode)
{
    primitives_path.clear();
    std::vector<Node> path;
    path.push_back(curNode);
    if(curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != nullptr)
        {
            do
            {
                path.push_back(curNode);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != nullptr);
        }
        path.push_back(curNode);
    }
    std::reverse(path.begin(), path.end());

    for(unsigned int i = 1; i < path.size(); i++)
    {
        if(path[i].g - (path[i - 1].g + path[i].primitive.duration) > CN_RESOLUTION)
        {
            Node add = path[i - 1];
            add.Parent = path[i].Parent;
            add.g = path[i].g - path[i].primitive.duration;
            add.angle_id = path[i-1].angle_id;
            path.emplace(path.begin() + i, add);
            i++;
        }
    }
    for(int i = 1; i < path.size(); i++)
    {
        Node cur, next;
        cur = path[i-1];
        next = path[i];
        if(cur.i == next.i && cur.j == next.j && cur.angle_id == next.angle_id)
        {
            Primitive wait;
            wait.source.i = cur.i;
            wait.source.j = cur.j;
            wait.source.angle_id = cur.primitive.target.angle_id;
            wait.target.i = cur.i;
            wait.target.j = cur.j;
            wait.id = 0;
            wait.target.angle_id = cur.primitive.target.angle_id;
            wait.begin = cur.g;
            wait.duration = next.g - cur.g;
            wait.cells = {Cell(cur.i, cur.j)};
            wait.cells[0].interval = {0, wait.duration}; //not sure...
            wait.type = -2;
            path[i+1].primitive.begin = next.g;
            primitives_path.push_back(wait);

        }
        else
            primitives_path.push_back(next.primitive);
    }
    return;
}

void SIPP_MP::makeSecondaryPath()
{
    point_path.clear();
    int i(0);
    double t(0);
    while(i < primitives_path.size())
    {
        Primitive cur = primitives_path[i];
        while(t < cur.begin + cur.duration)
        {
            auto p = cur.getPos(t-cur.begin);
            p.i = cur.source.i+p.i;
            p.j = cur.source.j+p.j;
            double angle = cur.getAngle(t-cur.begin)*180/PI;
            if(cur.type == -2)//wait action
            {
                cur.id=0;
                angle = cur.source.angle_id*this->angle_step;
            }
            else if(cur.type == -1)//rotation action
            {
                angle = cur.source.angle_id*this->angle_step + (cur.target.angle_id - cur.source.angle_id)*this->angle_step*(t-cur.begin)/cur.duration;
            }
            TerminalPoint n(p.i, p.j, t, angle, cur.id);
            point_path.push_back(n);
            t+=0.1;
        }
        i++;
    }
    TerminalPoint n(curagent.goal_i, curagent.goal_j, primitives_path.back().begin + primitives_path.back().duration, 180 - primitives_path.back().target.angle_id*this->angle_step, primitives_path.back().id);
    point_path.push_back(n);
    return;
}