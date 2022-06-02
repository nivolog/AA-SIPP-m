#include "constraints.h"

Constraints::Constraints(int width, int height)
{
    safe_intervals.resize(height);
    collision_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        collision_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            collision_intervals[i][j].clear();
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
    constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        constraints[i].resize(width);
        for(int j = 0; j < width; j++)
            constraints[i][j].resize(0);
    }
    prim_id=0;
}

void Constraints::resetSafeIntervals(int width, int height)
{
    safe_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
}

void Constraints::updateCellSafeIntervals(std::pair<int, int> cell)
{
    std::vector<int> prim_ids = constraints[cell.first][cell.second];
    std::vector<Primitive> prims;
    for(int i:prim_ids)
        prims.push_back(obstacles->getPrimitive(i));
    for(int k = 0; k < prims.size(); k++)
    {
        Primitive prim = prims[k];
        double radius = agentsize;
        std::pair<double, double> p = prim.getInterval(cell.first, cell.second, radius);
        if(p.first < prim.begin)
            continue;
        SafeInterval interval(p.first, p.second);
        int i2(cell.first), j2(cell.second);
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            if(safe_intervals[i2][j2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][j].end + CN_EPSILON > interval.begin)
            {
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                    if(safe_intervals[i2][j2][j].end < interval.end)
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    else
                        safe_intervals[i2][j2][j].begin = interval.end;
                }
                else if(safe_intervals[i2][j2][j].end < interval.end)
                    safe_intervals[i2][j2][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[i2][j2][j].end;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            else if(safe_intervals[i2][j2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][j].begin < interval.end)
            {
                if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                    j++;
                }
                if(safe_intervals[i2][j2][j].end < interval.end)
                {
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                }
                else
                {
                    safe_intervals[i2][j2][j].begin = interval.end;
                }
            }
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
            safe_intervals[i2][j2][j].id = j;

        if(safe_intervals[i2][j2][0].begin > 0)
            collision_intervals[i2][j2].push_back({0, safe_intervals[i2][j2][0].begin});
        for(int i=0; i<safe_intervals[i2][j2].size()-1; i++)
            collision_intervals[i2][j2].push_back({safe_intervals[i2][j2][i].end, safe_intervals[i2][j2][i+1].begin});
        if(safe_intervals[i2][j2].back().end < CN_INFINITY)
            collision_intervals[i2][j2].push_back({safe_intervals[i2][j2].back().end, CN_INFINITY});

    }
}

void Constraints::countSafeIntervals(std::pair <int, int> cell)
{
    std::vector<std::pair<int, int>> cells = los->getCells(cell.first, cell.second);
    for(auto c: cells){
        if(c.first < 0 || c.second < 0 || c.first >= safe_intervals.size() || c.second >= safe_intervals[0].size())
            continue;
        auto collisions = collision_intervals[c.first][c.second];
        for(SafeInterval interval : collisions){
            if(interval.end - interval.begin < CN_EPSILON)
                continue;
            int i2(cell.first), j2(cell.second);
            for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
            {
                if(safe_intervals[i2][j2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][j].end + CN_EPSILON > interval.begin)
                {
                    if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                    {
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                        j++;
                        if(safe_intervals[i2][j2][j].end < interval.end)
                            safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                        else
                            safe_intervals[i2][j2][j].begin = interval.end;
                    }
                    else if(safe_intervals[i2][j2][j].end < interval.end)
                        safe_intervals[i2][j2][j].end = interval.begin;
                    else
                    {
                        std::pair<double,double> new1, new2;
                        new1.first = safe_intervals[i2][j2][j].begin;
                        new1.second = interval.begin;
                        new2.first = interval.end;
                        new2.second = safe_intervals[i2][j2][j].end;
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                        if(new2.first < CN_INFINITY)
                            safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new2.first, new2.second));
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(new1.first, new1.second));
                    }
                }
                else if(safe_intervals[i2][j2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][j].begin < interval.end)
                {
                    if(fabs(safe_intervals[i2][j2][j].begin - interval.begin) < CN_EPSILON)
                    {
                        safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, SafeInterval(safe_intervals[i2][j2][j].begin,safe_intervals[i2][j2][j].begin));
                        j++;
                    }
                    if(safe_intervals[i2][j2][j].end < interval.end)
                    {
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    }
                    else
                    {
                        safe_intervals[i2][j2][j].begin = interval.end;
                    }
                }
            }
            for(unsigned int j = 0; j < safe_intervals[i2][j2].size(); j++)
                safe_intervals[i2][j2][j].id = j;

        }
    }
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode, const ClosedList &close)
{
    std::vector<SafeInterval> intervals(0);
    for(unsigned int i = 0; i < safe_intervals[curNode.i][curNode.j].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][i].end >= curNode.g
                && safe_intervals[curNode.i][curNode.j][i].begin <= (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
        {
            auto has = close.get<0>().find(boost::make_tuple(curNode.i, curNode.j, safe_intervals[curNode.i][curNode.j][i].id, curNode.angle_id, curNode.speed));
            if(has == close.get<0>().end())
                intervals.push_back(safe_intervals[curNode.i][curNode.j][i]);
        }
    return intervals;
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.i][curNode.j];
}

void Constraints::addConstraints(const std::vector<Primitive> &primitives, double size, double mspeed, const Map &map)
{
    for(auto prim: primitives){
        for(auto c: prim.getCells()){
            if(c.i < constraints.size() && c.j < constraints[0].size()){
                constraints[c.i][c.j].push_back(prim.id);
                SafeInterval interval(c.interval.first, c.interval.second);
                collision_intervals[c.i][c.j].push_back(interval);
            }
        }
    }
}

void Constraints::countCollisions(){

    for(int i = 0; i < collision_intervals.size(); ++i){
        for(int j = 0; j < collision_intervals[i].size(); ++j){
            std::sort(collision_intervals[i][j].begin(), collision_intervals[i][j].end());
            if(collision_intervals[i][j].size() > 1)
                for(int k = 0; k < collision_intervals[i][j].size()-1; ++k){
                    if(collision_intervals[i][j][k].end > collision_intervals[i][j][k+1].begin){
                        if(collision_intervals[i][j][k].end < collision_intervals[i][j][k+1].end){
                            collision_intervals[i][j][k].end = collision_intervals[i][j][k+1].end;
                            collision_intervals[i][j].erase(collision_intervals[i][j].begin() + k + 1);
                            --k;
                        }else{
                            collision_intervals[i][j].erase(collision_intervals[i][j].begin() + k + 1);
                            --k;
                        }
                    }else if(fabs(collision_intervals[i][j][k].end - collision_intervals[i][j][k+1].begin) < CN_EPSILON){
                        collision_intervals[i][j][k].end = collision_intervals[i][j][k+1].end;
                        collision_intervals[i][j].erase(collision_intervals[i][j].begin() + k + 1);
                        --k;
                    }
                }
        }
    }
}

std::vector<SafeInterval> Constraints::findIntervals(Node curNode, std::vector<double> &EAT, const ClosedList &close, const OpenContainer &open)
{
    std::vector<SafeInterval> curNodeIntervals = getSafeIntervals(curNode, close);
    std::vector<SafeInterval> result;
    if(curNodeIntervals.empty())
    {
        return curNodeIntervals;
    }
    EAT.clear();
    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        SafeInterval cur_interval(curNodeIntervals[i]);
        curNode.interval = cur_interval;
        if(cur_interval.begin < curNode.g)
            cur_interval.begin = curNode.g;
        double startTime = curNode.Parent->g;
        if(cur_interval.begin > curNode.g)
        {
            if(curNode.Parent->speed > 0)
                continue;
            startTime = cur_interval.begin - curNode.primitive.duration;
        }
        curNode.interval = cur_interval;
        Node open_node = open.findNode(curNode);
        if(open_node.g - CN_EPSILON < startTime + curNode.primitive.duration)
            continue;
        double initTime(startTime);
        getEAT(curNode, startTime, open_node.g);
        if(startTime > curNode.Parent->interval.end || startTime + curNode.primitive.duration > cur_interval.end || (curNode.Parent->speed > 0 && startTime > initTime + CN_EPSILON) || startTime + curNode.primitive.duration > open_node.g)
            continue;
        EAT.push_back(startTime + curNode.primitive.duration);
        result.push_back(curNodeIntervals[i]);
    }
    return result;
}

void Constraints::getEAT(Node curNode, double& startTime, double open_node_g)
{
    auto cells = curNode.primitive.getCells();
    for(int k = 0; k < cells.size(); k++)
    {
        auto c = cells[k];
        std::pair<double, double> interval = {startTime + c.interval.first, startTime + c.interval.second};
        std::vector<SafeInterval> intervals = collision_intervals[c.i][c.j];
        for(int i = 0; i < intervals.size(); i++)
            if((interval.first <= intervals[i].begin && interval.second > intervals[i].begin) ||
               (interval.first >= intervals[i].begin && interval.first < intervals[i].end))
            {
                startTime = startTime - interval.first + intervals[i].end + CN_EPSILON;
                if(startTime > curNode.Parent->interval.end || startTime + curNode.primitive.duration > curNode.interval.end || curNode.Parent->speed > 0 || startTime + curNode.primitive.duration > open_node_g)
                    return;
                k=-1;
                break;
            }
    }
    return;
}
