#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include "tinyxml2.h"
#include "map.h"

//int wrap(int x, int lower, int upper){
//    int range = upper - lower + 1;
//    x = ((x-lower) % range);
//    if (x<0)
//        return upper + 1 + x;
//    else
//        return lower + x;
//}

class Position
{
    public:
    int i;
    int j;
    int angle_id;
    int speed;
};

class Cell
{
    public:
    Cell(int i_, int j_):i(i_), j(j_){}
    int i;
    int j;
    std::pair<double, double> interval;
    bool operator <(const Cell& other) const
    {
        if(i == other.i)
            return j < other.j;
        return i < other.i;
    }
    bool operator ==(const Cell& other) const
    {
        if(i == other.i && j == other.j)
            return true;
        else
            return false;
    }

    friend std::ostream& operator<< (std::ostream& stream, const Cell& cell) {
        stream << "(" << cell.j << ", " << cell.i << ")";
        return stream;
    }
};

class Point {
    public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
    double minDist(Point C, Point D)
    {
        int classA = this->classify(C, D);
        if(classA == 3)
            return sqrt(pow(this->i - C.i, 2) + pow(this->j - C.j, 2));
        else if(classA == 4)
            return sqrt(pow(this->i - D.i, 2) + pow(this->j - D.j, 2));
        else
            return fabs((C.i - D.i)*this->j + (D.j - C.j)*this->i + (C.j*D.i - D.j*C.i))/sqrt(pow(C.i - D.i, 2) + pow(C.j - D.j, 2));
    }
};

class Primitive
{
public:
    int id;
    int type;
    double begin;
    double duration;
    Position source;
    Position target;
    double agentSize;
    std::vector<double> i_coefficients;
    std::vector<double> j_coefficients;
    std::vector<Cell> cells;
    std::vector<Cell> cellsCenter;
    bool operator==(const Primitive& other) { return this->id == other.id; }
    void setSize(double size) { agentSize = size; }
    std::vector<Cell> getCells() { return cells; }
    std::vector<Cell> getCellsCenter() { return cellsCenter; }
    double size() {return agentSize;}
    void setSource(int i, int j)
    {
        source.i = i;
        source.j = j;
        target.i = i + target.i;
        target.j = j + target.j;
        for(auto &c:cells)
        {
            c.i = i + c.i;
            c.j = j + c.j;
        }
    }
    double getEndpoint(Cell cell, double begin, double end, double resolution, double size, bool start)
    {
        if(resolution < CN_RESOLUTION)
        {
            if(start && begin + resolution*10 > duration - CN_EPSILON)
                return -1;
            else
                return start?begin:begin+resolution;
        }
        double cur_t(begin);
        while(true)
        {
            Point pos = this->getPos(cur_t);
            double dist = pow(pos.i - cell.i, 2) + pow(pos.j - cell.j, 2);
            if((start && dist < pow(size + agentSize,2)) || (!start && dist > pow(size + agentSize,2)))
            {
                if(cur_t - resolution < 0)
                    return 0;
                return getEndpoint(cell, cur_t - resolution, std::min(cur_t, duration), resolution/10, size, start);
            }
            cur_t += resolution;
            if(cur_t > end - CN_EPSILON)
            {
                if(resolution > CN_RESOLUTION)
                    return getEndpoint(cell, cur_t - resolution, std::min(cur_t, duration), resolution/10, size, start);
                else
                    return start?-1:end;
            }
        }
    }
    std::pair<double, double> getInterval(int i, int j, double size)
    {
        auto it = std::find(cells.begin(), cells.end(), Cell(i,j));
        if(it == cells.end())
            return {-1,-1};
        else if(it->interval.first < 0)
            return {-1,-1};
//        return {begin + it->interval.first, begin + it->interval.second};
        return {it->interval.first, it->interval.second};
    }

    std::pair<double, double> getIntervalCount(int i, int j, double size, double max_time = CN_INFINITY)
    {
        Cell c(i,j);
        double end = duration > max_time ? max_time : duration;
        std::pair<double, double> interval;
        if(duration == CN_INFINITY){
            interval.first = begin;
            interval.second = CN_INFINITY;
            return interval;
        }else{
            interval.first = getEndpoint(c, 0, end, 0.01, size, true);
            interval.second = getEndpoint(c, interval.first + CN_RESOLUTION*2, end, 0.01, size, false);
            if(interval.first > interval.second)
                interval.first = -1;
            return interval;
        }
    }

    double getAngle(double t)
    {
        if(type < 0)
            return -1;
        if(j_coefficients[1]+2*j_coefficients[2]*t+3*j_coefficients[3]*t*t < CN_EPSILON && fabs(i_coefficients[2]) < CN_EPSILON)
        {
            return PI - i_coefficients[1]*PI/2;
        }
        return atan2(i_coefficients[1]+2*i_coefficients[2]*t + 3*i_coefficients[3]*t*t,j_coefficients[1]+2*j_coefficients[2]*t+3*j_coefficients[3]*t*t);
    }
    Point getPos(double t)
    {
        if(type < 0)
            return Point(0,0);
        Point p;
        p.j = j_coefficients[0] + j_coefficients[1]*t + j_coefficients[2]*t*t + j_coefficients[3]*t*t*t;
        p.i = i_coefficients[0] + i_coefficients[1]*t + i_coefficients[2]*t*t + i_coefficients[3]*t*t*t;
        return p;
    }
    void countIntervals(double size)
    {
        double r = size + agentSize;
        double end = duration; // > CN_MAX_OBSTACLE_TIME ? CN_MAX_OBSTACLE_TIME : duration;
        for(auto &c:cells)
        {
            c.interval.first = getEndpoint(c, 0, end, 0.01, size, true);
            if(c.interval.first < 0)
                continue;
            c.interval.second = getEndpoint(c, c.interval.first + CN_RESOLUTION*2, end, 0.01, size, false);
            if(c.interval.first > c.interval.second)
                c.interval.first = -1;
        }
    }
    std::vector<std::pair<int, int>> getCircleCells(double r){
        std::vector<std::pair<int, int>> circleCells;
        std::pair<int, int> c;
        int num = r + 0.5 - CN_EPSILON;
        for(int i = -num; i <= +num; i++)
            for(int j = -num; j <= +num; j++)
                if((pow((abs(i) > 0 ? abs(i) - 0.5 : 0), 2) + pow((abs(j) > 0 ? abs(j) - 0.5 : 0), 2)) < pow(r, 2)){
                    c.first = i;
                    c.second = j;
                    if(std::find(circleCells.begin(), circleCells.end(), c) == circleCells.end())
                        circleCells.push_back(c);
                }
        return circleCells;
    }
    void countLastCells(double inflation = 0){
        auto circleCells = getCircleCells(agentSize + inflation);
        double endtime = CN_INFINITY;
        for(auto c : circleCells){
            Cell cell(source.i + c.first, source.j + c.second);
            cell.interval.first = begin;
            cell.interval.second = endtime;
            if(std::find(cells.begin(), cells.end(), cell) == cells.end())
                cells.push_back(cell);
        }
    }
    void countCells(double inflation = 0)
    {
        double t(begin);
        bool stop(false);
        double endtime = begin + duration;
        while(t < endtime + CN_EPSILON)
        {
            auto p = getPos(t);
            auto circleCells = getCircleCells(agentSize+inflation);
            for(auto c : circleCells){
                Cell cell(p.i + c.first, p.j + c.second);
                auto it = std::find(cells.begin(), cells.end(), cell);
                if(it == cells.end()){
                    cell.interval.first = t;
                    cell.interval.second = t;
                    cells.push_back(cell);
                }else{
                    it->interval.second = t;
                }
            }
            if(t + 0.01 >= endtime)
            {
                if(stop)
                    break;
                t = endtime;
                stop = true;
            }
            else
                t += 0.01;
        }
    }

    bool isSafe(int i, int j, const Map& map){
        for (auto c : cellsCenter){
            if (!map.CellFarFromObst(i+c.i, j+c.j)) return false;
        }
        return true;
    }

    bool hasCollision(int i, int j, const Map& map){
        for (auto c : cellsCenter)
            if (map.CellOnGrid(i+c.i, j+c.j))
            {if (map.CellCloseToObst(i+c.i, j+c.j)) return true;}
            else
            {return true;}

        return false;
    }
};

class Primitives
{
    public:
    double resolution = 0.2;
    double angle_step = 45.0;
    double max_velocity = 1.0;
    double avg_velocity = 1.0;
    std::vector<std::pair <int, int> > N3;
    std::vector<std::vector<Primitive>> type0;
    std::vector<std::vector<Primitive>> type1;
    bool loadPrimitives(const char* FileName, double resolution)
    {
        this->resolution = resolution;
        std::string value;
        std::stringstream stream;

        tinyxml2::XMLDocument doc;
        if(doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
        {
            std::cout << "Error openning input XML file."<<std::endl;
            return false;
        }

        tinyxml2::XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
        if (!root)
        {
            std::cout << "No 'root' element found in XML file."<<std::endl;
            return false;
        }

        /*
         * Getting config for primitives
         * Max and average velocity are essential for heuristic function in the planner
         * Angle step is the same, but also performs the way primitives being stored in containers*/
        tinyxml2::XMLElement *config = root->FirstChildElement("config");
        if (!config)
        {
            std::cout << "No config found in primitives! Using default values\n";
        }else{
            if (config->DoubleAttribute("angle_step") > CN_EPSILON)
                this->angle_step = config->DoubleAttribute("angle_step");
            if (config->DoubleAttribute("max_velocity") > CN_EPSILON)
                this->max_velocity = config->DoubleAttribute("max_velocity");
            if (config->DoubleAttribute("avg_velocity") > CN_EPSILON)
                this->avg_velocity = config->DoubleAttribute("avg_velocity");
        }

        int id=0;
        for(tinyxml2::XMLElement  *elem = root->FirstChildElement();elem;elem = elem->NextSiblingElement("trajectory"))
        {
            for(tinyxml2::XMLElement  *coef = elem->FirstChildElement();coef; coef = coef->NextSiblingElement("coeff"))
            {
                Primitive prim;
                prim.id = coef->IntAttribute("id");
                id++;
                prim.type = coef->IntAttribute("v0");

                prim.duration = coef->DoubleAttribute("Tf");
                prim.agentSize = 0.5;

                prim.source.i = 0;
                prim.source.j = 0;

                prim.source.angle_id = int(coef->DoubleAttribute("phi0")/this->angle_step);
                prim.source.speed = coef->IntAttribute("v0");

                prim.target.i = int(std::round(coef->DoubleAttribute("yf") / this->resolution));
                prim.target.j = int(std::round(coef->DoubleAttribute("xf") / this->resolution));
                prim.target.angle_id = int(coef->DoubleAttribute("phif")/this->angle_step);
                prim.target.speed = coef->IntAttribute("vf");

                prim.i_coefficients.push_back(coef->DoubleAttribute("b1"));
                prim.i_coefficients.push_back(coef->DoubleAttribute("b2"));
                prim.i_coefficients.push_back(coef->DoubleAttribute("b3"));
                prim.i_coefficients.push_back(coef->DoubleAttribute("b4"));

                prim.j_coefficients.push_back(coef->DoubleAttribute("a1"));
                prim.j_coefficients.push_back(coef->DoubleAttribute("a2"));
                prim.j_coefficients.push_back(coef->DoubleAttribute("a3"));
                prim.j_coefficients.push_back(coef->DoubleAttribute("a4"));
                /* Getting precomputed sweeping cells, that primitive covers
                 * If there is no cells, we compute them */

                 for (tinyxml2::XMLElement *sweeping_cells = coef->FirstChildElement(); sweeping_cells; sweeping_cells = sweeping_cells->NextSiblingElement("sweeping_cells")) {
                    if (sweeping_cells->DoubleAttribute("resolution") == resolution)
                        for (tinyxml2::XMLElement *cell = sweeping_cells->FirstChildElement(); cell; cell = cell->NextSiblingElement("cell")) {
                            int j = cell->IntAttribute("x");
                            int i = cell->IntAttribute("y");
                            Cell c(i, j);
                            c.interval.first = cell->DoubleAttribute("begin");
                            c.interval.second = cell->DoubleAttribute("end");
                            prim.cells.push_back(c);
                        }
                }
                if (prim.cells.size() == 0) prim.countCells();
                /* The same thing goes with center cells
                 */
                for (tinyxml2::XMLElement *sweeping_cells = coef->FirstChildElement(); sweeping_cells; sweeping_cells = sweeping_cells->NextSiblingElement("sweeping_cells_center")) {
                    if (sweeping_cells->DoubleAttribute("resolution") == resolution)
                        for (tinyxml2::XMLElement *cell = sweeping_cells->FirstChildElement(); cell; cell = cell->NextSiblingElement("cell")) {
                            int j = cell->IntAttribute("x");
                            int i = cell->IntAttribute("y");
                            double begin = cell->DoubleAttribute("begin");
                            double end = cell->DoubleAttribute("end");
                            Cell c(i, j);
                            c.interval.first = begin;
                            c.interval.second = end;
                            prim.cellsCenter.push_back(c);
                        }
                }

//                prim.countIntervals(0.5);
                if(prim.type == 0)
                {
                    if(type0.empty() || type0.back().at(0).source.angle_id != prim.source.angle_id)
                        type0.push_back({prim});
                    else
                        type0.back().push_back(prim);
                }
                else
                {
                    if(type1.empty() || type1.back().at(0).source.angle_id != prim.source.angle_id)
                        type1.push_back({prim});
                    else
                        type1.back().push_back(prim);
                }
            }
        }
    }
    Primitive getPrimitive(int id)
    {
        for(auto p:type0)
            for(auto t:p)
                if(t.id == id)
                    return t;
        for(auto p:type1)
            for(auto t:p)
                if(t.id == id)
                    return t;
    }

    bool checkPrimitive(int i, int j, Primitive prim, const Map& map){
        for (auto c:prim.getCells())
            if (!map.CellOnGrid(i + c.i, j + c.j) || map.CellIsObstacle(i + c.i, j + c.j)) {
                return false;
            }
        return true;
    }

    bool checkPrimitiveCenter(int i, int j, Primitive prim, const Map& map){
        for (auto c:prim.getCellsCenter())
            if (!map.CellOnGrid(i + c.i, j + c.j) || map.CellIsObstacle(i + c.i, j + c.j)) {
                return false;
            }
        return true;
    }

    std::vector<Primitive> getPrimitives(int i, int j, int angle_id, int speed, const Map& map)
    {
        std::vector<Primitive> prims, res;
        if(speed == 1)
            prims = type1[angle_id];
        else
            prims = type0[angle_id];

        for(int k = 0; k < prims.size(); k++){
            if (!checkPrimitive(i, j, prims[k], map)){
                prims.erase(prims.begin() + k);
                k--;
            }
        }
        return prims;
    }

};



#endif
