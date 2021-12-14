#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include "tinyxml2.h"
#include "map.h"

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
    bool operator==(const Primitive& other) { return this->id == other.id; }
    void setSize(double size) { agentSize = size; }
    std::vector<Cell> getCells() { return cells; }
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
        return {begin + it->interval.first, begin + it->interval.second};
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
        for(auto &c:cells)
        {
            c.interval.first = getEndpoint(c, 0, duration, 0.01, size, true);
            if(c.interval.first < 0)
                continue;
            c.interval.second = getEndpoint(c, c.interval.first + CN_RESOLUTION*2, duration, 0.01, size, false);
            if(c.interval.first > c.interval.second)
                c.interval.first = -1;
        }
    }
    void countCells()
    {
        double t(0);
        bool stop(false);
        while(t < duration + CN_EPSILON)
        {
            double angle = getAngle(t);
            double gap_i = cos(angle)*0.5;
            double gap_j = sin(angle)*0.5;
            auto p = getPos(t);
            int c_i(p.i+gap_i+0.5-1e-3);
            if(p.i+gap_i < 0)
                c_i = p.i+gap_i-0.5+1e-3;
            int c_j(p.j-gap_j+0.5-1e-3);
            if(p.j-gap_j < 0)
                c_j = p.j-gap_j-0.5+1e-3;
            Cell c(c_i, c_j);
            if(std::find(cells.begin(), cells.end(), c) == cells.end())
                cells.push_back(c);

            c_i = p.i-gap_i+0.5-1e-3;
            if(p.i-gap_i < 0)
                c_i = p.i-gap_i-0.5+1e-3;
            c_j = p.j+gap_j+0.5-1e-3;
            if(p.j+gap_j < 0)
                c_j = p.j+gap_j-0.5+1e-3;
            c = Cell(c_i, c_j);
            if(std::find(cells.begin(), cells.end(), c) == cells.end())
                cells.push_back(c);

            c_i = p.i+0.5-1e-3;
            if(p.i < 0)
                c_i = p.i-0.5+1e-3;
            c_j = p.j+0.5-1e-3;
            if(p.j < 0)
                c_j = p.j-0.5+1e-3;
            c = Cell(c_i, c_j);
            if(std::find(cells.begin(), cells.end(), c) == cells.end())
                cells.push_back(c);

            if(t + 0.1 >= duration)
            {
                if(stop)
                    break;
                t = duration;
                stop = true;
            }
            else
                t += 0.1;
        }
    }
};

class Primitives
{
    public:
    double resolution = 0.2;
    double angle_step = 45.0;
    double max_velocity = 1.0;
    double avg_velocity = 1.0;

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
        std::cout << "Max velocity: " << this->max_velocity << "\tAvg velocity: " << this->avg_velocity<< "\tAngle step: " << this->angle_step << std::endl;

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

                for (tinyxml2::XMLElement *sweeping_cells = coef->FirstChildElement(); sweeping_cells; sweeping_cells = sweeping_cells->NextSiblingElement("sweeping_cells")) {
                    if (sweeping_cells->DoubleAttribute("resolution") == resolution)
                        for (tinyxml2::XMLElement *cell = sweeping_cells->FirstChildElement(); cell; cell = cell->NextSiblingElement("cell")) {
                            int j = cell->IntAttribute("x");
                            int i = cell->IntAttribute("y");
                            Cell c(i, j);
                            c.interval.first = 0;
                            c.interval.second = CN_INFINITY;
                            prim.cells.push_back(c);
                        }
                }

                if (prim.cells.size() == 0) prim.countCells();
//                std::cout << "ID loaded: " << prim.id  << "\tTarget: " << prim.target.j << " " << prim.target.i  << "\tSource: " << " " << prim.source.angle_id<< "\n";
//                for(auto cell : prim.cells){
//                    std::cout << "Cell: " << cell.i << " " << cell.j << "\n";
//                }
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
            for(tinyxml2::XMLElement  *turning = elem->FirstChildElement();turning; turning = turning->NextSiblingElement("time_finish"))
            {
                Primitive prim;
                prim.type = -1;
                prim.id = turning->IntAttribute("id");
                prim.source.i = prim.source.j = prim.target.i = prim.target.j = prim.source.speed = prim.target.speed = 0;
                prim.source.angle_id = int(turning->IntAttribute("phi0")/this->angle_step);
                prim.target.angle_id = int(turning->IntAttribute("phif")/this->angle_step);
                prim.duration = turning->DoubleAttribute("Tf");
                prim.agentSize = 0.5;
                prim.cells = {Cell(0,0)};
                prim.cells[0].interval = {0, prim.duration};
                type0.back().push_back(prim);
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
    std::vector<Primitive> getPrimitives(int i, int j, int angle_id, int speed, const Map& map)
    {
        std::vector<Primitive> prims, res;
        if(speed == 1)
            prims = type1[angle_id];
        else
            prims = type0[angle_id];
        for(int k = 0; k < prims.size(); k++)
            for(auto c:prims[k].getCells())
                if(c.interval.first > -CN_EPSILON){
                    if(!map.CellOnGrid(i+c.i,j+c.j) || map.CellIsObstacle(i+c.i, j+c.j))
                    {
//                        std::cout << "Rejected primitive number " << prims[k].id << " because of cell in " << j+c.j << " " << i+c.i << "\n";
                        prims.erase(prims.begin() + k);
                        k--;
                        break;
                    }
//                    std::cout << "Primitive " << prims[k].id << " .Cell in " << j+c.j << " " << i+c.i << " is okay\n";
                }

        }

        return prims;
    }
};
#endif
