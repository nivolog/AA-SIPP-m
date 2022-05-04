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
    void countIntervals(double size, double max_time = CN_INFINITY)
    {
        double r = size + agentSize;
        double end = duration > max_time ? max_time : duration;
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
        for(int i = -num-1; i <= +num+1; i++)
            for(int j = -num-1; j <= +num+1; j++)
                if((pow(abs(i) - 0.5, 2) + pow(abs(j) - 0.5, 2)) < pow(r, 2)){
                    c.first = i;
                    c.second = j;
                    if(std::find(circleCells.begin(), circleCells.end(), c) == circleCells.end())
                        circleCells.push_back(c);
                }
        return circleCells;
    }
    void countLastCells(){
        auto circleCells = getCircleCells(agentSize);
        for(auto c : circleCells){
            Cell cell(source.i + c.first, source.j + c.second);
            cell.interval.first = begin;
            cell.interval.second = CN_INFINITY;
            if(std::find(cells.begin(), cells.end(), cell) == cells.end())
                cells.push_back(cell);
        }
//        while(r <= agentSize){
//            angle = 0;
//            while(angle < 2*PI){
//                c_i = source.i + int(cos(angle)*(r + 0.5 - 1e-3));
//                c_j = source.j + int(sin(angle)*(r + 0.5 - 1e-3));
//                Cell c(c_i, c_j);
//                c.interval.first = begin;
//                c.interval.second = CN_INFINITY;
//                if(std::find(cells.begin(), cells.end(), c) == cells.end())
//                    cells.push_back(c);
//
//                angle+=0.1;
//            }
//            r += 0.1;
//        }
    }
    void countCells()
    {
        double t(0);
        bool stop(false);
//        while(t < duration + CN_EPSILON)
//        {
//            double angle = getAngle(t);
//            double gap_i = cos(angle)*agentSize;
//            double gap_j = sin(angle)*agentSize;
//            auto p = getPos(t);
//            int c_i(p.i+gap_i+0.5-1e-3);
//            if(p.i+gap_i < 0)
//                c_i = p.i+gap_i-0.5+1e-3;
//            int c_j(p.j-gap_j+0.5-1e-3);
//            if(p.j-gap_j < 0)
//                c_j = p.j-gap_j-0.5+1e-3;
//            Cell c(c_i, c_j);
//            if(std::find(cells.begin(), cells.end(), c) == cells.end())
//                cells.push_back(c);
//
//            c_i = p.i-gap_i+0.5-1e-3;
//            if(p.i-gap_i < 0)
//                c_i = p.i-gap_i-0.5+1e-3;
//            c_j = p.j+gap_j+0.5-1e-3;
//            if(p.j+gap_j < 0)
//                c_j = p.j+gap_j-0.5+1e-3;
//            c = Cell(c_i, c_j);
//            if(std::find(cells.begin(), cells.end(), c) == cells.end())
//                cells.push_back(c);
//
//            c_i = p.i+0.5-1e-3;
//            if(p.i < 0)
//                c_i = p.i-0.5+1e-3;
//            c_j = p.j+0.5-1e-3;
//            if(p.j < 0)
//                c_j = p.j-0.5+1e-3;
//            c = Cell(c_i, c_j);
//            if(std::find(cells.begin(), cells.end(), c) == cells.end())
//                cells.push_back(c);
//
//            if(t + 0.01 >= duration)
//            {
//                if(stop)
//                    break;
//                t = duration;
//                stop = true;
//            }
//            else
//                t += 0.01;
//        }
        while(t < duration + CN_EPSILON)
        {
            auto p = getPos(t);
            auto circleCells = getCircleCells(agentSize);
            for(auto c : circleCells){
                Cell cell(p.i + c.first, p.j + c.second);
                if(std::find(cells.begin(), cells.end(), cell) == cells.end())
                    cells.push_back(cell);
            }
            if(t + 0.01 >= duration)
            {
                if(stop)
                    break;
                t = duration;
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

        N3.push_back(std::pair<int, int>(0, 1));
        N3.push_back(std::pair<int, int>(1, 3));
        N3.push_back(std::pair<int, int>(1, 2));
        N3.push_back(std::pair<int, int>(2, 3));
        N3.push_back(std::pair<int, int>(1, 1));
        N3.push_back(std::pair<int, int>(3, 2));
        N3.push_back(std::pair<int, int>(2, 1));
        N3.push_back(std::pair<int, int>(3, 1));
        N3.push_back(std::pair<int, int>(1, 0));


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
                            c.interval.first = cell->DoubleAttribute("begin");
                            c.interval.second = cell->DoubleAttribute("end");
                            prim.cells.push_back(c);
                        }
                }

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

                if (prim.cells.size() == 0) prim.countCells();

//                std::cout << "ID loaded: " << prim.id  << "\tTarget: " << prim.target.j << " " << prim.target.i  << "\tSource: " << " " << prim.source.angle_id<< "\n";
//                for(auto cell : prim.cellsCenter){
//                    std::cout << "Cell: " << cell << "\n";
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
//            for(tinyxml2::XMLElement  *turning = elem->FirstChildElement();turning; turning = turning->NextSiblingElement("time_finish"))
//            {
//                    Primitive prim;
//                    prim.type = -1;
//                    prim.id = turning->IntAttribute("id");
//                    prim.source.i = prim.source.j = prim.target.i = prim.target.j = prim.source.speed = prim.target.speed = 0;
//                    prim.source.angle_id = int(turning->IntAttribute("phi0")/this->angle_step);
//                    prim.target.angle_id = int(turning->IntAttribute("phif")/this->angle_step);
//                    prim.duration = turning->DoubleAttribute("Tf");
//                    prim.agentSize = 0.5;
//                    prim.cells = {Cell(0,0)};
//                    prim.cells[0].interval = {0, prim.duration};
//                    type0.back().push_back(prim);
//                }
//            }
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

    Primitive getPrimitiveEndpoint(int i, int j)
    {
//        std::cout << "Attemptimg to find primitive to " << Cell(i,j) << "\n";
        for(auto p:type0)
            for(auto t:p)
                if(t.target.i == i && t.target.j == j){
//                    std::cout << "Found primitive " << t.id << "\n";
                    return t;
                }
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

//            for (auto c:prims[k].getCells())
//                if (!map.CellOnGrid(i + c.i, j + c.j) || map.CellIsObstacle(i + c.i, j + c.j)) {
//                    prims.erase(prims.begin() + k);
//                    k--;
//                    break;
//                }
        }
        return prims;
    }


    bool isCanonical(Primitive a, Primitive b){
        if(((a.id - 1 + 2)/4)%2 == 1){
            int id_lower_bound = 3; //We leave id = {1,2} for two basic orthogonal moves along one axis
            int id_upper_bound = type0[0].size() - 2; // And same thing along other axis at the end of moves list

            //For move to be canonical we basically need to check if it is a neighbor of main move
            //For each move in positive quadrant we have three additional variants on each part of plane
            //Therefore we have to look for moves, which ids differentiates from main move's id for 4 in each direction
            int id_prev = a.id - 4 < id_lower_bound ? ((a.id - 4) + 1) / 2 + 1 : a.id - 4; //At the beginning we have 2 variants of move, instead of 4
            int id_next = a.id + 4 > id_upper_bound ? ((a.id + 4) - id_upper_bound - 1) / 2 + id_upper_bound + 1 : a.id + 4; // Same thing at the end
            return b.id == id_prev || b.id == id_next || b.id == a.id;
        }else
            return b.id == a.id;
    }


    std::vector<Primitive> canonicalOrdering_8connected(int i, int j, int parents_prim_id, const Map& map){
        std::vector<Primitive> prims, successors;
        Primitive parents_prim = getPrimitive(parents_prim_id);
        for(Primitive p:type0[0])
            if(isCanonical(parents_prim, p)){
                prims.push_back(p);
            }

        if(parents_prim_id == -1)
            std::cout << "For primitive " << parents_prim_id << " there are " << prims.size() << " canonical successors\n";

        if(parents_prim.target.i * parents_prim.target.j != 0){//If move is diagonal
            if(!map.CellOnGrid(i - parents_prim.target.i, j) || map.CellIsObstacle(i - parents_prim.target.i, j))
                prims.push_back(getPrimitiveEndpoint(-parents_prim.target.i, parents_prim.target.j));
            if(!map.CellOnGrid(i, j - parents_prim.target.j) || map.CellIsObstacle(i, j - parents_prim.target.j))
                prims.push_back(getPrimitiveEndpoint(parents_prim.target.i, -parents_prim.target.j));
        }else if(parents_prim.target.i * parents_prim.target.j == 0){//If move is orthogonal
            if(!map.CellOnGrid(i + parents_prim.target.j, j + parents_prim.target.i) || map.CellIsObstacle(i + parents_prim.target.j, j + parents_prim.target.i))
                prims.push_back(getPrimitiveEndpoint(parents_prim.target.i + parents_prim.target.j, parents_prim.target.i + parents_prim.target.j));
            if(!map.CellOnGrid(i - parents_prim.target.j, j - parents_prim.target.i) || map.CellIsObstacle(i - parents_prim.target.j, j - parents_prim.target.i))
                prims.push_back(getPrimitiveEndpoint(parents_prim.target.i - parents_prim.target.j, parents_prim.target.j - parents_prim.target.i));
        }

        for(Primitive p:prims){
//            successors.push_back(p);
//
            if(checkPrimitiveCenter(i,j,p,map))
                successors.push_back(p);
        }
        return successors;
    }

    std::vector<Primitive> canonicalOrdering(int i, int j, int parents_prim_id, const Map& map){
        std::vector<Primitive> prims, successors;
        int canons = 0;
        Primitive parents_prim = getPrimitive(parents_prim_id);
        for(Primitive p:type0[0])
            if(isCanonical(parents_prim, p)){
                prims.push_back(p);
                ++canons;
//                std::cout << "Adding to primitive " << parents_prim.id << " canonical successor " << p.id << "\n";
            }else if(!checkPrimitive(i-parents_prim.target.i, j-parents_prim.target.j, p, map)){
//                std::cout << i << " " << j << "\n";
//                std::cout << parents_prim.target.i << " " << parents_prim.target.j << "\n";
//                std::cout << p.target.i << " " << p.target.j << "\n";
//                std::cout << "Cell " << Cell(i-parents_prim.target.i+p.target.i, j-parents_prim.target.j+p.target.j) << " is not allowed\n";
                prims.push_back(p);
            }

        for(Primitive p:prims){
            if(checkPrimitive(i,j,p,map))
                successors.push_back(p);
        }
//        std::cout << "Using canonical ordering I just removed " << type0[0].size() - successors.size() << " primitives\n" << canons << " from them are canonical\n";
        return successors;
    }

    std::vector<Primitive> getAllPrimitives(){
        return type0[0];
    }
};



#endif
