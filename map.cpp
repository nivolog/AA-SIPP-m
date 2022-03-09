#include "map.h"
using namespace tinyxml2;

#define INF 1E20
#define LOW 1
#define HIGH 2.2360679775

Map::Map()
{
    height = 0;
    width = 0;
}
Map::~Map()
{	
    Grid.clear();
}

bool Map::getMap(const char* FileName)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = nullptr;
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML map-file."<<std::endl;
        return false;
    }

    XMLElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *grid = map->FirstChildElement(CNS_TAG_GRID);
    if (!grid)
    {
        std::cout << "No '"<<CNS_TAG_GRID<<"' element found in XML file."<<std::endl;
        return false;
    }
    height = grid->IntAttribute(CNS_TAG_ATTR_HEIGHT);
    if(height <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_HEIGHT<<" attribute. It should be >0.\n";
        return false;
    }
    width = grid->IntAttribute(CNS_TAG_ATTR_WIDTH);
    if(width <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_WIDTH<<" attribute. It should be >0.\n";
        return false;
    }
    XMLElement *row = grid->FirstChildElement(CNS_TAG_ROW);
    Grid.resize(height);
    for(int i = 0; i < height; i++)
        Grid[i].resize(width, 0);

    std::string value;
    const char* rowtext;
    std::stringstream stream;
    for(int i = 0; i < height; i++)
    {
        if (!row)
        {
            std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
            return false;
        }

        rowtext = row->GetText();
        unsigned int k = 0;
        value = "";
        int j = 0;

        for(k = 0; k < strlen(rowtext); k++)
        {
            if (rowtext[k] == ' ')
            {
                stream << value;
                stream >> Grid[i][j];
                stream.clear();
                stream.str("");
                value = "";
                j++;
            }
            else
            {
                value += rowtext[k];
            }
        }
        stream << value;
        stream >> Grid[i][j];
        stream.clear();
        stream.str("");

        if (j < width-1)
        {
            std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given." << std::endl;
            return false;
        }
        row = row->NextSiblingElement(CNS_TAG_ROW);
    }


    distances.resize(height);
    for(int i = 0; i < height; i++)
        distances[i].resize(width, 0);

    computeDistances();

    GridLow = Grid;
    GridHigh = Grid;
    for (int i = 0; i < height; ++i)
        for (int j = 0; j < width; ++j)
        {
            GridLow[i][j] = distances[i][j] < LOW ? 1 : 0;
            GridHigh[i][j] = distances[i][j] < HIGH ? 1 : 0;
        }

    return true;
}


bool Map::CellIsTraversable(int i, int j) const
{
    return (Grid[i][j] == 0);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != 0);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

int Map::getValue(int i, int j) const
{
    if(i < 0 || i >= height)
        return -1;
    if(j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}


/* dt of 1d function using euclidean distance */
float * Map::dt1(float *f, int n)
{
    float *d = new float[n];
    int *v = new int[n];
    float *z = new float[n+1];
    int k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = +INF;
    for (int q = 1; q <= n-1; q++)
    {
        float s  = ((f[q] + q * q)-(f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        while (s <= z[k]) {
            k--;
            s  = ((f[q] + q * q)-(f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k+1] = +INF;
    }

    k = 0;
    for (int q = 0; q <= n-1; q++) {
        while (z[k+1] < q)
            k++;
        d[q] = (q-v[k]) * (q-v[k]) + f[v[k]];
    }

    delete [] v;
    delete [] z;
    return d;
}

/* dt of 2d function using euclidean distance */
void Map::dt(float *image)
{

    float *f = new float[std::max(width, height)];

    // transform along columns
    for (int j = 0; j < width; ++j)
    {
        for (int i = 0; i < height; ++i)
        {
            f[i] = image[j + i * width];
        }

        float *d = dt1(f, height);

        for (int i = 0; i < height; ++i) {
            image[j + i * width] = d[i];
        }
        delete [] d;
    }

    // transform along rows
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            f[j] = image[j + i * width];
        }
        float *d = dt1(f, width);
        for (int j = 0; j < width; ++j)
        {
            image[j + i * width] = d[j];
        }
        delete [] d;
    }

    delete f;
}

void Map::computeDistances()
{
    /* dt of binary image using squared distance */
    float *image = new float[width * height];
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (Grid[i][j])
                image[j + i * width] = 0;
            else
                image[j + i * width] = INF;
        }
    }

    dt(image);



    for (int i = 0 ; i < height; ++i)
    {
        for (int j = 0 ; j < width; ++j)
        {
//            std::cout << "Debug\n";
            distances[i][j] = sqrt(image[j + i * width]);
        }
    }
    delete image;
}

double Map::getDistance(int i, int j) const{
    return distances[i][j];
}

bool Map::CellCloseToObst(int i, int j) const{
    return bool(GridLow[i][j]);
}


bool Map::CellFarFromObst(int i, int j) const{
    return (GridHigh[i][j] == 0);
}