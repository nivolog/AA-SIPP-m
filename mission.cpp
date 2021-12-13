#include"mission.h"

Mission::Mission()
{
    m_pSearch = nullptr;
    m_pLogger = nullptr;
}

Mission::~Mission()
{
    delete m_pSearch;
    delete m_pLogger;
}

void Mission::setFileNames(const char *taskName, const char *mapName, const char *configName, const char *obstaclesName, const char *primitivesName)
{
    this->taskName = taskName;
    this->mapName = mapName;
    this->configName = configName;
    this->obstaclesName = obstaclesName;
    this->primitivesName = primitivesName;
}

bool Mission::getMap()
{
    return m_map.getMap(mapName);
}

bool Mission::getTask()
{
    return (m_task.getTask(taskName) && m_task.validateTask(m_map));
}

bool Mission::getConfig()
{
    return m_config.getConfig(configName);
}

bool Mission::getObstacles()
{
    if(obstaclesName)
        return m_obstacles.getObstacles(obstaclesName);
    else
        return false;
}

void Mission::createSearch()
{
    if(m_pSearch)
    {
        delete m_pSearch;
        delete m_pLogger;
    }
    m_pSearch = new AA_SIPP(m_config, primitivesName);
}

void Mission::createLog()
{
    if(m_config.loglevel != CN_LOGLVL_NO)
    {
        m_pLogger = new XmlLogger(m_config.loglevel);
        m_pLogger->createLog(taskName);
    }
}

void Mission::startSearch()
{
    //std::cout<<"SEARCH STARTED\n";
    sr = m_pSearch->startSearch(m_map, m_task, m_obstacles);
}

void Mission::printSearchResultsToConsole()
{
    //std::ofstream out;
    //out.open("250obs.txt", std::ios::app);
    //out<<sr.runtime<<" "<<sr.pathInfo[0].runtime<<" "<<sr.flowtime<<"\n";
    //out.close();
    //std::cout<<bool(sr.agentsSolved/sr.agents)<<" "<<sr.time<<" "<<sr.makespan<<" "<<sr.pathlength<<" "<<sr.flowlength<<"\n";
    std::cout<<"Task solved: "<<bool(sr.agentsSolved/sr.agents)<<"\nTries: "<<sr.tries<<" Runtime: "<<sr.runtime<<" Agents solved: "<<sr.agentsSolved<<" ("<<(float)sr.agentsSolved*100/sr.agents<<"%) Flowtime: "<<sr.flowtime<<" Makespan: "<<sr.makespan<<"\n";
}

void Mission::saveSearchResultsToLog()
{
    if(m_config.loglevel == CN_LOGLVL_NO)
        return;
    //std::cout<<"LOG STARTED\n";
    m_pLogger->writeToLogInput(taskName, mapName, configName, obstaclesName);
    m_pLogger->writeToLogSummary(sr);
    if(sr.pathfound)
    {
        m_pLogger->writeToLogMap(m_map, sr);
        m_pLogger->writeToLogPath(sr, m_task, m_config);
    }
    m_pLogger->saveLog();
    //std::cout<<"LOG SAVED\n";
}

