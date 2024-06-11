#ifndef DASTS2_VERSION9_C_CONFIG_H
#define DASTS2_VERSION9_C_CONFIG_H

#include <string>
class Config
{
public:
    std::string ws;
    std::string TaburesultFolder;
    std::string MultiLevelresultFolder;
    std::string dataPath;
    std::string dataName;
    std::string dataType;
    int NumRunPerDataSet;
    bool multiData;
    int addEjectionType;
    int ejectionIte;
    bool overwrite;
    int run_type;


    double droneVelocity;
    double techVelocity;
    int numDrone;
    int numTech;
    double droneLimitationFlightTime;
    double sampleLimitationWaitingTime;


    bool use_ejection;
    bool use_inter;
    bool use_intra;
    int num_level;
    double percent_match;
    double percent_select;
    int tabuMaxIter;
    int tabu_size;
    bool isCycle;
    int tabuDuration;


    
    int tabuNumRunPerDataSet;
    int tabuNotImproveIter;
    int minTabuDuration;
    int maxTabuDuration;
    double tabuAlpha1;
    double tabuAlpha2;
    double tabuBeta;
    double tabuEpsilon;
    int maxEjectionLevel;

    Config();
};
#endif