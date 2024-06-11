#include "Config.h"

Config::Config()
{
    
    tabuMaxIter = 500;
    tabuNumRunPerDataSet = 5;
    tabuNotImproveIter = 200;
    tabuAlpha1 = 1;
    tabuAlpha2 = 1;
    tabuBeta = 0.5;
    tabuEpsilon = 1e-3;
    maxEjectionLevel = 2;
    addEjectionType = 1;
    overwrite = true;
    run_type = 1;

    tabu_size = 5;
    num_level = 3;
    percent_match = 0.2;


    droneVelocity = 0.83;
    techVelocity = 0.58;
    numDrone = 2;
    numTech = 1;
    droneLimitationFlightTime = 120;
    sampleLimitationWaitingTime = 60;
    isCycle = 1;
    ejectionIte = 1;
    tabuMaxIter = 100;
    tabuDuration = 5;


    use_ejection = true;
    use_inter = true;
    use_intra = true;

    NumRunPerDataSet = 5;
    ws = "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C";
    TaburesultFolder = "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/tabu_result";
    MultiLevelresultFolder = "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/multilevel_result";
    dataPath = "/data";
    dataName = "6.5.1.txt";
    multiData = true;
    dataType = "6";
}