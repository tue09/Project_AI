#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>

#include "nlohmann/json.hpp"
#include "src/Config.h"
#include "src/Input.h"
#include "src/Solution.h"
#include "src/tabuSearch/TabuSearch.h"
#include "src/multiLevel/MultiLevel.h"
#include "src/Utils.h"
using namespace std::chrono;
using json = nlohmann::json;

std::vector<std::string> file_browsing(std::string folderPath, std::string numcus){
    std::vector<std::string> file_browse;
    for (const auto& ite : std::filesystem::directory_iterator(folderPath)) {
        std::string number;
        for (int i = 0; i < ite.path().filename().string().size(); i++){
            if (ite.path().filename().string()[i] == '.') break;
            number = number + ite.path().filename().string()[i];
        }
        if (ite.is_regular_file() && number == numcus){
            file_browse.push_back(ite.path().filename().string());
        }
    }
    return file_browse;
}


int main(int argc, char **argv)
{    
    Config config;
    std::vector<std::map<int, std::vector<int>>> mapLevel;
    std::vector<std::vector<std::vector<double>>> DistanceMatrixLevel;
    std::vector<std::vector<bool>> C1Level;
    std::string configFilePath = "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/Config.json";
    std::ifstream configFile(configFilePath);
    json jsConfig = json::parse(configFile);

    config.droneVelocity = jsConfig["parameter"]["droneVelocity"].get<double>();
    config.techVelocity = jsConfig["parameter"]["techVelocity"].get<double>();
    config.numDrone = jsConfig["parameter"]["numDrone"].get<int>();
    config.numTech = jsConfig["parameter"]["numTech"].get<int>();
    config.droneLimitationFlightTime = jsConfig["parameter"]["droneLimitationFlightTime"].get<double>();
    config.sampleLimitationWaitingTime = jsConfig["parameter"]["sampleLimitationWaitingTime"].get<double>();
    config.use_ejection = jsConfig["parameter"]["use_ejection"].get<bool>();
    config.use_inter = jsConfig["parameter"]["use_inter"].get<bool>();
    config.use_intra = jsConfig["parameter"]["use_intra"].get<bool>();
  
    config.num_level = jsConfig["multiLevel_para"]["num_level"].get<int>();
    config.percent_match = jsConfig["multiLevel_para"]["percent_match"].get<double>();
    config.tabu_max_ite = jsConfig["multiLevel_para"]["tabu_max_ite"].get<int>();
    config.tabu_size = jsConfig["multiLevel_para"]["tabu_size"].get<int>();
    config.tabuMaxIter = jsConfig["multiLevel_para"]["tabu_max_ite"].get<int>();
//    config.tabuNumRunPerDataSet = jsConfig["tabu_para"]["tabuNumRunPerDataSet"].get<int>();
    config.tabuNumRunPerDataSet = 1;
    config.tabuNotImproveIter = jsConfig["tabu_para"]["tabuNotImproveIter"].get<int>();
    config.tabuAlpha1 = jsConfig["tabu_para"]["tabuAlpha1"].get<double>();
    config.tabuAlpha2 = jsConfig["tabu_para"]["tabuAlpha2"].get<double>();
    config.tabuBeta = jsConfig["tabu_para"]["tabuBeta"].get<double>();
    config.tabuEpsilon = jsConfig["tabu_para"]["tabuEpsilon"].get<double>();
    config.maxEjectionLevel = jsConfig["tabu_para"]["maxEjectionLevel"].get<int>();
    config.minTabuDuration = jsConfig["tabu_para"]["minTabuDuration"].get<int>();
    config.maxTabuDuration = jsConfig["tabu_para"]["maxTabuDuration"].get<int>();
    config.ws = jsConfig["ws"].get<std::string>();;
    config.dataName = jsConfig["dataName"].get<std::string>();
    config.dataPath = jsConfig["dataPath"].get<std::string>();
    config.TaburesultFolder = jsConfig["TaburesultFolder"].get<std::string>();
    config.MultiLevelresultFolder = jsConfig["MultiLevelresultFolder"].get<std::string>();
    config.NumRunPerDataSet = jsConfig["NumRunPerDataSet"].get<int>();
    config.multiData = jsConfig["multiData"].get<bool>();
    config.dataType = jsConfig["dataType"].get<std::string>();
    std::vector<std::string> dataList = file_browsing(config.ws + config.dataPath, config.dataType);
    if (config.multiData == false)
    {
        dataList.resize(0);
        std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, config.dataName);
        for (const std::string &path : paths)
        {
            Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
            dataList.push_back(input.dataSet + ".txt");
        }
    }
    //dataList = {"100.10.1.txt", "100.20.1.txt", "100.30.1.txt", "100.40.1.txt"};
    //dataList = {"50.10.1.txt", "50.20.1.txt", "50.30.1.txt", "50.40.1.txt"};
    json logDataSet;
    for (int ite = 0; ite < dataList.size(); ite++)
    {
        std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, dataList[ite]);
        json logData;
        for (const std::string &path : paths)
        {
            std::cout << path << std::endl;
            Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
            config.tabuMaxIter = config.tabu_max_ite;
            double best = 9999999;
            double average = 0;
            std::vector<double> acc;
            for (int run = 0; run < config.NumRunPerDataSet; run++)
            {
                Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
                std::cout << "Run set: " << path << "." << run << std::endl; 
                TabuSearch tabuSearch(config, input);
                if (tabuSearch.initSolution.droneTripList.empty() && tabuSearch.initSolution.techTripList.empty())
                    {
                        std::cout << "Infeasible!" << std::endl;
                        return 1;
                    }
                
                json log;
                log["Num_Drone: "] = config.numDrone;
                log["Num_Tech: "] = config.numTech;
                
                std::string path_e;
                MultiLevel multilev(config, input);        
                auto begin = high_resolution_clock::now();
                std::vector<double> score_list;
                std::tuple<double, Solution, std::vector<double>> result = multilev.run(config, input);
                score_list = std::get<2> (result);
                std::vector<double> Listt;
                std::string Scoree = "Level 0: ";
                Listt.push_back(score_list[0]);
                Scoree = Scoree + std::to_string(score_list[0]) + " | ";
                for (int i = 1; i <= config.num_level + 1; i++){
                    Listt.push_back((score_list[i] - score_list[i-1]) / score_list[i-1]);
                    if (i == config.num_level + 1){
                        Scoree = Scoree + "Post Optimization: " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                    }else{
                        Scoree = Scoree + "Level " + std::to_string(i) + ": " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                    }
                }
                for (int i = config.num_level + 2; i < score_list.size(); i++){
                    Listt.push_back((score_list[i] - score_list[i-1]) / score_list[i-1]);
                    if (i == score_list.size() - 1){
                        Scoree = Scoree + "Post Optimization: " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                    }else{
                        Scoree = Scoree + "Level " + std::to_string(score_list.size() - i - 2 ) + ": " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                    }
                }
                Solution bestSol = std::get<1> (result);
                auto end = high_resolution_clock::now();
                json jDroneBest(bestSol.droneTripList);
                json jTechBest(bestSol.techTripList);
                if (std::get<0> (result) < best)
                {
                    best = std::get<0> (result);
                }
                acc.push_back(std::get<0> (result));
                average += std::get<0> (result);
                log["Best Score: "] = std::get<0> (result);
                std::cout<<"Best score: " <<std::get<0> (result);
                log["Best Solution: "] = std::to_string(std::get<0> (result)) + " == " + jDroneBest.dump() + " || " + jTechBest.dump();
                log["Run Time: "] = duration_cast<milliseconds> (end - begin).count() / 1000.0;
                log["Score List: "] = Scoree;
                std::string name = dataList[ite] + "." + std::to_string(run + 1);
                logData[name] = log;
            }
            double std = 0;
            average = average / config.NumRunPerDataSet;
            if (config.NumRunPerDataSet == 1){
                std = 0;
            }else{
                for (int h = 0; h < acc.size(); h++){
                    std = std + pow(acc[h] - average, 2);
                }
                std = std/(config.NumRunPerDataSet - 1);
                std = sqrt(std);
            }
            std::cout<<std::endl<<"Dataset is: "<<input.dataSet;
            logDataSet[dataList[ite]] = logData;
            std::string tempp = input.dataSet + " | Best: ";
            std::string tempu = input.dataSet + " | Average: ";
            std::string tempv = input.dataSet + " | STD: ";
            logDataSet[tempp] = best;
            logDataSet[tempu] = average;
            logDataSet[tempv] = std;
        }
    }
    std::ofstream o (config.MultiLevelresultFolder + config.dataType + "_" + std::to_string(int(config.percent_match*100)) + "%" + ".json");
    o << std::setw(4) << logDataSet << std::endl;
    o.close();
    return 0;
}