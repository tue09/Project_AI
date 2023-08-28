#include "MultiLevel.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Utils.h"
#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Random.h"

using Random = effolkronium::random_static;


MultiLevel::MultiLevel(Config &config, Input &input)
{
    this->config = config;
    this->config.tabuMaxIter = config.tabu_max_ite;
    this->input = input;
    this->tabuDuration = Random::get(config.minTabuDuration, config.maxTabuDuration);
}

std::vector<std::vector<double>> MultiLevel::convertMatrix(std::vector<std::vector<double>> currentMatrix, std::map<int, std::vector<int>> Map)
{

    std::vector<std::vector<double>> newMatrix;
    int sizee = Map.size();
    std::vector<double> temp;
    for (int i = 0; i <= sizee; i++)
    {
        temp.resize(sizee + 1, 0);
        newMatrix.push_back(temp);
    }
    std::vector<int> plus;
    for (int i = 0; i < newMatrix.size() - 1; i++)
    {
        for (int j = 0; j < newMatrix.size() - 1; j++)
        {
            plus = Map[i];
            plus.insert(plus.end(), Map[j].begin(), Map[j].end());
            if (Map[j].size() > 1)
            {
                int sizee = Map[j].size();
                for (int k = 0; k < sizee - 1; k++)
                {
                    plus.pop_back();
                }
            }
            for (int k = 0; k < plus.size() - 1; k++)
            {
                newMatrix[i][j] = newMatrix[i][j] + currentMatrix[plus[k]][plus[k+1]];
            }
        }
    }
    for (int i = 0; i < newMatrix.size(); i++)
    {
        newMatrix[i][newMatrix.size() - 1] = newMatrix[i][0];
        newMatrix[newMatrix.size() - 1][i] = newMatrix[0][i];
    }  
    return newMatrix;
}

std::tuple<int, Solution, std::map<int, std::vector<int>>> MultiLevel::mergeSol(Solution solution, int NumCus, std::vector<std::vector<int>> mainMatrix, std::vector<std::vector<double>> distanceMatrix, std::vector<bool> C1)
{
    //make update for matrix
    std::vector<std::tuple<int , int>> update_Real;
    
    std::vector<std::tuple<int , int>> update;
    std::vector<std::tuple<int , int>> edgeSol;
    
    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        if (solution.droneTripList[i].size() == 0) {continue;}
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            if (solution.droneTripList[i][j].size() <= 1) {continue;}
            for (int k = 0; k < solution.droneTripList[i][j].size() - 1; k++)
            {
                edgeSol.push_back(std::make_tuple(solution.droneTripList[i][j][k], solution.droneTripList[i][j][k+1]));
            }
        }
    }   

    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        if (solution.techTripList[i].size() <= 1) {continue;}
        for (int j = 0; j < solution.techTripList[i].size() - 1; j++)
        {
            edgeSol.push_back(std::make_tuple(solution.techTripList[i][j], solution.techTripList[i][j+1]));
        }
    }

    // Select 5% shortest edge from Solution
    int numUpdate = NumCus*0.05;
    std::vector<int> ro;
    std::vector<int> co;
    while (update.size() < numUpdate)
    {
        double count = 9999999;
        int row = 1;
        int col = 1;

        for (int i = 1; i <= NumCus; i++)
        {
            for (int j = 1; j <= NumCus; j++)
            {
                auto find1 = std::find(update.begin(), update.end(), std::make_tuple(i, j));
                auto find2 = std::find(edgeSol.begin(), edgeSol.end(), std::make_tuple(i, j));
                auto find3 = std::find(ro.begin(), ro.end(), i);
                auto find4 = std::find(co.begin(), co.end(), j);
                if ((distanceMatrix[i][j] <= count) and (find1 == update.end()) and (find2 == edgeSol.end()) and (i != j) and (find3 == ro.end()) and (find4 == co.end()))
                {
                    count = distanceMatrix[i][j];
                    row = i;
                    col = j;
                }
            }
        }
        ro.push_back(row);
        co.push_back(col);
        update.push_back(std::make_tuple(row, col));
    }

    for (int l = 0; l < update.size(); l++)
    {
        Solution solbefore = solution;
        int row = std::get<0> (update[l]);
        int col = std::get<1> (update[l]);
        int x1, x2, x3;
        int y1, y2, y3;
        for (int i = 0; i < solution.droneTripList.size(); i++)
        {
            if (solution.droneTripList[i].size() == 0) {continue;}
            for (int j = 0; j < solution.droneTripList[i].size(); j++)
            {
                if (solution.droneTripList[i][j].size() <= 0) {continue;}
                for (int k = 0; k < solution.droneTripList[i][j].size(); k++)
                {
                    if (solution.droneTripList[i][j][k] == row)
                    {
                        x1 = i;
                        x2 = j;
                        x3 = k;
                    }
                    if (solution.droneTripList[i][j][k] == col)
                    {
                        y1 = i;
                        y2 = j;
                        y3 = k;
                    }
                }
            }
        }   

        for (int i = 0; i < solution.techTripList.size(); i++)
        {
            if (solution.techTripList[i].size() <= 0) {continue;}
            for (int j = 0; j < solution.techTripList[i].size(); j++)
            {
                if (solution.techTripList[i][j] == row)
                {
                    x1 = i;
                    x2 = j;
                    x3 = -1;
                }
                if (solution.techTripList[i][j] == col)
                {
                    y1 = i;
                    y2 = j;
                    y3 = -1;
                }
            }
        }
        //if ((x1 > NumCus) or (x1 <= 0))
        if ((x3 != -1) and (y3 != -1))
        {
            //std::cout<<std::endl<<"CASE 1";
            if ((x1 == y1) and (x2 == y2))
            {          
                //std::cout<<std::endl<<"CASE 1.1";   
                if (x3 == solution.droneTripList[x1][x2].size() - 1)
                {
                    //std::cout<<std::endl<<"CASE 1.1.1";   
                    Solution solutionx = solution;
                    int temp = solutionx.droneTripList[y1][y2][y3];
                    solutionx.droneTripList[y1][y2].erase(solutionx.droneTripList[y1][y2].begin() + y3);
                    solutionx.droneTripList[x1][x2].insert(solutionx.droneTripList[x1][x2].end(), temp);
                    Solution solutiony = solutionx;
                    auto result = std::find(solutiony.droneTripList[x1][x2].begin(), solutiony.droneTripList[x1][x2].end(), row);
                    int index = 0;
                    if (result != solutiony.droneTripList[x1][x2].end()) {
                        index = std::distance(solutiony.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solutiony.droneTripList[x1][x2][index] = col;
                    solutiony.droneTripList[x1][x2][index + 1] = row;

                    std::vector<Solution> check;
                    if (solutionx.check_feasible()){
                        check.insert(check.end() ,solutionx);
                    }
                    if (solutiony.check_feasible()){
                        check.insert(check.end() ,solutiony);
                    }
                    if (check.size() == 0){
                        
                    }else{
                        Solution fisol;
                        double ress = 999999;
                        for (int i = 0; i < check.size(); i++){
                            double hh = check[i].getScore();
                            if (hh < ress){
                                ress = hh;
                                fisol = check[i];
                            }
                        }
                        solution = fisol;
                    }
                }
                else
                {
                    //std::cout<<std::endl<<"CASE 1.1.2";   
                    Solution solutionx = solution;
                    int temp = solutionx.droneTripList[x1][x2][x3 + 1];
                    solutionx.droneTripList[x1][x2][x3 + 1] = solutionx.droneTripList[y1][y2][y3];
                    solutionx.droneTripList[y1][y2][y3] = temp;
                    Solution solutiony = solutionx;
                    auto result = std::find(solutiony.droneTripList[x1][x2].begin(), solutiony.droneTripList[x1][x2].end(), row);
                    int index = 0;
                    if (result != solutiony.droneTripList[x1][x2].end()) {
                        index = std::distance(solutiony.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solutiony.droneTripList[x1][x2][index] = col;
                    solutiony.droneTripList[x1][x2][index + 1] = row;
                    std::vector<Solution> check;
                    if (solutionx.check_feasible()){
                        check.insert(check.end() ,solutionx);
                    }
                    if (solutiony.check_feasible()){
                        check.insert(check.end() ,solutiony);
                    }
                    if (check.size() == 0){
                        
                    }else{
                        Solution fisol;
                        double ress = 999999;
                        for (int i = 0; i < check.size(); i++){
                            double hh = check[i].getScore();
                            if (hh < ress){
                                ress = hh;
                                fisol = check[i];
                            }
                        }
                        solution = fisol;
                    }
                }                
            }
            else
            {
                //std::cout<<std::endl<<"CASE 1.2";   
                std::vector<int> temp1;
                std::vector<int> temp2;
                Solution solCase1 = solution;
                Solution solCase2 = solution;
                Solution solCase3 = solution;
                Solution solCase1x;
                Solution solCase2x;
                Solution solCase3x;
                if (x3 + 1 >= solution.droneTripList[x1][x2].size())
                {
                    //std::cout<<std::endl<<"CASE 1.2.1";   
                    //Case 1:
                    for (int i = y3; i < solCase1.droneTripList[y1][y2].size(); i++)
                    {
                        temp2.push_back(solCase1.droneTripList[y1][y2][i]);
                    }
                    solCase1.droneTripList[y1][y2].erase(solCase1.droneTripList[y1][y2].begin() + y3, solCase1.droneTripList[y1][y2].end());
                    solCase1.droneTripList[x1][x2].insert(solCase1.droneTripList[x1][x2].end(), temp2.begin(), temp2.end());
                    solCase1x = solCase1;

                    auto result = std::find(solCase1x.droneTripList[x1][x2].begin(), solCase1x.droneTripList[x1][x2].end(), row);
                    int index = 0;
                    if (result != solCase1x.droneTripList[x1][x2].end()) {
                        index = std::distance(solCase1x.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase1x.droneTripList[x1][x2][index] = col;
                    solCase1x.droneTripList[x1][x2][index + 1] = row;
                    //Case 2:
                    int temp22 = solCase2.droneTripList[y1][y2][y3];
                    solCase2.droneTripList[y1][y2].erase(solCase2.droneTripList[y1][y2].begin() + y3);
                    solCase2.droneTripList[x1][x2].insert(solCase2.droneTripList[x1][x2].end(), temp22);
                    solCase2x = solCase2;

                    result = std::find(solCase2x.droneTripList[x1][x2].begin(), solCase2x.droneTripList[x1][x2].end(), row);
                    index = 0;
                    if (result != solCase2x.droneTripList[x1][x2].end()) {
                        index = std::distance(solCase2x.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase2x.droneTripList[x1][x2][index] = col;
                    solCase2x.droneTripList[x1][x2][index + 1] = row;
                    //Case 3: 
                    int temp3 = solCase3.droneTripList[y1][y2][y3];
                    solCase3.droneTripList[y1][y2].erase(solCase3.droneTripList[y1][y2].begin() + y3);
                    solCase3.droneTripList[x1][x2].insert(solCase3.droneTripList[x1][x2].end(), temp3);
                    solCase3x = solCase3;
                    
                    result = std::find(solCase3x.droneTripList[x1][x2].begin(), solCase3x.droneTripList[x1][x2].end(), row);
                    index = 0;
                    if (result != solCase3x.droneTripList[x1][x2].end()) {
                        index = std::distance(solCase3x.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase3x.droneTripList[x1][x2][index] = col;
                    solCase3x.droneTripList[x1][x2][index + 1] = row;

                }
                else
                {
                    //std::cout<<std::endl<<"CASE 1.2.2";   
                    //Case 1:
                    for (int i = x3 + 1; i < solCase1.droneTripList[x1][x2].size(); i++)
                    {
                        temp1.push_back(solCase1.droneTripList[x1][x2][i]);
                    }
                    solCase1.droneTripList[x1][x2].erase(solCase1.droneTripList[x1][x2].begin() + x3 + 1, solCase1.droneTripList[x1][x2].end());
                    for (int i = y3; i < solCase1.droneTripList[y1][y2].size(); i++)
                    {
                        temp2.push_back(solCase1.droneTripList[y1][y2][i]);
                    }
                    solCase1.droneTripList[y1][y2].erase(solCase1.droneTripList[y1][y2].begin() + y3, solCase1.droneTripList[y1][y2].end());

                    solCase1.droneTripList[x1][x2].insert(solCase1.droneTripList[x1][x2].end(), temp2.begin(), temp2.end());
                    solCase1.droneTripList[y1][y2].insert(solCase1.droneTripList[y1][y2].end(), temp1.begin(), temp1.end());
                    
                    solCase1x = solCase1;
                    auto result = std::find(solCase1x.droneTripList[x1][x2].begin(), solCase1x.droneTripList[x1][x2].end(), row);
                    int index = 0;
                    if (result != solCase1x.droneTripList[x1][x2].end()) {
                        index = std::distance(solCase1x.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase1x.droneTripList[x1][x2][index] = col;
                    solCase1x.droneTripList[x1][x2][index + 1] = row;
                    //Case 2:
                    int temp22 = solCase2.droneTripList[y1][y2][y3];
                    solCase2.droneTripList[y1][y2][y3] = solCase2.droneTripList[x1][x2][x3+1];
                    solCase2.droneTripList[x1][x2][x3+1] = temp22;
                    
                    solCase2x = solCase2;
                    result = std::find(solCase2x.droneTripList[x1][x2].begin(), solCase2x.droneTripList[x1][x2].end(), row);
                    index = 0;
                    if (result != solCase2x.droneTripList[x1][x2].end()) {
                        index = std::distance(solCase2x.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase2x.droneTripList[x1][x2][index] = col;
                    solCase2x.droneTripList[x1][x2][index + 1] = row;
                    //Case 3:

                    int temp3 = solCase3.droneTripList[y1][y2][y3];
                    solCase3.droneTripList[y1][y2].erase(solCase3.droneTripList[y1][y2].begin() + y3);
                    solCase3.droneTripList[x1][x2].insert(solCase3.droneTripList[x1][x2].begin() + x3, temp3);
                    
                    solCase3x = solCase3;
                    result = std::find(solCase3x.droneTripList[x1][x2].begin(), solCase3x.droneTripList[x1][x2].end(), row);
                    index = 0;
                    if (result != solCase3x.droneTripList[x1][x2].end()) {
                        index = std::distance(solCase3x.droneTripList[x1][x2].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase3x.droneTripList[x1][x2][index] = col;
                    solCase3x.droneTripList[x1][x2][index + 1] = row;
                }
                std::vector<Solution> check;
                if (solCase1.check_feasible()){
                    check.insert(check.end() ,solCase1);
                }
                if(solCase1x.check_feasible()){
                    check.insert(check.end() ,solCase1x);
                }
                if (solCase2.check_feasible()){
                    check.insert(check.end() ,solCase2);
                }
                if(solCase2x.check_feasible()){
                    check.insert(check.end() ,solCase2x);
                }
                if (solCase3.check_feasible()){
                    check.insert(check.end() ,solCase2);
                }
                if(solCase3x.check_feasible()){
                    check.insert(check.end() ,solCase3x);
                }
                
                if (check.size() == 0){
                    
                }else{
                    Solution fisol;
                    double ress = 999999;
                    for (int i = 0; i < check.size(); i++){
                        double hh = check[i].getScore();
                        if (hh < ress){
                            ress = hh;
                            fisol = check[i];
                        }
                    }
                    solution = fisol;
                }
            }
            //std::cout<<std::endl<<"CASE 1 OUT";
        }
        else if ((x3 != -1) and (y3 == -1))
        {
            //std::cout<<std::endl<<"CASE 2";
            if (y2 < solution.techTripList[y1].size())
            {
                bool flag = false;
                for (int i = y2; i < solution.techTripList[y1].size(); i++)
                {
                    if (C1[solution.techTripList[y1][i]] == true) 
                    {
                        flag = true;
                        break;
                    }
                }
                if (flag == true) {continue;}
            }
            std::vector<int> temp1;
            std::vector<int> temp2;
            Solution solCase1 = solution;
            Solution solCase2 = solution;
            Solution solCase3 = solution;
            Solution solCase1x;
            Solution solCase2x;
            Solution solCase3x;
            if (x3 + 1 >= solution.droneTripList[x1][x2].size()) 
            {
                //Case 1:
                for (int i = y2; i < solCase1.techTripList[y1].size(); i++)
                {
                    temp2.push_back(solCase1.techTripList[y1][i]);
                }
                solCase1.techTripList[y1].erase(solCase1.techTripList[y1].begin() + y2, solCase1.techTripList[y1].end());
                solCase1.droneTripList[x1][x2].insert(solCase1.droneTripList[x1][x2].end(), temp2.begin(), temp2.end());
                
                solCase1x = solCase1;
                auto result = std::find(solCase1x.droneTripList[x1][x2].begin(), solCase1x.droneTripList[x1][x2].end(), row);
                int index = 0;
                if (result != solCase1x.droneTripList[x1][x2].end()) {
                    index = std::distance(solCase1x.droneTripList[x1][x2].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase1x.droneTripList[x1][x2][index] = col;
                solCase1x.droneTripList[x1][x2][index + 1] = row;

                //Case 2:
                int temp22 = solCase2.techTripList[y1][y2];
                solCase2.techTripList[y1].erase(solCase2.techTripList[y1].begin() + y2);
                solCase2.droneTripList[x1][x2].insert(solCase2.droneTripList[x1][x2].end(), temp22);
                
                solCase2x = solCase2;
                result = std::find(solCase2x.droneTripList[x1][x2].begin(), solCase2x.droneTripList[x1][x2].end(), row);
                index = 0;
                if (result != solCase2x.droneTripList[x1][x2].end()) {
                    index = std::distance(solCase2x.droneTripList[x1][x2].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase2x.droneTripList[x1][x2][index] = col;
                solCase2x.droneTripList[x1][x2][index + 1] = row;

                //Case 3: 
                int temp3 = solCase3.techTripList[y1][y2];
                solCase3.techTripList[y1].erase(solCase3.techTripList[y1].begin() + y2);
                solCase3.droneTripList[x1][x2].insert(solCase3.droneTripList[x1][x2].end(), temp3);
                
                solCase3x = solCase3;
                result = std::find(solCase3x.droneTripList[x1][x2].begin(), solCase3x.droneTripList[x1][x2].end(), row);
                index = 0;
                if (result != solCase3x.droneTripList[x1][x2].end()) {
                    index = std::distance(solCase3x.droneTripList[x1][x2].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase3x.droneTripList[x1][x2][index] = col;
                solCase3x.droneTripList[x1][x2][index + 1] = row;
            }
            else
            {
                
                //Case 1:
                for (int i = x3 + 1; i < solCase1.droneTripList[x1][x2].size(); i++)
                {
                    temp1.push_back(solCase1.droneTripList[x1][x2][i]);
                }
                solCase1.droneTripList[x1][x2].erase(solCase1.droneTripList[x1][x2].begin() + x3 + 1, solCase1.droneTripList[x1][x2].end());

                for (int i = y2; i < solCase1.techTripList[y1].size(); i++)
                {
                    temp2.push_back(solCase1.techTripList[y1][i]);
                }
                solCase1.techTripList[y1].erase(solCase1.techTripList[y1].begin() + y2, solCase1.techTripList[y1].end());

                solCase1.droneTripList[x1][x2].insert(solCase1.droneTripList[x1][x2].end(), temp2.begin(), temp2.end());
                solCase1.techTripList[y1].insert(solCase1.techTripList[y1].end(), temp1.begin(), temp1.end());
                
                solCase1x = solCase1;
                auto result = std::find(solCase1x.droneTripList[x1][x2].begin(), solCase1x.droneTripList[x1][x2].end(), row);
                int index = 0;
                if (result != solCase1x.droneTripList[x1][x2].end()) {
                    index = std::distance(solCase1x.droneTripList[x1][x2].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase1x.droneTripList[x1][x2][index] = col;
                solCase1x.droneTripList[x1][x2][index + 1] = row;
                //Case 2:
                int temp22 = solCase2.techTripList[y1][y2];
                solCase2.techTripList[y1][y2] = solCase2.droneTripList[x1][x2][x3+1];
                solCase2.droneTripList[x1][x2][x3+1] = temp22;
                
                solCase2x = solCase2;
                result = std::find(solCase2x.droneTripList[x1][x2].begin(), solCase2x.droneTripList[x1][x2].end(), row);
                index = 0;
                if (result != solCase2x.droneTripList[x1][x2].end()) {
                    index = std::distance(solCase2x.droneTripList[x1][x2].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase2x.droneTripList[x1][x2][index] = col;
                solCase2x.droneTripList[x1][x2][index + 1] = row;

                //Case 3:
                int temp3 = solCase3.techTripList[y1][y2];
                solCase3.techTripList[y1].erase(solCase3.techTripList[y1].begin() + y2);
                solCase3.droneTripList[x1][x2].insert(solCase3.droneTripList[x1][x2].begin() + x3, temp3);
                
                solCase3x = solCase3;
                result = std::find(solCase3x.droneTripList[x1][x2].begin(), solCase3x.droneTripList[x1][x2].end(), row);
                index = 0;
                if (result != solCase3x.droneTripList[x1][x2].end()) {
                    index = std::distance(solCase3x.droneTripList[x1][x2].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase3x.droneTripList[x1][x2][index] = col;
                solCase3x.droneTripList[x1][x2][index + 1] = row;
            }
            std::vector<Solution> check;
            if (solCase1.check_feasible()){
                check.insert(check.end() ,solCase1);
            }
            if(solCase1x.check_feasible()){
                    check.insert(check.end() ,solCase1x);
            }
            if (solCase2.check_feasible()){
                check.insert(check.end() ,solCase2);
            }
            if(solCase2x.check_feasible()){
                    check.insert(check.end() ,solCase2x);
            }
            if (solCase3.check_feasible()){
                check.insert(check.end() ,solCase3);
            }
            if(solCase3x.check_feasible()){
                    check.insert(check.end() ,solCase3x);
            }

            if (check.size() == 0){
                    
            }else{
                Solution fisol;
                double ress = 999999;
                for (int i = 0; i < check.size(); i++){
                    double hh = check[i].getScore();
                    if (hh < ress){
                        ress = hh;
                        fisol = check[i];
                    }
                }
                solution = fisol;
            }
            //std::cout<<std::endl<<"CASE 2 OUT";
        }
        else if ((x3 == -1) and (y3 != -1))
        {
            //std::cout<<std::endl<<"CASE 3";
            if (x2 < solution.techTripList[x1].size())
            {
                bool flag = false;
                for (int i = x2; i < solution.techTripList[x1].size(); i++)
                {
                    if (C1[solution.techTripList[x1][i]] == true) 
                    {
                        flag = true;
                        break;
                    }
                }
                if (flag == true) {continue;}
            }
            std::vector<int> temp1;
            std::vector<int> temp2;
            Solution solCase1 = solution;
            Solution solCase2 = solution;
            Solution solCase3 = solution;
            Solution solCase1x;
            Solution solCase2x;
            Solution solCase3x;
            if (x2 + 1 >= solution.techTripList[x1].size())
            { 

                //Case 1:
                for (int i = y3; i < solCase1.droneTripList[y1][y2].size(); i++)
                {
                    temp1.push_back(solCase1.droneTripList[y1][y2][i]);
                }
                solCase1.droneTripList[y1][y2].erase(solCase1.droneTripList[y1][y2].begin() + y3, solCase1.droneTripList[y1][y2].end());
                solCase1.techTripList[x1].insert(solCase1.techTripList[x1].end(), temp1.begin(), temp1.end()); 
                
                solCase1x = solCase1;
                auto result = std::find(solCase1x.techTripList[x1].begin(), solCase1x.techTripList[x1].end(), row);
                int index = 0;
                if (result != solCase1x.techTripList[x1].end()) {
                    index = std::distance(solCase1x.techTripList[x1].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase1x.techTripList[x1][index] = col;
                solCase1x.techTripList[x1][index + 1] = row;

                //Case 2:
                int temp22 = solCase2.droneTripList[y1][y2][y3];
                solCase2.droneTripList[y1][y2].erase(solCase2.droneTripList[y1][y2].begin() + y3);
                solCase2.techTripList[x1].insert(solCase2.techTripList[x1].end(), temp22);
                
                solCase2x = solCase2;
                result = std::find(solCase2x.techTripList[x1].begin(), solCase2x.techTripList[x1].end(), row);
                index = 0;
                if (result != solCase2x.techTripList[x1].end()) {
                    index = std::distance(solCase2x.techTripList[x1].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase2x.techTripList[x1][index] = col;
                solCase2x.techTripList[x1][index + 1] = row;

                //Case 3: 
                int temp3 = solCase3.droneTripList[y1][y2][y3];
                solCase3.droneTripList[y1][y2].erase(solCase3.droneTripList[y1][y2].begin() + y3);
                solCase3.techTripList[x1].insert(solCase3.techTripList[x1].end(), temp3);
                
                solCase3x = solCase3;
                result = std::find(solCase3x.techTripList[x1].begin(), solCase3x.techTripList[x1].end(), row);
                index = 0;
                if (result != solCase3x.techTripList[x1].end()) {
                    index = std::distance(solCase3x.techTripList[x1].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase3x.techTripList[x1][index] = col;
                solCase3x.techTripList[x1][index + 1] = row;

            }
            else
            {
                //Case 1:
                for (int i = y3; i < solCase1.droneTripList[y1][y2].size(); i++)
                {
                    temp1.push_back(solCase1.droneTripList[y1][y2][i]);
                }
                solCase1.droneTripList[y1][y2].erase(solCase1.droneTripList[y1][y2].begin() + y3, solCase1.droneTripList[y1][y2].end());

                for (int i = x2 + 1; i < solCase1.techTripList[x1].size(); i++)
                {
                    temp2.push_back(solCase1.techTripList[x1][i]);
                }
                solCase1.techTripList[x1].erase(solCase1.techTripList[x1].begin() + x2 + 1, solCase1.techTripList[x1].end());

                solCase1.droneTripList[y1][y2].insert(solCase1.droneTripList[y1][y2].end(), temp2.begin(), temp2.end());
                solCase1.techTripList[x1].insert(solCase1.techTripList[x1].end(), temp1.begin(), temp1.end());
                
                solCase1x = solCase1;
                auto result = std::find(solCase1x.techTripList[x1].begin(), solCase1x.techTripList[x1].end(), row);
                int index = 0;
                if (result != solCase1x.techTripList[x1].end()) {
                    index = std::distance(solCase1x.techTripList[x1].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase1x.techTripList[x1][index] = col;
                solCase1x.techTripList[x1][index + 1] = row;
                //Case 2:
                int temp22 = solCase2.droneTripList[y1][y2][y3];
                solCase2.droneTripList[y1][y2][y3] = solCase2.techTripList[x1][x2+1];
                solCase2.techTripList[x1][x2+1] = temp22;
                
                solCase2x = solCase2;
                result = std::find(solCase2x.techTripList[x1].begin(), solCase2x.techTripList[x1].end(), row);
                index = 0;
                if (result != solCase2x.techTripList[x1].end()) {
                    index = std::distance(solCase2x.techTripList[x1].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase2x.techTripList[x1][index] = col;
                solCase2x.techTripList[x1][index + 1] = row;
                //Case 3:
                int temp3 = solCase3.droneTripList[y1][y2][y3];
                solCase3.droneTripList[y1][y2].erase(solCase3.droneTripList[y1][y2].begin() + y3);
                solCase3.techTripList[x1].insert(solCase3.techTripList[x1].begin() + x2, temp3);
                
                solCase3x = solCase3;
                result = std::find(solCase3x.techTripList[x1].begin(), solCase3x.techTripList[x1].end(), row);
                index = 0;
                if (result != solCase3x.techTripList[x1].end()) {
                    index = std::distance(solCase3x.techTripList[x1].begin(), result);
                } else {
                    std::cout << "BUG." << std::endl;
                }
                solCase3x.techTripList[x1][index] = col;
                solCase3x.techTripList[x1][index + 1] = row;
            }
            std::vector<Solution> check;
            if (solCase1.check_feasible()){
                check.insert(check.end() ,solCase1);
            }
            if(solCase1x.check_feasible()){
                check.insert(check.end() ,solCase1x);
            }
            if (solCase2.check_feasible()){
                check.insert(check.end() ,solCase2);
            }
            if(solCase2x.check_feasible()){
                check.insert(check.end() ,solCase2x);
            }
            if (solCase3.check_feasible()){
                check.insert(check.end() ,solCase3);
            }
            if(solCase3x.check_feasible()){
                check.insert(check.end() ,solCase3x);
            }

            if (check.size() == 0){
                
            }else{
                Solution fisol;
                double ress = 999999;
                for (int i = 0; i < check.size(); i++){
                    double hh = check[i].getScore();
                    if (hh < ress){
                        ress = hh;
                        fisol = check[i];
                    }
                }
                solution = fisol;
                
            }
            //std::cout<<std::endl<<"CASE 3 OUT";
        }
        else if ((x3 == -1) and (y3 == -1))
        {
            //std::cout<<std::endl<<"CASE 4";
            if (x1 == y1)
            {
                //std::cout<<std::endl<<"CASE 4.1";
                if (x2 == solution.techTripList[x1].size() - 1)
                {
                    /*
                    std::cout<<std::endl<<"Solution TECH"<<std::endl;
                    for (int u = 0; u < solution.techTripList.size(); u++){
                        for (int v = 0; v < solution.techTripList[u].size(); v++){
                            std::cout<<solution.techTripList[u][v]<<" ";
                        }
                        std::cout<<std::endl;
                    }
                    std::cout<<std::endl<<"Solution DRONE"<<std::endl;
                    for (int u = 0; u < solution.droneTripList.size(); u++){
                        for (int v = 0; v < solution.droneTripList[u].size(); v++){
                            for (int w = 0; w < solution.droneTripList[u][v].size(); w++){
                                std::cout<<solution.droneTripList[u][v][w]<<" ";
                            }std::cout<<" | ";
                        }
                        std::cout<<std::endl;
                    }
                    */
                    //std::cout<<std::endl<<"Row: "<<row<<" | "<<"Col: "<<col;
                    //std::cout<<std::endl<<"CASE 4.1.1";
                    Solution solutionx = solution;
                    int temp = solutionx.techTripList[y1][y2];
                    solutionx.techTripList[y1].erase(solutionx.techTripList[y1].begin() + y2);
                    solutionx.techTripList[x1].insert(solutionx.techTripList[x1].end(), temp);
                    
                    Solution solutiony = solutionx;
                    auto result = std::find(solutiony.techTripList[x1].begin(), solutiony.techTripList[x1].end(), row);
                    int index = 0;
                    if (result != solutiony.techTripList[x1].end()) {
                        index = std::distance(solutiony.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solutiony.techTripList[x1][index] = col;
                    solutiony.techTripList[x1][index + 1] = row;
                    std::vector<Solution> check;
                    if (solutionx.check_feasible()){
                        check.insert(check.end() ,solutionx);
                    }
                    if (solutiony.check_feasible()){
                        check.insert(check.end() ,solutiony);
                    }
                    if (check.size() == 0){
                        
                    }else{
                        Solution fisol;
                        double ress = 999999;
                        for (int i = 0; i < check.size(); i++){
                            double hh = check[i].getScore();
                            if (hh < ress){
                                ress = hh;
                                fisol = check[i];
                            }
                        }
                        solution = fisol;
                    }
                }
                else
                {
                    //std::cout<<std::endl<<"CASE 4.1.2";
                    Solution solutionx = solution;
                    int temp = solutionx.techTripList[x1][x2 + 1];
                    solutionx.techTripList[x1][x2 + 1] = solutionx.techTripList[y1][y2];
                    solutionx.techTripList[y1][y2] = temp;
                    
                    Solution solutiony = solutionx;
                    auto result = std::find(solutiony.techTripList[x1].begin(), solutiony.techTripList[x1].end(), row);
                    int index = 0;
                    if (result != solutiony.techTripList[x1].end()) {
                        index = std::distance(solutiony.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solutiony.techTripList[x1][index] = col;
                    solutiony.techTripList[x1][index + 1] = row;
                    std::vector<Solution> check;
                    if (solutionx.check_feasible()){
                        check.insert(check.end() ,solutionx);
                    }
                    if (solutiony.check_feasible()){
                        check.insert(check.end() ,solutiony);
                    }
                    if (check.size() == 0){
                        
                    }else{
                        Solution fisol;
                        double ress = 999999;
                        for (int i = 0; i < check.size(); i++){
                            double hh = check[i].getScore();
                            if (hh < ress){
                                ress = hh;
                                fisol = check[i];
                            }
                        }
                        solution = fisol;
                    }
                }          
            }
            else
            {
                //std::cout<<std::endl<<"CASE 4.2";
                std::vector<int> temp1;
                std::vector<int> temp2;   
                Solution solCase1 = solution;
                Solution solCase2 = solution;
                Solution solCase3 = solution;
                Solution solCase1x;
                Solution solCase2x;
                Solution solCase3x;
                if (x2 + 1 >= solution.techTripList[x1].size())
                {
                    //std::cout<<std::endl<<"CASE 4.2.1";
                    //Case 1:
                    for (int i = y2; i < solCase1.techTripList[y1].size(); i++)
                    {
                        temp2.push_back(solCase1.techTripList[y1][i]);
                    }
                    solCase1.techTripList[y1].erase(solCase1.techTripList[y1].begin() + y2, solCase1.techTripList[y1].end());
                    solCase1.techTripList[x1].insert(solCase1.techTripList[x1].end(), temp2.begin(), temp2.end());  
                    
                    solCase1x = solCase1;
                    auto result = std::find(solCase1x.techTripList[x1].begin(), solCase1x.techTripList[x1].end(), row);
                    int index = 0;
                    if (result != solCase1x.techTripList[x1].end()) {
                        index = std::distance(solCase1x.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase1x.techTripList[x1][index] = col;
                    solCase1x.techTripList[x1][index + 1] = row;
                    //Case 2:
                    int temp22 = solCase2.techTripList[y1][y2];
                    solCase2.techTripList[y1].erase(solCase2.techTripList[y1].begin() + y2);
                    solCase2.techTripList[x1].insert(solCase2.techTripList[x1].end(), temp22);
                    
                    solCase2x = solCase2;
                    result = std::find(solCase2x.techTripList[x1].begin(), solCase2x.techTripList[x1].end(), row);
                    index = 0;
                    if (result != solCase2x.techTripList[x1].end()) {
                        index = std::distance(solCase2x.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase2x.techTripList[x1][index] = col;
                    solCase2x.techTripList[x1][index + 1] = row;
                    //Case 3: 
                    int temp33 = solCase3.techTripList[y1][y2];
                    solCase3.techTripList[y1].erase(solCase3.techTripList[y1].begin() + y2);
                    solCase3.techTripList[x1].insert(solCase3.techTripList[x1].end(), temp33);
                    
                    solCase3x = solCase3;
                    result = std::find(solCase3x.techTripList[x1].begin(), solCase3x.techTripList[x1].end(), row);
                    index = 0;
                    if (result != solCase3x.techTripList[x1].end()) {
                        index = std::distance(solCase3x.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase3x.techTripList[x1][index] = col;
                    solCase3x.techTripList[x1][index + 1] = row;
                }
                else
                {
                    //std::cout<<std::endl<<"CASE 4.2.2";
                    //Case 1:
                    for (int i = x2 + 1; i < solCase1.techTripList[x1].size(); i++)
                    {
                        temp1.push_back(solCase1.techTripList[x1][i]);
                    }      
                    solCase1.techTripList[x1].erase(solCase1.techTripList[x1].begin() + x2 + 1, solCase1.techTripList[x1].end());

                    for (int i = y2; i < solCase1.techTripList[y1].size(); i++)
                    {
                        temp2.push_back(solCase1.techTripList[y1][i]);
                    }
                    solCase1.techTripList[y1].erase(solCase1.techTripList[y1].begin() + y2, solCase1.techTripList[y1].end());

                    solCase1.techTripList[x1].insert(solCase1.techTripList[x1].end(), temp2.begin(), temp2.end());
                    solCase1.techTripList[y1].insert(solCase1.techTripList[y1].end(), temp1.begin(), temp1.end());
                    solCase1x = solCase1;
                    auto result = std::find(solCase1x.techTripList[x1].begin(), solCase1x.techTripList[x1].end(), row);
                    int index = 0;
                    if (result != solCase1x.techTripList[x1].end()) {
                        index = std::distance(solCase1x.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase1x.techTripList[x1][index] = col;
                    solCase1x.techTripList[x1][index + 1] = row;
                    //Case 2:
                    int temp22 = solCase2.techTripList[y1][y2];
                    solCase2.techTripList[y1][y2] = solCase2.techTripList[x1][x2+1];
                    solCase2.techTripList[x1][x2+1] = temp22;
                    
                    solCase2x = solCase2;
                    result = std::find(solCase2x.techTripList[x1].begin(), solCase2x.techTripList[x1].end(), row);
                    index = 0;
                    if (result != solCase2x.techTripList[x1].end()) {
                        index = std::distance(solCase2x.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase2x.techTripList[x1][index] = col;
                    solCase2x.techTripList[x1][index + 1] = row;
                    //Case 3:
                    int temp3 = solCase3.techTripList[y1][y2];
                    solCase3.techTripList[y1].erase(solCase3.techTripList[y1].begin() + y2);
                    solCase3.techTripList[x1].insert(solCase3.techTripList[x1].begin() + x2, temp3);
                    
                    solCase3x = solCase3;
                    result = std::find(solCase3x.techTripList[x1].begin(), solCase3x.techTripList[x1].end(), row);
                    index = 0;
                    if (result != solCase3x.techTripList[x1].end()) {
                        index = std::distance(solCase3x.techTripList[x1].begin(), result);
                    } else {
                        std::cout << "BUG." << std::endl;
                    }
                    solCase3x.techTripList[x1][index] = col;
                    solCase3x.techTripList[x1][index + 1] = row;
                }
                std::vector<Solution> check;
                if (solCase1.check_feasible()){
                    check.insert(check.end() ,solCase1);
                }
                if(solCase1x.check_feasible()){
                    check.insert(check.end() ,solCase1x);
                }
                if (solCase2.check_feasible()){
                    check.insert(check.end() ,solCase2);
                }
                if(solCase2x.check_feasible()){
                    check.insert(check.end() ,solCase2x);
                }
                if (solCase3.check_feasible()){
                    check.insert(check.end() ,solCase3);
                }
                if(solCase3x.check_feasible()){
                    check.insert(check.end() ,solCase3x);
                }

                if (check.size() == 0){
                    
                }else{
                    Solution fisol;
                    double ress = 999999;
                    for (int i = 0; i < check.size(); i++){
                        double hh = check[i].getScore();
                        if (hh < ress){
                            ress = hh;
                            fisol = check[i];
                        }
                    }
                    solution = fisol; 
                }
            }
            //std::cout<<std::endl<<"CASE 4 OUT";
        }
        else
        {
            std::cout<<std::endl<<"BUG";
        }
        if (solution.check_feasible() == false){
            solution = solbefore;
        }
    } 
    int length = 0;
    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            length = length + std::max((int)solution.droneTripList[i][j].size() - 1, 0);
        }
    }
    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        length = length + std::max((int)solution.techTripList[i].size() - 1, 0);
    }
    int number = NumCus * config.percent_match + 1;
    int numUpdateReal = std::min(number, length);

    while (update_Real.size() < numUpdateReal)
    {
        double count = 9999999;
        int row = 1;
        int col = 1;
        for (int i = 0; i < solution.droneTripList.size(); i++)
        {
            if (solution.droneTripList[i].size() == 0) {continue;}
            for (int j = 0; j < solution.droneTripList[i].size(); j++)
            {
                if (solution.droneTripList[i][j].size() <= 1) {continue;}
                for (int k = 0; k < solution.droneTripList[i][j].size() - 1; k++)
                {
                    auto find = std::find(update_Real.begin(), update_Real.end(), std::make_tuple(solution.droneTripList[i][j][k], solution.droneTripList[i][j][k+1]));
                    if ((distanceMatrix[solution.droneTripList[i][j][k]][solution.droneTripList[i][j][k+1]] <= count) && (find == update_Real.end()))
                    {
                        count = distanceMatrix[solution.droneTripList[i][j][k]][solution.droneTripList[i][j][k+1]];
                        row = solution.droneTripList[i][j][k];
                        col = solution.droneTripList[i][j][k+1];
                    }
                }
            }
        }        
        for (int i = 0; i < solution.techTripList.size(); i++)
        {
            if (solution.techTripList[i].size() <= 1) {continue;}
            for (int j = 0; j < solution.techTripList[i].size() - 1; j++)
            {
                auto find = std::find(update_Real.begin(), update_Real.end(), std::make_tuple(solution.techTripList[i][j], solution.techTripList[i][j+1]));
                if ((distanceMatrix[solution.techTripList[i][j]][solution.techTripList[i][j+1]] <= count) && (find == update_Real.end()))
                {
                    count = distanceMatrix[solution.techTripList[i][j]][solution.techTripList[i][j+1]];
                    row = solution.techTripList[i][j];
                    col = solution.techTripList[i][j+1];
                }
            }
        }
        update_Real.push_back(std::make_tuple(row, col));
    }
    
    update = update_Real;
    numUpdate = update.size();
    
    int max2 = 0;
    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            for (int k = 0; k < solution.droneTripList[i][j].size(); k++)
            {
                if (solution.droneTripList[i][j][k] > max2) max2 = solution.droneTripList[i][j][k];
            }
        }
    }
    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        for (int j = 0; j < solution.techTripList[i].size(); j++)
        {
            if (solution.techTripList[i][j] > max2) max2 = solution.techTripList[i][j];
        }
    }
    //merge of DroneTripList and TechTripList: First Part
    std::vector<std::tuple<int , int>> updateFake = update;
    std::map<int, std::vector<int>> map;
    map[0] = {0};
    std::vector<std::vector<int>> beMerge;
    std::vector<int> temp;
    std::tuple<int, int> fi;
    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        if (solution.droneTripList[i].size() == 0) {continue;}
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            if (solution.droneTripList[i][j].size() <= 1) {continue;}
            for (int k = 0; k < solution.droneTripList[i][j].size() - 1; k++)
            {
                fi = std::make_tuple(solution.droneTripList[i][j][k], solution.droneTripList[i][j][k+1]);
                for (int u = 0; u < updateFake.size(); u++)
                {
                    if (fi == updateFake[u])
                    {
                        if (temp.size() == 0)
                        {
                            temp.push_back(solution.droneTripList[i][j][k]);
                            temp.push_back(solution.droneTripList[i][j][k+1]);
                        }
                        else
                        {
                            if (temp[temp.size() - 1] != solution.droneTripList[i][j][k])
                            {
                                beMerge.push_back(temp);
                                temp.resize(0);
                                temp.push_back(solution.droneTripList[i][j][k]);
                                temp.push_back(solution.droneTripList[i][j][k+1]);
                            }
                            else
                            {
                                temp.push_back(solution.droneTripList[i][j][k+1]);
                            }
                        }
                        updateFake.erase(updateFake.begin() + u);
                        break;
                    }
                }
            }
            if (temp.size() != 0) {beMerge.push_back(temp);}
            temp = {};
        }
    }
    temp = {};

    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        if (solution.techTripList[i].size() <= 1) {continue;}
        for (int j = 0; j < solution.techTripList[i].size() - 1; j++)
        {
            fi = std::make_tuple(solution.techTripList[i][j], solution.techTripList[i][j+1]);
            for (int u = 0; u < updateFake.size(); u++)
            {
                if (fi == updateFake[u])
                {
                    if (temp.size() == 0)
                    {
                        temp.push_back(solution.techTripList[i][j]);
                        temp.push_back(solution.techTripList[i][j+1]);
                    }
                    else
                    {
                        if (temp[temp.size() - 1] != solution.techTripList[i][j])
                        {
                            beMerge.push_back(temp);
                            temp.resize(0);
                            temp.push_back(solution.techTripList[i][j]);
                            temp.push_back(solution.techTripList[i][j+1]);
                        }
                        else
                        {
                            temp.push_back(solution.techTripList[i][j+1]);
                        }
                    }
                    updateFake.erase(updateFake.begin() + u);
                    break;
                }
            }
        }
        if (temp.size() != 0) {beMerge.push_back(temp);}
        temp = {};
    }


    std::vector<int> del;
    for (int u = 0; u < beMerge.size(); u++)
    {
        for (int i = 0; i < solution.droneTripList.size(); i++)
        {
            if (solution.droneTripList[i].size() == 0) {continue;}
            for (int j = 0; j < solution.droneTripList[i].size(); j++)
            {
                if (solution.droneTripList[i][j].size() <= 1) {continue;}
                for (int k = 0; k < solution.droneTripList[i][j].size() - 1; k++)
                {

                    if (beMerge[u][0] == solution.droneTripList[i][j][k])
                    {
                        auto minElement = std::min_element(beMerge[u].begin(), beMerge[u].end());
                        int insertt = *minElement;
                        for (int l = 0; l < beMerge[u].size(); l++)
                        {
                            del.push_back(solution.droneTripList[i][j][k]);
                            solution.droneTripList[i][j].erase(solution.droneTripList[i][j].begin() + k);
                        }
                        solution.droneTripList[i][j].insert(solution.droneTripList[i][j].begin() + k, insertt);
                        break;
                    }
                    
                }
            }
        }
    }
    for (int u = 0; u < beMerge.size(); u++)
    {
        for (int i = 0; i < solution.techTripList.size(); i++)
        {
            if (solution.techTripList[i].size() <= 1) {continue;}
            for (int j = 0; j < solution.techTripList[i].size() - 1; j++)
            {

                if (beMerge[u][0] == solution.techTripList[i][j])
                {
                    auto minElement = std::min_element(beMerge[u].begin(), beMerge[u].end());
                    int insertt = *minElement;
                    map[insertt] = {std::get<0>(update[u]), std::get<1>(update[u])};
                    for (int l = 0; l < beMerge[u].size(); l++)
                    {
                        del.push_back(solution.techTripList[i][j]);
                        solution.techTripList[i].erase(solution.techTripList[i].begin() + j);
                    }
                    solution.techTripList[i].insert(solution.techTripList[i].begin() + j, insertt);
                    break;
                }
                
                
            }
        }
    }
    //merge of DroneListTrip and TechTripList: Second Part
    for (int i = 0; i < beMerge.size(); i++)
    {
        auto minElement = std::min_element(beMerge[i].begin(), beMerge[i].end());
        map[*minElement] = beMerge[i];
    }

    std::vector<int> fill;

    for (int i = 1; i <= max2; i++)
    {
        int keytofind = i;
        auto ite1 = map.find(keytofind);
        auto ite2 = std::find(del.begin(), del.end(), i);
        if ((ite1 == map.end()) and (ite2 == del.end()))
        {
            map[i] = {i};
        }
    }
    for (int i = 1; i <= max2; i++)
    {
        fill.push_back(i);
    }
    for (int u = 1; u <= max2; u++)
    {
        for (int i = 0; i < solution.droneTripList.size(); i++)
        {
            for (int j = 0; j < solution.droneTripList[i].size(); j++)
            {
                auto find = std::find(solution.droneTripList[i][j].begin(), solution.droneTripList[i][j].end(), u);
                if (find != solution.droneTripList[i][j].end()) 
                {
                    fill.erase(std::remove(fill.begin(), fill.end(), u), fill.end());
                }
            }
        }

        for (int i = 0; i < solution.techTripList.size(); i++)
        {
            auto find = std::find(solution.techTripList[i].begin(), solution.techTripList[i].end(), u);
            if (find != solution.techTripList[i].end())
            {
                fill.erase(std::remove(fill.begin(), fill.end(), u), fill.end());
            }
        }
    }
    
    std::sort(fill.begin(), fill.end());
    for (int i = 1; i <= max2; i++)
    {
        int keytofind = i;
        auto ite = map.find(keytofind);
        if (ite != map.end())
        {
            if (i > fill[(int)fill.size() - 1])
            {
                map[i - (int)fill.size()] = map[i];
            }
            else
            {
                for (int j = 0; j < (int)fill.size() - 1; j++)
                {
                    if ((fill[j] < i) and (fill[j+1] > i))
                    {
                        map[i - (j+1)] = map[i];
                    }
                }
            }
        }
    }
    

    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            for (int k = 0; k < solution.droneTripList[i][j].size(); k++)
            {
                if (solution.droneTripList[i][j][k] > fill[(int)fill.size() - 1])
                {
                    
                    solution.droneTripList[i][j][k] = solution.droneTripList[i][j][k] - (int)fill.size();
                }
                else
                {
                    for (int u = 0; u < (int)fill.size() - 1; u++)
                    {
                        if ((fill[u] < solution.droneTripList[i][j][k]) and (solution.droneTripList[i][j][k] < fill[u+1]))
                        {
                            solution.droneTripList[i][j][k] = solution.droneTripList[i][j][k] - (u+1);
                        }
                    }
                } 
            }
        }
    }
    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        for (int j = 0; j < solution.techTripList[i].size(); j++)
        {
            if (solution.techTripList[i][j] > fill[(int)fill.size() - 1])
            {
                solution.techTripList[i][j] = solution.techTripList[i][j] - (int)fill.size();
            }
            else
            {
                for (int u = 0; u < (int)fill.size(); u++)
                {
                    if ((fill[u] < solution.techTripList[i][j]) and (solution.techTripList[i][j] < fill[u+1]))
                    {                        
                        solution.techTripList[i][j] = solution.techTripList[i][j] - (u+1);
                    }
                }
            }
        }
    }

    int max1 = 0;
    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            for (int k = 0; k < solution.droneTripList[i][j].size(); k++)
            {
                if (solution.droneTripList[i][j][k] > max1) max1 = solution.droneTripList[i][j][k];
            }
        }
    }
    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        for (int j = 0; j < solution.techTripList[i].size(); j++)
        {
            if (solution.techTripList[i][j] > max1) max1 = solution.techTripList[i][j];
        }
    }

    numUpdate = max2 - max1;
    for (int i = max1 + 1; i <= max2; i++)
    {
        map.erase(i);
    }

    return std::make_tuple(numUpdate, solution, map);
}

//main process: Init Sol -> Tabu(40) -> merge_process:((Best(level_i) -> merge_sol -> Tabu(40) -> Best_new(level_i+1)) * num_level) 
//                                  -> Best(level_3) -> split_process:((Best(level_i) -> split_sol -> Tabu(40) -> Best(level_i-1)) * num_level) -> Best_Solution

std::tuple<Solution, std::map<int, std::vector<int>>, std::vector<double>> MultiLevel::mergeProcess(Config &config, Input &input, std::vector<std::map<int, std::vector<int>>> &mapLevel, std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, std::vector<std::vector<bool>> &C1Level)
{
    std::vector<double> scoree;
    TabuSearch tabuSearch(config, input);
    Solution currentSol;
    currentSol.setInput(input);
    currentSol = tabuSearch.initSolution;
    std::vector<std::vector<int>> mainMatrix;
    std::tuple<int, Solution, std::map<int, std::vector<int>>> merge;
    std::vector<std::vector<double>> distanceMatrix = input.distances;
    DistanceMatrixLevel.push_back(distanceMatrix);
    config.tabuMaxIter = config.tabu_max_ite;
    double bestScore = currentSol.getScore();
    scoree.push_back(bestScore);
    int numcus = input.numCus;
    C1Level.push_back(input.cusOnlyServedByTech);
    for (int i = 0; i < config.num_level; i++)
    {
        json log;
        std::string path_e;  
        TabuSearch tabuSearch(config, input);
        MultiLevel multilev(config, input);
        currentSol.setInput(input);
        std::cout<<std::endl<<"Score before tabu in merge process Level "<<(i+1)<<" is: "<<currentSol.getScore();
        std::tuple<double, Solution, std::vector<std::vector<int>>> result = tabuSearch.run(log, path_e, input, currentSol);
        Solution sol;
        sol.setInput(input);
        currentSol.setInput(input);
        double temp = currentSol.getScore();
        sol = std::get<1> (result);
        if (sol.check_feasible() == false)
        {
            sol = currentSol;
        }
        if ((sol.droneTripList.size() == 0) or (sol.techTripList.size() == 0))
        {
            sol = currentSol;
        }
        mainMatrix = std::get<2> (result);
        merge = multilev.mergeSol(sol, numcus, mainMatrix, DistanceMatrixLevel[i], C1Level[i]);
        numcus = numcus - std::get<0> (merge);
        currentSol = std::get<1> (merge);
        if (bestScore > temp)
        {
            bestScore = temp;
        }
        mapLevel.push_back(std::get<2> (merge));
        distanceMatrix = multilev.convertMatrix(distanceMatrix, std::get<2> (merge));
        int sizee = distanceMatrix.size();
        std::vector<std::vector<double>> droneFlightMatrix(sizee, std::vector<double>(sizee, 0));
        std::vector<std::vector<double>> techMoveMatrix(sizee, std::vector<double>(sizee, 0));
        for (int j = 0; j < sizee; j++)
        {
            for (int k = 0; k < sizee; k++)
            {
                droneFlightMatrix[j][k] = distanceMatrix[j][k] / config.droneVelocity;
                techMoveMatrix[j][k] = distanceMatrix[j][k] / config.techVelocity;
            }
        }
        std::vector<std::vector<double>> coor(sizee, std::vector<double>(sizee, 0));
        input.numCus = numcus;
        input.coordinates = coor;
        input.distances = distanceMatrix;
        DistanceMatrixLevel.push_back(distanceMatrix);
        input.droneTimes = droneFlightMatrix;
        input.techTimes = techMoveMatrix;
        std::vector<bool> C1(numcus + 1, false);

        for (int j = 1; j < numcus + 1; j++)
        {
            int count = DistanceMatrixLevel[i][0][std::get<2> (merge)[j][0]];
            for (int k = 1; k < std::get<2> (merge)[j].size() - 1; k++)
            {
                count = count + DistanceMatrixLevel[i][std::get<2> (merge)[j][k]][std::get<2> (merge)[j][k+1]];
            }
            count = count/config.droneVelocity;
            if (count > config.droneLimitationFlightTime)
            {
                C1[j] = true;
            }
        }

        C1Level.push_back(C1);
        input.cusOnlyServedByTech = C1;
        currentSol.setInput(input);
        scoree.push_back(currentSol.getScore());
    }
    json logg;
    std::cout<<std::endl<<"Score before post optimization is: "<<currentSol.getScore();
    currentSol = tabuSearch.runPostOptimization(logg, currentSol);
    std::cout<<std::endl<<"Score after post optimization is: "<<currentSol.getScore();
    scoree.push_back(currentSol.getScore());
    return std::make_tuple(currentSol, std::get<2> (merge), scoree);
}

Solution MultiLevel::splitSol(Solution solution, std::map<int, std::vector<int>> Map)
{

    for (int i = 0; i < solution.droneTripList.size(); i++)
    {
        for (int j = 0; j < solution.droneTripList[i].size(); j++)
        {
            for (int k = 0; k < solution.droneTripList[i][j].size(); k++)
            {
                int temp = solution.droneTripList[i][j][k];
                solution.droneTripList[i][j].erase(solution.droneTripList[i][j].begin() + k);
                for (int u = Map[temp].size() - 1; u >= 0; u--)
                {
                    solution.droneTripList[i][j].insert(solution.droneTripList[i][j].begin() + k, Map[temp][u]);
                }
                k = k + Map[temp].size() - 1;
                if (k >= solution.droneTripList[i][j].size() - 1) {break;}
            }
        }
    }
    for (int i = 0; i < solution.techTripList.size(); i++)
    {
        for (int j = 0; j < solution.techTripList[i].size(); j++)
        {
            int temp = solution.techTripList[i][j];
            solution.techTripList[i].erase(solution.techTripList[i].begin() + j);
            for (int u = Map[temp].size() - 1; u >= 0; u--)
            {
                solution.techTripList[i].insert(solution.techTripList[i].begin() + j, Map[temp][u]);
            }
            j = j + Map[temp].size() - 1;
            if (j >= solution.techTripList[i].size() - 1) {break;}
        }
    }
    return solution;
}

std::tuple<Solution, std::vector<double>> MultiLevel::splitProcess(Solution solution, Config &config, Input &input, std::vector<std::map<int, std::vector<int>>> &mapLevel, std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, std::vector<std::vector<bool>> &C1Level)
{
    solution.setInput(input);
    double best = solution.getScore();
    std::vector<double> scoree;
    for (int i = 0; i < config.num_level; i++)
    {
        json log;
        std::string path_e;
        MultiLevel multilev(config, input);
        solution.setInput(input);
        solution = multilev.splitSol(solution, mapLevel[config.num_level - i - 1]);
        int sizee = DistanceMatrixLevel[config.num_level - i - 1].size();
        std::vector<std::vector<double>> droneFlightMatrix(sizee, std::vector<double>(sizee, 0));
        std::vector<std::vector<double>> techMoveMatrix(sizee, std::vector<double>(sizee, 0));
        for (int j = 0; j < sizee; j++)
        {
            for (int k = 0; k < sizee; k++)
            {
                droneFlightMatrix[j][k] = DistanceMatrixLevel[config.num_level - i - 1][j][k] / config.droneVelocity;
                techMoveMatrix[j][k] = DistanceMatrixLevel[config.num_level - i - 1][j][k] / config.techVelocity;
            }
        }
        std::vector<std::vector<double>> coor(sizee, std::vector<double>(sizee, 0));
        input.numCus = sizee - 2;
        input.coordinates = coor;
        input.distances = DistanceMatrixLevel[config.num_level - i - 1];
        input.droneTimes = droneFlightMatrix;
        input.techTimes = techMoveMatrix;
        input.cusOnlyServedByTech = C1Level[config.num_level - i - 1];
        TabuSearch tabuSearch(config, input);
        solution.setInput(input);
        std::cout<<std::endl<<"Score before tabu in Split process Level "<<(i+1)<<" is: "<<solution.getScore();
        std::tuple<double, Solution, std::vector<std::vector<int>>> result = tabuSearch.run(log, path_e, input, solution);
        solution = std::get<1> (result);
        best = std::get<0> (result);
        scoree.push_back(best);
    }
    json logg;
    std::cout<<std::endl<<"Score before post optimization is: "<<solution.getScore();
    solution = tabuSearch.runPostOptimization(logg, solution);
    std::cout<<std::endl<<"Score after post optimization is: "<<solution.getScore();
    scoree.push_back(solution.getScore());
    return std::make_tuple(solution, scoree);
}

std::tuple<double, Solution, std::vector<double>> MultiLevel::run(Config &config, Input &input)
{
    json log;
    std::string path_e;
    MultiLevel multilev(config, input);        
    
    std::tuple<Solution, std::map<int, std::vector<int>>, std::vector<double>> result = multilev.mergeProcess(config, input, mapLevel, DistanceMatrixLevel, C1Level);
    Solution res_sol = std::get<0> (result);

    std::tuple<Solution, std::vector<double>> tue = multilev.splitProcess(res_sol, config, input, mapLevel, DistanceMatrixLevel, C1Level);
    Solution solution = std::get<0> (tue);
    solution.setInput(input);
    std::vector<double> score1 = std::get<2> (result);
    std::vector<double> score2 = std::get<1> (tue);
    std::vector<double> scoree;
    scoree.insert(scoree.end(), score1.begin(), score1.end());
    scoree.insert(scoree.end(), score2.begin(), score2.end());
    return std::make_tuple(solution.getScore(), solution, scoree);
}
