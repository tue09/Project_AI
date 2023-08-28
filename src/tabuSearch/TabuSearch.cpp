
#include "TabuSearch.h"
#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Solution.h"
#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Random.h"
#include <chrono>
#include "nlohmann/json.hpp"

using namespace std::chrono;

using Random = effolkronium::random_static;
using json = nlohmann::json;

void TabuSearch::setConfigandInput(Config &conf, Input &inp)
{
    this->config = conf;
    this->input = inp;
}

std::tuple<double, Solution, std::vector<std::vector<int>>> TabuSearch::run(json &log, std::string &path, Input &input, Solution solution)
{
    setConfigandInput(config, input);
    solution.setInput(input);

    json feasible;
    std::vector<std::vector<int>> matrix;
    matrix.resize(input.numCus + 1);
    for (int i = 0; i < matrix.size(); i++)
    {
        matrix[i].resize(input.numCus + 1);
    }
    std::map<NeighborhoodType, std::vector<std::string>> tabuLists;
    tabuLists[MOVE_10] = std::vector<std::string>();
    tabuLists[MOVE_11] = std::vector<std::string>();
    tabuLists[MOVE_20] = std::vector<std::string>();
    tabuLists[MOVE_21] = std::vector<std::string>();
    tabuLists[TWO_OPT] = std::vector<std::string>();
    Solution bestFeasibleSolution;
    bestFeasibleSolution.setInput(input);
    bestSolution.setInput(input);
    int notImproveIter = 0;
    currentSolution = solution;
    currentSolution.setInput(input);
    double currentScore = solution.getScore();
    bestSolution = solution;
    double bestScore = currentScore;
    double bestFeasibleScore = 999999;
    NeighborhoodType neighborhoodType;
    int actOrd;
    Solution *s;
    feasible["best feasible score"] = std::to_string(9999999);
    if (currentSolution.check_feasible()){
        json jDrone(currentSolution.droneTripList);
        json jTech(currentSolution.techTripList);
        feasible["best feasible score"] = std::to_string(currentScore);
        feasible["best feasible"] = std::to_string(currentScore) + jDrone.dump() + " || " + jTech.dump();
        bestFeasibleSolution = currentSolution;
        bestFeasibleScore = currentScore;
        bestFeasibleSolution.setInput(input);
    }

    //start Tabu
    int notFeasibleImprove = 0;
    auto start = high_resolution_clock::now();
    int actOrderCycle = -1;
    for (int it = 0; it < config.tabuMaxIter; it++)
    {
        json jDrone(currentSolution.droneTripList);
        json jTech(currentSolution.techTripList);
        actOrderCycle = (actOrderCycle + 1) % 5;
        if (config.isCycle)
        {
            actOrd = actOrderCycle + 1;
        }
        else
        {
            actOrd = Random::get(1, 5);
        }
        //std::cout<<std::endl<<"actord is: "<<actOrd;
        config.isCycle = 1;
        
        //        actOrd = 4;
        //        std::cout << "act: " << actOrd << std::endl;
        //        auto start = high_resolution_clock::now();
        switch (actOrd)
        {
        case MOVE_10:
        {
            neighborhoodType = MOVE_10;
            s = currentSolution.relocate(tabuLists[MOVE_10], bestScore);
            break;
        }
        case MOVE_11:
        {
            neighborhoodType = MOVE_11;
            s = currentSolution.exchange(tabuLists[MOVE_11], bestScore);
            break;
        }
        case MOVE_20:
        {
            neighborhoodType = MOVE_20;
            s = currentSolution.orOpt(tabuLists[MOVE_20], bestScore);
            break;
        }
        case MOVE_21:
        {
            neighborhoodType = MOVE_21;
            s = currentSolution.crossExchange(tabuLists[MOVE_21], bestScore);
            break;
        }
        case TWO_OPT:
        {
            neighborhoodType = TWO_OPT;
            s = currentSolution.twoOpt(tabuLists[TWO_OPT], bestScore);
            break;
        }
        default:
        {
            s = nullptr;
            break;
        }
        }
        if (s != nullptr)
        {
            s->refactorSolution();
            tabuLists[neighborhoodType].push_back(s->ext["state"]);
            while (tabuLists[neighborhoodType].size() > tabuDuration)
            {
                tabuLists[neighborhoodType].erase(tabuLists[neighborhoodType].begin());
            }
            json jDroneOld(currentSolution.droneTripList);
            json jTechOld(currentSolution.techTripList);
            currentScore = currentSolution.getScore();
            currentSolution = *s;
            currentSolution.setInput(input);
            currentScore = currentSolution.getScore();
            json jDrone(currentSolution.droneTripList);
            json jTech(currentSolution.techTripList);
            if (currentSolution.check_feasible() && (currentScore - bestFeasibleScore < config.tabuEpsilon))
            {
                notFeasibleImprove = 0;
                bestScore = currentScore;
                //config.isCycle = 0;
                bestSolution = currentSolution;
                bestSolution.setInput(input);
                notImproveIter = 0;
                updatePenalty(bestSolution.dz, bestSolution.dz);
                bestSolution.alpha1 = alpha1;
                bestSolution.alpha2 = alpha2;
                currentSolution.alpha1 = alpha1;
                currentSolution.alpha2 = alpha2;
                feasible["best feasible score"] = std::to_string(currentScore);
                feasible["best feasible"] = std::to_string(currentScore) + jDrone.dump() + " || " + jTech.dump();
                bestFeasibleSolution = currentSolution;
                bestFeasibleSolution.setInput(input);
                bestFeasibleScore = currentScore;
            }else{
                notFeasibleImprove = notFeasibleImprove + 1;
                if (notFeasibleImprove >= 5){
                    //config.isCycle = 0;
                }
                notImproveIter++;
                if (notImproveIter > config.tabuNotImproveIter)
                {
                    break;
                }
            }
            json jDroneBest(bestSolution.droneTripList);
            json jTechBest(bestSolution.techTripList);
        }

        //        auto stop = high_resolution_clock::now();
        //        std::cout << it << ": "
        //                  << bestScore << "-" << neighborhoodType << " time: "
        //                  << duration_cast<seconds>(stop - start).count() << "s" << std::endl;

        //        std::cout << it << ": " << itLog.dump(4) << std::endl;
    }
    auto stop = high_resolution_clock::now();
    json jDroneBest(bestSolution.droneTripList);
    json jTechBest(bestSolution.techTripList);
    return std::make_tuple(bestFeasibleScore, bestFeasibleSolution, matrix);
}

TabuSearch::TabuSearch(Config &conf, Input &inp)
{
    this->config = conf;
    this->input = inp;
    this->tabuDuration = Random::get(config.minTabuDuration, config.maxTabuDuration);
    this->alpha1 = conf.tabuAlpha1;
    this->alpha2 = conf.tabuAlpha2;
    Solution *init = Solution::initSolution(this->config, this->input, InitType::MIX, alpha1, alpha2);
    if (init == nullptr)
    {
        return;
    }
    initSolution = *init;
}

void TabuSearch::updatePenalty(double dz, double cz)
{
    if (dz > 0)
    {
        alpha1 *= 1 + config.tabuBeta;
    }
    else
    {
        alpha1 /= 1 + config.tabuBeta;
    }

    if (cz > 0)
    {
        alpha2 *= 1 + config.tabuBeta;
    }
    else
    {
        alpha2 /= 1 + config.tabuBeta;
    }
}

Solution TabuSearch::runPostOptimization(json &log, Solution solution)
{
    //auto start = high_resolution_clock::now();

    Solution solutionx = solution;
    solutionx = runEjection(solutionx);
    if (solutionx.check_feasible() && solution.getScore() < solutionx.getScore()){
        solution = solutionx;
    }
    //json jDroneBestEjection(solution.droneTripList);
    //json jTechBestEjection(solution.techTripList);
    //log["best_ejection"] = std::to_string(bestSolution.getScore()) + " == " + jDroneBestEjection.dump() + " || " + jTechBestEjection.dump();

    //    std::cout << "Ejection: " << log["best_ejection"].dump(4) << std::endl;

    solution = runInterRoute(solution);
    //json jDroneBestInter(solution.droneTripList);
    //json jTechBestInter(solution.techTripList);
    //log["best_inter"] = std::to_string(bestSolution.getScore()) + " == " + jDroneBestInter.dump() + " || " + jTechBestInter.dump();

    //    std::cout << "Inter: " << log["best_inter"].dump(4) << std::endl;

    solution = runIntraRoute(solution);


    //json jDroneBestIntra(bestSolution.droneTripList);
    //json jTechBestIntra(bestSolution.techTripList);
    //log["best_intra"] = std::to_string(bestSolution.getScore()) + " == " + jDroneBestIntra.dump() + " || " + jTechBestIntra.dump();
    //    std::cout << "Intra: " << log["best_intra"].dump(4) << std::endl;
    auto stop = high_resolution_clock::now();
    //log["post_optimization_time"] = duration_cast<milliseconds>(stop - start).count();
    return solution;
}

Solution TabuSearch::runInterRoute(Solution &solution)
{
    auto rng = std::default_random_engine(std::chrono::system_clock::now()
                                              .time_since_epoch()
                                              .count());
    std::vector<InterRouteType> order{INTER_RELOCATE, INTER_CROSS_EXCHANGE, INTER_EXCHANGE, INTER_OR_OPT,
                                      INTER_TWO_OPT};

    double score = solution.getScore();
    double newScore;

    Solution *s;
    int x = 0;
    while (true)
    {
        x = x + 1;
        std::shuffle(order.begin(), order.end(), rng);
        bool hasImprove = false;

        for (InterRouteType type : order)
        {
            switch (type)
            {
            case INTER_RELOCATE:
            {
                s = solution.relocate({}, score, INTER);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_EXCHANGE:
            {
                s = solution.exchange({}, score, INTER);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_OR_OPT:
            {
                s = solution.orOpt({}, score, INTER);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_TWO_OPT:
            {
                s = solution.twoOpt({}, score, INTER);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_CROSS_EXCHANGE:
            {
                s = solution.crossExchange({}, score, INTER);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            }
        }
        solution.refactorSolution();

        if (!hasImprove && x >= 5)
        {
            std::cout<<std::endl<<x<<" | "<<solution.getScore();
            break;
        }
    }
    return solution;
}

Solution TabuSearch::runIntraRoute(Solution &solution)
{
    auto rng = std::default_random_engine(std::chrono::system_clock::now()
                                              .time_since_epoch()
                                              .count());
    std::vector<IntraRouteType> order{INTRA_RELOCATE, INTRA_EXCHANGE, INTRA_OR_OPT, INTRA_TWO_OPT};

    double score = solution.getScore();
    double newScore;
    int x = 0;
    Solution *s;
    while (true)
    {
        x = x + 1;
        std::shuffle(order.begin(), order.end(), rng);
        bool hasImprove = false;

        for (IntraRouteType type : order)
        {
            switch (type)
            {
            case INTRA_RELOCATE:
            {
                s = solution.relocate({}, score, INTRA);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTRA_EXCHANGE:
            {
                s = solution.exchange({}, score, INTRA);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTRA_TWO_OPT:
            {
                s = solution.twoOpt({}, score, INTRA);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTRA_OR_OPT:
            {
                s = solution.orOpt({}, score, INTRA);
                if (s != nullptr)
                {
                    newScore = s->getScore();
                    if (newScore < score && s->check_feasible())
                    {
                        x = 0;
                        solution = *s;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            }
        }
        solution.refactorSolution();

        if (!hasImprove && x >= 5)
        {
            std::cout<<std::endl<<x<<" | "<<solution.getScore();
            break;
        }
    }
    return solution;
}

Solution TabuSearch::runEjection(Solution &solution)
{
    solution = solution.ejection();
    solution.refactorSolution();
    return solution;
}

TabuSearch::TabuSearch() = default;