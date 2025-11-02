#pragma once

#include "GANOPConfig.h"
#include "nop.hpp"
#include <vector>
#include <cmath>
#include <random>
#include <functional>
#include <string>
#include <fstream>
#include <stdexcept>
#include "runner.hpp"

using TArrInt = std::vector<int>;
using TArrReal = std::vector<float>;

class GANOP
{
public:
    // --- популяция ---
    std::vector<TArrInt> PopChrPar;              // параметры 
    std::vector<std::vector<TArrInt>> PopChrStr; // структуры

    int m_p;   // число параметров
    int m_c;   // число бит в целой части
    int m_d;   // число бит в плавающей части
    int m_nfu; // размер Fu
    int m_HH;  // размер популяции
    int m_lchr;  // количества вариаций в одном решении
    int m_PP; // число поколений
    int m_RR; // число кроссоверов 

    std::vector<TArrReal> m_Fuh; // значения функций
    std::vector<int> m_Lh;       // расстояния

    std::vector<TArrReal> m_FuhSon;
    std::vector<int> m_LhSon;
    std::vector<int> Pareto;          // индексы Парето-оптимальных

    // Дополнительные параметры из config
    float mutation_prob;
    float selection_alpha;
    int search_neighbors;
    uint32_t seed;

    // Network topology
    std::vector<int> nodes_for_vars;
    std::vector<int> nodes_for_params;
    std::vector<int> nodes_for_output;

    // Simulation params
    float dt;
    float time_limit;
    float epsilon_term;
    int num_trajectories;
    std::vector<float> qyminc;
    std::vector<float> qymaxc;
    std::string model_path;

    // Logging
    std::string output_csv;
    bool enable_logging;

    // Flexible fitness: ИСПРАВЛЕННЫЙ тип (void, заполняет Fu на месте)
    std::function<void(TArrReal&, NetOper&)> m_fitness_func;

    // Callbacks
    std::function<void(int gen, float avg_fitness)> on_generation_end;

public:
    GANOP(const GANOPConfig& config)
        : m_p(config.num_params), m_c(config.int_bits), m_d(config.frac_bits),
          m_lchr(config.num_struct_variations), m_HH(config.population_size),
          m_nfu(config.num_objectives), m_PP(config.num_generations),
          m_RR(config.num_crossovers_per_gen),
          mutation_prob(config.mutation_prob), selection_alpha(config.selection_alpha),
          search_neighbors(config.search_neighbors), seed(config.seed),
          nodes_for_vars(config.nodes_for_vars),
          nodes_for_params(config.nodes_for_params),
          nodes_for_output(config.nodes_for_output),
          dt(config.dt), time_limit(config.time_limit), epsilon_term(config.epsilon_term),
          num_trajectories(config.num_trajectories), qyminc(config.qyminc),
          qymaxc(config.qymaxc), model_path(config.model_path),
          output_csv(config.output_csv), enable_logging(config.enable_logging),
          m_fitness_func(config.fitness_func),  // <-- Теперь void-тип, совпадает с config
          on_generation_end(config.on_generation_end)
    {
        PopChrPar.assign(m_HH, TArrInt(m_p * (m_c + m_d)));
        PopChrStr.assign(m_HH, std::vector<TArrInt>(m_lchr));
        m_Fuh.assign(m_HH, TArrReal(m_nfu));
        m_Lh.assign(m_HH, 0);

        m_FuhSon.assign(4, TArrReal(m_nfu));
        m_LhSon.assign(4, 0);

        if (m_nfu <= 0) {
            throw std::invalid_argument("num_objectives must be > 0");
        }
        
    }

    // преобразование из кода Грея в вектор параметров: ИСПРАВЛЕНО signed/unsigned
    void GreyToVector(const std::vector<int>& y, NetOper& nop) {
        std::vector<int> m_zb;           
        int l = m_c + m_d;
        size_t lf1 = y.size();  // <-- size_t для безопасного сравнения

        if (m_zb.size() < lf1)
            m_zb.resize(lf1);

        // --- перевод из кода Грея в бинарный ---
        for (size_t i = 0; i < lf1; i++) {  // <-- size_t для i
            if (i % l == 0)
                m_zb[i] = y[i];
            else
                m_zb[i] = m_zb[i - 1] ^ y[i]; // XOR
        }

        int j = -1;
        double g1 = 1.0;
        double g = 1.0;

        // g1 = 2^(c-1)
        for (int i = 0; i < m_c - 1; i++)
            g1 *= 2.0;
       
        if (nop.get_parameters().size() < lf1 / l + 1) {
            nop.get_parameters().resize(static_cast<size_t>(lf1 / l + 1));
        }
        // --- перевод бинарных блоков в числа ---
        for (size_t i = 0; i < lf1; i++) {  // <-- size_t
            if (i % l == 0) {
                j++;
                nop.get_parameters()[j] = 0.0;
                g = g1;
            }
            nop.get_parameters()[j] += g * m_zb[i];
            g /= 2.0;
        }
    }

    // устанавливаем в параметры NOP
    void VectorToGrey(std::vector<int>& y, NetOper& nop) {
        std::vector<int> m_zb;           
        int totalBits = m_p * (m_c + m_d);

        if (y.size() < static_cast<size_t>(totalBits))
            y.resize(totalBits);

        if (m_zb.size() < static_cast<size_t>(totalBits))
            m_zb.resize(totalBits);

        std::fill(m_zb.begin(), m_zb.end(), 0);

        for (int j = 0; j < m_p; j++) {
            float param = nop.get_parameters()[j];
            if (param < 0.0f) {  // Обработка отрицательных
                param = std::abs(param);  // Или throw
            }
            int x = static_cast<int>(std::floor(param)); // целая часть
            double r = static_cast<double>(param - static_cast<float>(x)); // дробная часть

            // целая часть (c бит)
            int k = m_c + j * (m_c + m_d) - 1;
            while (k >= j * (m_c + m_d)) {
                m_zb[k] = x % 2;
                x /= 2;
                k--;
            }

            // дробная часть (d бит)
            k = m_c + j * (m_c + m_d);
            while (k < (m_c + m_d) * (j + 1)) {
                r *= 2.0;
                x = static_cast<int>(std::floor(r));
                m_zb[k] = x;
                r -= static_cast<float>(x);
                k++;
            }

            // преобразуем бинарный блок в код Грея
            y[j * (m_c + m_d)] = m_zb[j * (m_c + m_d)];
            for (int i = j * (m_c + m_d) + 1; i < (j + 1) * (m_c + m_d); i++) {
                y[i] = m_zb[i] ^ m_zb[i - 1];
            }
        }
    }

    int Rast(const TArrReal& Fu) const {
        int count = 0;

        for (int i = 0; i < m_HH; i++) {
            int j = 0;
            while (j < m_nfu && Fu[j] >= m_Fuh[i][j])
                j++;

            if (j >= m_nfu) {
                int k = 0;
                while (k < m_nfu && Fu[k] == m_Fuh[i][k])
                    k++;

                if (k < m_nfu) // не все равны
                    count++;
            }
        }

        return count;
    }

    void ChoosePareto() {
        Pareto.clear();
        for (int i = 0; i < m_HH; ++i) {
            if (m_Lh[i] == 0)  // особь не доминирована
            {
                Pareto.push_back(i);
            }
        }
    }

    // Default fitness для робота (твой Func0): УБРАЛ debug-print, добавил поддержку любого m_nfu
    void DefaultRobotFitness(TArrReal& Fu, NetOper& net) {
        Model::State currState = {0.0f, 0.0f, 0.0f}; 
        Model model(currState, dt, model_path);
        
        Model::State goal = {0.0f, 0.0f, 0.0f};
        Controller controller(goal, net);

        Runner runner(model, controller); 
        runner.setGoal(goal);

        std::vector<Model::State> init_states; 

        int nGraphc = num_trajectories;
        
        for (int i = 0; i < nGraphc; ++i) {
            init_states.push_back(
                Model::State{   i & 4 ? qymaxc[0] : qyminc[0], 
                                i & 2 ? qymaxc[1] : qyminc[1], 
                                i & 1 ? qymaxc[2] : qyminc[2] }
                );
        }

        float sumt = 0.0f;
        float sumdelt = 0.0f;
        float sum_path = 0.0f;

        for (int i = 0; i < nGraphc; ++i) {
            runner.init(init_states[i]);
            float currTime = 0.0f;
            float path_length = 0.0f;
            Model::State prevState = init_states[i];
            while (currTime < time_limit) {
                currState = runner.makeStep();
                float dx = currState.x - prevState.x;
                float dy = currState.y - prevState.y;
                path_length += std::sqrt(dx * dx + dy * dy);
                prevState = currState;
                currTime += dt;
                if (currState.dist(goal) < epsilon_term)
                    break; 
            }
            sumt += currTime;
            sumdelt += currState.dist(goal);
            sum_path += path_length;
        }

        Fu.resize(static_cast<size_t>(m_nfu));  // <-- Авто-ресайз под m_nfu
        Fu[0] = sumt;
        Fu[1] = sumdelt * 2.0f;
        Fu[2] = sum_path;
        if (m_nfu > 3) {
            Fu[3] = sum_path * 1.5f + sumt + sumdelt * 7.0f;
        }
        // Если m_nfu > 4, заполни остальное (например, 0.0f или дублируй)
        for (size_t i = 4; i < static_cast<size_t>(m_nfu); ++i) {
            Fu[i] = 0.0f;
        }
    }

    // Общая оценка fitness: ИСПРАВЛЕНА для void-функции
    void EvaluateFitness(TArrReal& Fu, NetOper& net) {
        if (m_fitness_func) {
            m_fitness_func(Fu, net);  // <-- Вызов void (заполняет Fu)
        } else {
            DefaultRobotFitness(Fu, net);
        }
        if (Fu.size() != static_cast<size_t>(m_nfu)) {
            throw std::runtime_error("Fitness func must fill vector of size " + std::to_string(m_nfu));
        }
    }

    // Приближённое обновление Lh (только для imax + 10 случайных)
    void UpdateLhLocally(int imax) {
        m_Lh[imax] = Rast(m_Fuh[imax]);
        std::mt19937 gen(seed ^ static_cast<uint32_t>(imax));  // Deterministic per imax
        std::uniform_int_distribution<int> dist(0, m_HH - 1);
        for (int k = 0; k < 10; ++k) {
            int idx = dist(gen);
            if (idx != imax) {
                m_Lh[idx] = Rast(m_Fuh[idx]);
            }
        }
    }

    void GenAlgorithm() {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<float> distReal(0.0f, 1.0f);
        std::uniform_int_distribution<int> distBit(0, 1);
        std::uniform_int_distribution<int> distPop(0, m_HH - 1);
        std::uniform_int_distribution<int> distLchr(0, m_lchr - 1);
        std::uniform_int_distribution<int> distP(0, m_p * (m_c + m_d) - 1);
        
        NetOper NOP;
        NOP.setCs(qc);                       
        NOP.setPsi(NopPsiN);                 
        NOP.setNodesForVars(nodes_for_vars);      
        NOP.setNodesForParams(nodes_for_params);    
        NOP.setNodesForOutput(nodes_for_output);
        
        VectorToGrey(PopChrPar[0], NOP);

        // create initial population
        for (int i = 1; i < m_HH; ++i) {
            for (int j = 0; j < m_lchr; ++j) {
                NOP.GenVar(PopChrStr[i][j]);
            }
            for (int j = 0; j < m_p * (m_c + m_d); ++j) {
                PopChrPar[i][j] = distBit(gen);
            }    
        }
        
        // вычисление функционала для популяции
        for (int i = 0; i < m_HH; ++i) {
            NOP.setPsi(NopPsiN); 
            for (int j = 0; j < m_lchr; ++j)
                NOP.Variations(PopChrStr[i][j]);

            GreyToVector(PopChrPar[i], NOP);
            EvaluateFitness(m_Fuh[i], NOP); 
        }

        // --- Расчёт расстояний до Парето ---
        for (int i = 0; i < m_HH; ++i) {    
            m_Lh[i] = Rast(m_Fuh[i]);
        }

        // Вызов колбэка для поколения 0: ИСПРАВЛЕНО на m_nfu-1 (общий случай)
        float sum_fitness_init = 0.0f;
        for (int i = 0; i < m_HH; ++i) {
            if (m_nfu > 0) sum_fitness_init += m_Fuh[i][m_nfu - 1];  // <-- Композит: последний
        }
        float avg_fitness_init = m_HH > 0 ? sum_fitness_init / static_cast<float>(m_HH) : 0.0f;
        if (on_generation_end) {
            on_generation_end(0, avg_fitness_init);
        }

        int pt = 1;  
        float alfa = selection_alpha;
        float pmut = mutation_prob;
        int ksearch = search_neighbors;

        while (pt <= m_PP) {
            std::cout << pt << " / " << m_PP << std::endl;
            for (int rt = 0; rt < m_RR; ++rt) {
                // --- Выбор двух родителей ---
                int k1 = distPop(gen);
                int lhmin = m_Lh[k1];
                for (int i = 0; i < ksearch; ++i) {
                    int ks = distPop(gen);
                    if (m_Lh[ks] < lhmin) {
                        k1 = ks;
                        lhmin = m_Lh[ks];
                    }
                }
                int k2 = distPop(gen);

                float ksi = distReal(gen);

                if (ksi < (1 + alfa * m_Lh[k1]) / (1 + m_Lh[k1]) || ksi < (1 + alfa * m_Lh[k2]) / (1 + m_Lh[k2])) {
                    // --- Кроссовер ---
                    int ks1 = distLchr(gen);
                    int ks2 = distP(gen);

                    std::vector<std::vector<int>> SonStr[4];
                    std::vector<int> SonPar[4];
                    for (int s = 0; s < 4; ++s) {
                        SonStr[s].resize(m_lchr);
                        for (int i = 0; i < m_lchr; ++i)
                            SonStr[s][i] = (s < 2 ? PopChrStr[k1][i] : PopChrStr[k2][i]);
                        SonPar[s].resize(m_p * (m_c + m_d));
                    }
                    
                    // Кроссовер параметров
                    for (int i = 0; i < ks2; ++i) {
                        SonPar[0][i] = PopChrPar[k1][i];
                        SonPar[1][i] = PopChrPar[k2][i];
                        SonPar[2][i] = PopChrPar[k1][i];
                        SonPar[3][i] = PopChrPar[k2][i];
                    }
                    for (int i = ks2; i < m_p * (m_c + m_d); ++i) {
                        SonPar[0][i] = PopChrPar[k2][i];
                        SonPar[1][i] = PopChrPar[k1][i];
                        SonPar[2][i] = PopChrPar[k2][i];
                        SonPar[3][i] = PopChrPar[k1][i];
                    }

                    // Кроссовер структур
                    for (int i = 0; i < ks1; ++i) {
                        SonStr[2][i] = PopChrStr[k1][i];
                        SonStr[3][i] = PopChrStr[k2][i];
                    }
                    for (int i = ks1; i < m_lchr; ++i) {
                        SonStr[2][i] = PopChrStr[k2][i];
                        SonStr[3][i] = PopChrStr[k1][i];
                    }

                    // --- Мутация и расчёт функционала для каждого сына ---
                    for (int s = 0; s < 4; ++s) {
                        if (distReal(gen) < pmut) {
                            SonPar[s][distP(gen)] = distBit(gen);
                            NOP.GenVar(SonStr[s][distLchr(gen)]);
                        }
                        

                        NOP.setPsi(NopPsiN); 
                        for (int j = 0; j < m_lchr; ++j)
                            NOP.Variations(SonStr[s][j]);
                        GreyToVector(SonPar[s], NOP);
                        EvaluateFitness(m_FuhSon[s], NOP); 
                        m_LhSon[s] = Rast(m_FuhSon[s]);
                        
                        // --- Замена хромосомы с наибольшим Lh ---
                        int Lmax = m_Lh[0];
                        int imax = 0;
                        for (int i = 1; i < m_HH; ++i)
                            if (m_Lh[i] > Lmax) {
                                Lmax = m_Lh[i];
                                imax = i;
                            }
                        if (m_LhSon[s] < Lmax) {
                            PopChrStr[imax] = SonStr[s];
                            PopChrPar[imax] = SonPar[s];
                            m_Fuh[imax] = m_FuhSon[s];
                            UpdateLhLocally(imax);  // Приближённое обновление
                        }
                    } 
                } 
            } 

            // Вызов колбэка для текущего поколения: ИСПРАВЛЕНО на m_nfu-1
            float sum_fitness = 0.0f;
            for (int i = 0; i < m_HH; ++i) {
                if (m_nfu > 0) sum_fitness += m_Fuh[i][m_nfu - 1];  // <-- Композит: последний
            }
            float avg_fitness = m_HH > 0 ? sum_fitness / static_cast<float>(m_HH) : 0.0f;
            if (on_generation_end) {
                on_generation_end(pt, avg_fitness);
            }

            // --- Эпоха закончена, обновление базиса и элита ---
            ChoosePareto();
            ++pt;
        } 

        // Pareto solutions
        // for (int idx : Pareto) {
        //     std::cout << "Pareto solution index: " << idx << "\n";
        //     std::cout << "Params: ";
        //     for (int p : PopChrPar[idx]) std::cout << p << " ";
        //     std::cout << "\nObjective values: ";
        //     for (float f : m_Fuh[idx]) std::cout << f << " ";
        //     std::cout << "\n\n";
        // }

        // --- Select and apply best Pareto solution ---
        // if (Pareto.empty()) {
        //     std::cerr << "No Pareto solutions found!" << std::endl;
        //     return;
        // }

        // Best by composite Fu[m_nfu-1]
        int best_idx = Pareto[0];
        float min_sumt = m_Fuh[best_idx][m_nfu - 1];  // <-- Общий случай
        for (int idx : Pareto) {
            if (m_Fuh[idx][m_nfu - 1] < min_sumt) {
                min_sumt = m_Fuh[idx][m_nfu - 1];
                best_idx = idx;
            }
        }

        std::cout << "Applying best Pareto solution (index: " << best_idx << ", sumt: " << min_sumt << ")" << std::endl;

        // Configure NOP with the best solution
        NOP.setPsi(NopPsiN);
        for (int j = 0; j < m_lchr; ++j) {
            NOP.Variations(PopChrStr[best_idx][j]);
        }
        GreyToVector(PopChrPar[best_idx], NOP);

        // Simulate trajectories for the best solution and log to file (if enabled)
        if (enable_logging) {
            std::ofstream outFile(output_csv);
            if (!outFile.is_open()) {
                std::cerr << "Failed to open " << output_csv << " for writing!" << std::endl;
                return;
            }

            // Write header
            outFile << "Trajectory,Time,X,Y,Theta\n";

            Model::State currState = {0.0f, 0.0f, 0.0f};
            Model model(currState, dt, model_path);
            Model::State goal = {0.0f, 0.0f, 0.0f};
            Controller controller(goal, NOP);
            Runner runner(model, controller);
            runner.setGoal(goal);

            std::vector<Model::State> init_states;
            int nGraphc = num_trajectories;

            for (int i = 0; i < nGraphc; ++i) {
                init_states.push_back(
                    Model::State{
                        i & 4 ? qymaxc[0] : qyminc[0],
                        i & 2 ? qymaxc[1] : qyminc[1],
                        i & 1 ? qymaxc[2] : qyminc[2]
                    }
                );
            }

            float timeLimit_local = time_limit;
            float epsterm_local = epsilon_term;

            // Simulate each trajectory and log
            for (int i = 0; i < nGraphc; ++i) {
                runner.init(init_states[i]);
                float currTime = 0.0f;
                while (currTime < timeLimit_local) {
                    currState = runner.makeStep();
                    outFile << i << "," << currTime << "," << currState.x << "," << currState.y << "," << currState.yaw << "\n";
                    currTime += dt;
                    if (currState.dist(goal) < epsterm_local)
                        break;
                }
            }

            outFile.close();
            std::cout << "Trajectories logged to " << output_csv << std::endl;
        }

        // Print final matrix for debugging
        NOP.printMatrix();

        for (auto i : NOP.get_parameters())
            std::cout << i << " ";
        std::cout << std::endl;
    }   
};