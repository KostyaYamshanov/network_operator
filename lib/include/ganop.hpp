#pragma once

#include "nop.hpp"
#include <vector>
#include <cmath>
#include <random>
#include "runner.hpp"

using TArrInt = std::vector<int>;
using TArrReal = std::vector<float>;

class GANOP
{

public:

    // NetOper NOP;

    // --- популяция ---
    std::vector<TArrInt> PopChrPar;              // параметры 
    std::vector<std::vector<TArrInt>> PopChrStr; // структры

    int m_p;   // число параметров
    int m_c;   // число бит в целой части
    int m_d;   // число бит в плавающей части
    int m_nfu; // размер Fu
    int m_HH;  // размер популяции
    int m_lchr;  // количества вариаций в одном решении
    int m_PP = 24; // число поколений
    int m_RR = 128; // число кроссоверов 

    std::vector<TArrReal> m_Fuh; // значения функций
    std::vector<int> m_Lh;       // расстояния

    std::vector<TArrReal> m_FuhSon;
    std::vector<int> m_LhSon;
    std::vector<int> Pareto;          // индексы Парето-оптимальных

public:
    GANOP(int _p, int _c, int _d, int _lchr, int _HH, int _nfu)
        : m_p(_p), m_c(_c), m_d(_d), m_lchr(_lchr), m_HH(_HH), m_nfu(_nfu)
    {
        PopChrPar.assign(m_HH, TArrInt(m_p * (m_c + m_d)));
        PopChrStr.assign(m_HH, std::vector<TArrInt>(m_lchr));
        m_Fuh.assign(m_HH, TArrReal(m_nfu));
        m_Lh.assign(m_HH, 0);

        m_FuhSon.assign(4, TArrReal(m_nfu));
        m_LhSon.assign(4, 0);
    }

    // преобразование из кода Грея в вектор параметров
    void GreyToVector(const std::vector<int>& y, NetOper& nop)
    {
        std::vector<int> m_zb;           // additional vector

        int l = m_c + m_d;
        int lf1 = static_cast<int>(y.size());

        if (m_zb.size() < lf1)
            m_zb.resize(lf1);

        // --- перевод из кода Грея в бинарный ---
        for (int i = 0; i < lf1; i++)
        {
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
       
        // bug
        if (nop.get_parameters().size() < lf1 / l + 1)
        {
            nop.get_parameters().resize(lf1 / l + 1);
        }
        // --- перевод бинарных блоков в числа ---
        for (int i = 0; i < lf1; i++)
        {
            if (i % l == 0)
            {
                j++;
                nop.get_parameters()[j] = 0.0;
                g = g1;
            }
            nop.get_parameters()[j] += g * m_zb[i];
            g /= 2.0;
        }
    }

    // тут ведь устанавливаем в параметры NOP
    void VectorToGrey(std::vector<int>& y, NetOper& nop)
    {
        std::vector<int> m_zb;           // additional vector

        int totalBits = m_p * (m_c + m_d);

        // убедимся, что размеры совпадают
        if (y.size() < static_cast<size_t>(totalBits)) // remove for tests
            y.resize(totalBits);

        if (m_zb.size() < static_cast<size_t>(totalBits))
            m_zb.resize(totalBits);

        // обнуление zb
        std::fill(m_zb.begin(), m_zb.end(), 0);

        for (int j = 0; j < m_p; j++)
        {
            int x = static_cast<int>(std::floor(nop.get_parameters()[j])); // целая часть
            double r = static_cast<double>(nop.get_parameters()[j] - static_cast<float>(x)); // дробная часть

            // целая часть (c бит)
            int k = m_c + j * (m_c + m_d) - 1;
            while (k >= j * (m_c + m_d))
            {
                m_zb[k] = x % 2;
                x /= 2;
                k--;
            }

            // дробная часть (d бит)
            k = m_c + j * (m_c + m_d);
            while (k < (m_c + m_d) * (j + 1))
            {
                r *= 2.0;
                x = static_cast<int>(std::floor(r));
                m_zb[k] = x;
                r -= static_cast<float>(x);
                k++;
            }

            // преобразуем бинарный блок в код Грея
            y[j * (m_c + m_d)] = m_zb[j * (m_c + m_d)];
            for (int i = j * (m_c + m_d) + 1; i < (j + 1) * (m_c + m_d); i++)
            {
                y[i] = m_zb[i] ^ m_zb[i - 1];
            }
        }
    }

    int Rast(const std::vector<float>& Fu) const
    {
        int count = 0;

        for (int i = 0; i < m_HH; i++)
        {
            int j = 0;
            while (j < m_nfu && Fu[j] >= m_Fuh[i][j])
                j++;

            if (j >= m_nfu)
            {
                int k = 0;
                while (k < m_nfu && Fu[k] == m_Fuh[i][k])
                    k++;

                if (k < m_nfu) // значит, не все равны
                    count++;
            }
        }

        return count;
    }

    void ChoosePareto()
    {
        Pareto.clear();
        for (int i = 0; i < m_HH; ++i)
        {
            if (m_Lh[i] == 0)  // особь не доминирована
            {
                Pareto.push_back(i);
            }
        }
    }

    void Func0(std::vector<float>& Fu, NetOper& net) {
        constexpr float dt = 0.033333;
        Model::State currState = {0.0, 0.0, 0.0}; 
        Model model(currState, dt, "rosbot_gazebo9_2d_model.onnx");
        
        Model::State goal = {0.0, 0.0, 0.0};
        Controller controller(goal, net);

        Runner runner(model, controller); 
        runner.setGoal(goal);

        std::vector<Model::State> init_states; 

        int nGraphc = 8; // num of graphs

        // std::vector<float> qyminc = {-3.5,-3,5,-1.31};
        std::vector<float> qyminc = {-3.5,-3,5,-1.31};
        std::vector<float> qymaxc = { 3.5, 3.5, 1.31};
        
        for (int i = 0; i < nGraphc; ++i) {
            init_states.push_back(
                Model::State{   i & 4? qymaxc[0] : qyminc[0], 
                                i & 2? qymaxc[1] : qyminc[1], 
                                i & 1? qymaxc[2] : qyminc[2] }
                );
        }

        float timeLimit = 30.0;          
        float epsterm = 0.1;
        float sumt = 0.0;
        float sumdelt = 0.0;
        float sum_path = 0.0;

        for (int i = 0; i <= nGraphc - 1; ++i) {
            runner.init(init_states[i]);
            float currTime = 0;
            float path_length = 0.0;
            Model::State prevState = init_states[i];
            while (currTime < timeLimit) {
                currState = runner.makeStep();
                float dx = currState.x - prevState.x;
                float dy = currState.y - prevState.y;
                path_length += std::sqrt(dx * dx + dy * dy);
                prevState = currState;
                // currState.print();
                currTime += dt;
                if (currState.dist(goal) < epsterm)
                    break; 
            }
            sumt += currTime;
            sumdelt += currState.dist(goal);
            sum_path += path_length;
        }

        Fu[0] = sumt * 2.0;
        Fu[1] = sumdelt * 5.0;
        Fu[2] = sum_path;
    }

    void GenAlgorithm()
    {
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<float> distReal(0.0f, 1.0f);
        std::uniform_int_distribution<int> distBit(0, 1);
        std::uniform_int_distribution<int> distPop(0, m_HH-1);
        std::uniform_int_distribution<int> distLchr(0, m_lchr-1);
        std::uniform_int_distribution<int> distP(0, m_p*(m_c+m_d)-1);
        
        
        auto NOP = NetOper();
        // for simple task 
        // NOP.setNodesForVars({0});      // Pnum
        // NOP.setNodesForParams({1,2,3});    // Rnum
        // NOP.setNodesForOutput({13});     // Dnum

        NOP.setCs(qc);                       // set Cs
        NOP.setPsi(NopPsiN);                 // set matrix

        NOP.setNodesForVars({0, 1, 2});      // Pnum
        NOP.setNodesForParams({3, 4, 5});    // Rnum
        NOP.setNodesForOutput({22, 23});     // Dnum

        // std::cout<<"Matrix after variations"<<std::endl;
        // NOP.printMatrix();
        
        VectorToGrey(PopChrPar[0], NOP);

        // create inital population
        for(int i=1; i<m_HH; ++i)
        {
            // std::cout<<"---- variations ------"<< std::endl;
            for(int j=0; j<m_lchr; ++j)
            {
                NOP.GenVar(PopChrStr[i][j]);
                // debug
                // for (const auto& item : PopChrStr[i][j])
                //     std::cout<<item<<" ";
                // std::cout<<std::endl;
            }

            // std::cout<<"---- params ------" <<std::endl;
            
            for(int j=0; j<m_p*(m_c+m_d); ++j)
            {
                PopChrPar[i][j] = distBit(gen);
                // std::cout<<PopChrPar[i][j]<<" ";
            }    
            // std::cout<<std::endl;  
        }
        // вычисление функционала для популяции
        for(int i=0; i<m_HH; ++i)
        {
            NOP.setPsi(NopPsiN); // начальная матрица
            for(int j=0; j<m_lchr; ++j)
                NOP.Variations(PopChrStr[i][j]);

            // std::cout<<"Matrix after variations"<<std::endl;
            //
            GreyToVector(PopChrPar[i], NOP);

            Func0(m_Fuh[i], NOP); // функция рассчет функционала
        }

        // --- Расчёт расстояний до Парето ---
        //std::cout<<"Parreto distance"<<std::endl;
        for(int i=0; i<m_HH; ++i)
        {    
            m_Lh[i] = Rast(m_Fuh[i]);
            // std::cout<<m_Lh[i]<<" ";
        }
        // std::cout<<std::endl;

        int pt = 1;  // номер поколения
        int ksearch = 8;

        // float alfa = 0.4;
        // float pmut = 0.7;

        float alfa = 0.5;
        float pmut = 0.5;
        while(pt <= m_PP) // PP — число поколений
        {
            std::cout<<pt<<" / "<<m_PP<<std::endl;
            for(int rt=0; rt<m_RR; ++rt) // RR — число кроссоверов на поколение
            {
                // --- Выбор двух родителей ---
                // std::cout<<"Choose two parrents"<<std::endl;
                int k1 = distPop(gen);
                int lhmin = m_Lh[k1];
                for(int i=0; i<ksearch; ++i)
                {
                    int ks = distPop(gen);
                    if(m_Lh[ks] < lhmin)
                    {
                        k1 = ks;
                        lhmin = m_Lh[ks];
                    }
                }
                int k2 = distPop(gen);

                float ksi = distReal(gen);
                // std::cout<<" k1 "<<k1<<" k2 "<<k2<<" lhmin "<<lhmin<<" ksi "<<ksi<<std::endl;

                if(ksi < (1+alfa*m_Lh[k1])/(1+m_Lh[k1]) || ksi < (1+alfa*m_Lh[k2])/(1+m_Lh[k2]))
                {
                    // --- Кроссовер ---
                    // std::cout<<"Crossover"<<std::endl;
                    int ks1 = distLchr(gen);
                    int ks2 = distP(gen);

                    std::vector<std::vector<int>> SonStr[4];
                    std::vector<int> SonPar[4];
                    for(int s=0; s<4; ++s)
                    {
                        SonStr[s].resize(m_lchr);
                        for(int i=0; i<m_lchr; ++i)
                            SonStr[s][i] = (s<2 ? PopChrStr[k1][i] : PopChrStr[k2][i]);
                        SonPar[s].resize(m_p*(m_c+m_d));
                    }
                    // std::cout<<""<<std::endl;
                    // Кроссовер параметров
                    // std::cout<<"start parameters crossover"<<std::endl;
                    for(int i=0; i<ks2; ++i)
                    {
                        SonPar[0][i] = PopChrPar[k1][i];
                        SonPar[1][i] = PopChrPar[k2][i];
                        SonPar[2][i] = PopChrPar[k1][i];
                        SonPar[3][i] = PopChrPar[k2][i];
                    }
                    for(int i=ks2; i<m_p*(m_c+m_d); ++i)
                    {
                        SonPar[0][i] = PopChrPar[k2][i];
                        SonPar[1][i] = PopChrPar[k1][i];
                        SonPar[2][i] = PopChrPar[k2][i];
                        SonPar[3][i] = PopChrPar[k1][i];
                    }

                    // Кроссовер структур
                    // std::cout<<"Crossover of structures"<<std::endl;
                    for(int i=0; i<ks1; ++i)
                    {
                        SonStr[2][i] = PopChrStr[k1][i];
                        SonStr[3][i] = PopChrStr[k2][i];
                    }
                    for(int i=ks1; i<m_lchr; ++i)
                    {
                        SonStr[2][i] = PopChrStr[k2][i];
                        SonStr[3][i] = PopChrStr[k1][i];
                    }

                    // std::cout<<"Mutations"<<std::endl;
                    // --- Мутация и расчёт функционала для каждого сына ---
                    for(int s=0; s<4; ++s)
                    {
                        if(distReal(gen) < pmut)
                        {
                            // std::cout<<"if(distReal(gen) < pmut)"<<std::endl;
                            SonPar[s][distP(gen)] = distBit(gen);
                            NOP.GenVar(SonStr[s][distLchr(gen)]);
                        }
                        

                        NOP.setPsi(NopPsiN); // базис
                        for(int j=0; j<m_lchr; ++j)
                            NOP.Variations(SonStr[s][j]);
                        GreyToVector(SonPar[s], NOP);
                        Func0(m_FuhSon[s], NOP); // функционалы для сына
                        m_LhSon[s] = Rast(m_FuhSon[s]);
                        // std::cout<<"cost function for son calculated"<<std::endl;
                        // --- Замена хромосомы с наибольшим Lh ---
                        // std::cout<<"chromosome changes for big Lh"<<std::endl;
                        int Lmax = m_Lh[0];
                        int imax = 0;
                        for(int i=1; i<m_HH; ++i)
                            if(m_Lh[i] > Lmax)
                            {
                                Lmax = m_Lh[i];
                                imax = i;
                            }
                        // std::cout<<"chromosome changed with big Lh"<<std::endl;
                        if(m_LhSon[s] < Lmax)
                        {
                            PopChrStr[imax] = SonStr[s];
                            PopChrPar[imax] = SonPar[s];
                            m_Fuh[imax] = m_FuhSon[s];
                            for(int i=0; i<m_HH; ++i)
                                m_Lh[i] = Rast(m_Fuh[i]);
                        }
                    } // конец цикла по сынам
                    // std::cout<<"End cicle for suns"<<std::endl;
                } // конец условия по ksi
            } // конец кроссоверов
            // std::cout<<"Crossover end"<<std::endl;

            // --- Эпоха закончена, обновление базиса и элита ---
            ChoosePareto();
            ++pt;
        } // конец поколений
        int optimal_idx = 0;
        for (int idx : Pareto) {
            optimal_idx = idx;
            std::cout << "Pareto solution index: " << idx << "\n";
            std::cout << "Params: ";
            for (int p : PopChrPar[idx]) std::cout << p << " ";
            std::cout << "\nObjective values: ";
            for (float f : m_Fuh[idx]) std::cout << f << " ";
            std::cout << "\n\n";
        }

            // --- Select and apply best Pareto solution ---
        if (Pareto.empty()) {
            std::cerr << "No Pareto solutions found!" << std::endl;
            return;
        }

        // Find the Pareto solution with the lowest sumt (m_Fuh[i][0])
        int best_idx = Pareto[0];
        float min_sumt = m_Fuh[best_idx][0];
        for (int idx : Pareto) {
            if (m_Fuh[idx][0] < min_sumt) {
                min_sumt = m_Fuh[idx][0];
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

        // Simulate trajectories for the best solution and log to file
        std::ofstream outFile("trajectories.csv");
        if (!outFile.is_open()) {
            std::cerr << "Failed to open trajectories.csv for writing!" << std::endl;
            return;
        }

        // Write header
        outFile << "Trajectory,Time,X,Y,Theta\n";

        // Initialize simulation parameters (same as in Func0)
        constexpr float dt = 0.03333;
        Model::State currState = {0.0, 0.0, 0.0};
        Model model(currState, dt, "rosbot_gazebo9_2d_model.onnx");
        Model::State goal = {0.0, 0.0, 0.0};
        Controller controller(goal, NOP);
        Runner runner(model, controller);
        runner.setGoal(goal);

        std::vector<Model::State> init_states;
        int nGraphc = 8;
        // 1.31
        std::vector<float> qyminc = {-3.5, -3.5, -1.31};
        std::vector<float> qymaxc = {3.5, 3.5, 1.31};

        for (int i = 0; i < nGraphc; ++i) {
            init_states.push_back(
                Model::State{
                    i & 4 ? qymaxc[0] : qyminc[0],
                    i & 2 ? qymaxc[1] : qyminc[1],
                    i & 1 ? qymaxc[2] : qyminc[2]
                }
            );
        }

        float timeLimit = 30.0;
        float epsterm = 0.1;

        // Simulate each trajectory and log
        for (int i = 0; i < nGraphc; ++i) {
            runner.init(init_states[i]);
            float currTime = 0;
            while (currTime < timeLimit) {
                currState = runner.makeStep();
                // Log: trajectory number, time, x, y, theta
                outFile << i << "," << currTime << "," << currState.x << "," << currState.y << "," << currState.yaw << "\n";
                currTime += dt;
                if (currState.dist(goal) < epsterm)
                    break;
            }
        }

        outFile.close();
        std::cout << "Trajectories logged to trajectories.csv" << std::endl;

        // Print final matrix for debugging
        NOP.printMatrix();
    }
};

