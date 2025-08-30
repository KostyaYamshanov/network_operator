#pragma once

#include "nop.hpp"
#include <vector>
#include <cmath>
#include <random>

using TArrInt = std::vector<int>;
using TArrReal = std::vector<float>;

// TEST
float targetFunction(float x, float q) {
    return std::sin(x) + std::cos(q * x);
}



static std::vector<float>get_target_values()
{
    std::vector<float> result;
    float q = 2.5f; // Фиксированный параметр q
    float x = -100.0;
    // std::cout<<"Expected Y"<<std::endl;
    for (int i = 0; i < 1000; i++)
    {
        float y_expected = targetFunction(x,q);
        result.push_back(y_expected);
        x = x + 0.2;
        // std::cout<<y_expected<<" ";
    }
    // std::cout<<std::endl;

    return result;
}

float computeRMSE(const std::vector<float>& y_out, const std::vector<float>& y_ref) {
    if (y_out.size() != y_ref.size()) {
        throw std::invalid_argument("Vectors must have the same size");
    }
    float sum_squares = 0.0f;
    for (size_t i = 0; i < y_out.size(); ++i) {
        float diff = y_out[i] - y_ref[i];
        sum_squares += diff * diff;
    }
    return std::sqrt(sum_squares / y_out.size());
}

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
    int m_PP = 128; // число поколений
    int m_RR = 128; // число кроссоверов 

    // std::vector<double> m_q;         // parameters
    // std::vector<int> m_zb;           // additional vector

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
        // std::cout<<"GreyToVector"<<std::endl;
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

        // TEST
        // fix
        // nop.get_parameters().resize(m_p);

        // g1 = 2^(c-1)
        for (int i = 0; i < m_c - 1; i++)
            g1 *= 2.0;

        // fix
        // if (nop.get_parameters().size() < static_cast<size_t>(lf1 / l))
        //     nop.get_parameters().resize(lf1 / l);
        
        // bug
        if (nop.get_parameters().size() < lf1 / l + 1)
        {
            // std::cout<<"resize"<<std::endl;
            nop.get_parameters().resize(lf1 / l + 1);
        }
        // --- перевод бинарных блоков в числа ---
        // std::cout<<"Bin to num"<<std::endl;
        for (int i = 0; i < lf1; i++)
        {
            if (i % l == 0)
            {
                j++;
                // std::cout<<" IF j="<< j <<" "<<nop.get_parameters().size()<<std::endl;
                nop.get_parameters()[j] = 0.0;
                g = g1;
            }
            // std::cout<<" NOT IF j="<< j <<" "<<nop.get_parameters().size()<<std::endl;
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
            double r = nop.get_parameters()[j] - x;                        // дробная часть

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
                r -= x;
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
        // std::cout<<"ChoosePareto"<<std::endl;
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
        // std::cout<<"FUNCTOINAL CALCULATE START"<<std::endl;
        static std::vector<float> y_expected = get_target_values();

        std::vector<float> output = {0};
        float x = -100.0;
        std::vector<float> y_current;
        // std::cout<<"current"<<std::endl;
        for (int i = 0; i < 1000; i++)
        {
            net.calcResult({x}, output);
            x = x + 0.2;
            // std::cout<<output.size()<<" <-- SIZE output "<<std::endl; 
            y_current.push_back(output[0]);
            // std::cout<<output[0]<<" "; 
        }
        // std::cout<<std::endl;
        Fu[0] = computeRMSE(y_expected, y_current);
        Fu[1] = Fu[0];
        // std::cout<<"FUNCTOINAL CALCULATE END"<<std::endl;
    }

    void GenAlgorithm()
    {
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<float> distReal(0.0f, 1.0f);
        std::uniform_int_distribution<int> distBit(0, 1);
        std::uniform_int_distribution<int> distPop(0, m_HH-1);
        std::uniform_int_distribution<int> distLchr(0, m_lchr-1);
        std::uniform_int_distribution<int> distP(0, m_p*(m_c+m_d)-1);
        
        // NOP.setNodesForVars({0, 1, 2});      // Pnum
        // NOP.setNodesForParams({3, 4, 5});    // Rnum
        // NOP.setNodesForOutput({22, 23});     // Dnum
        
        auto NOP = NetOper();
        NOP.setNodesForVars({0});      // Pnum
        NOP.setNodesForParams({1});    // Rnum
        NOP.setNodesForOutput({13});     // Dnum

        NOP.setCs(qc);                       // set Cs
        NOP.setPsi(NopPsiN);                 // set matrix
        // std::cout<<"Matrix after variations"<<std::endl;
        // NOP.printMatrix();
        
        // fix
        VectorToGrey(PopChrPar[0], NOP);

        // create inital population
        for(int i=1; i<m_HH; ++i)
        {
            std::cout<<"---- variations ------"<< std::endl;
            for(int j=0; j<m_lchr; ++j)
            {
                NOP.GenVar(PopChrStr[i][j]);
                // debug
                for (const auto& item : PopChrStr[i][j])
                    std::cout<<item<<" ";
                std::cout<<std::endl;
            }

            std::cout<<"---- params ------" <<std::endl;
            
            for(int j=0; j<m_p*(m_c+m_d); ++j)
            {
                PopChrPar[i][j] = distBit(gen);
                std::cout<<PopChrPar[i][j]<<" ";
            }    
            std::cout<<std::endl;  
        }
        // вычисление функционала для популяции
        for(int i=0; i<m_HH; ++i)
        {
            NOP.setPsi(NopPsiN); // начальная матрица
            for(int j=0; j<m_lchr; ++j)
                NOP.Variations(PopChrStr[i][j]);

            // std::cout<<"Matrix after variations"<<std::endl;
            // NOP.printMatrix();
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
        float alfa = 0.4;
        float pmut = 0.7;
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

        for (int idx : Pareto) {
            std::cout << "Pareto solution index: " << idx << "\n";
            std::cout << "Params: ";
            for (int p : PopChrPar[idx]) std::cout << p << " ";
            std::cout << "\nObjective values: ";
            for (float f : m_Fuh[idx]) std::cout << f << " ";
            std::cout << "\n\n";
        }


    }


};
