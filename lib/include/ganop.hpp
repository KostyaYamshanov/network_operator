#pragma once

#include "nop.hpp"
#include <vector>
#include <cmath>
#include <random>

using TArrInt = std::vector<int>;
using TArrReal = std::vector<float>;

// TEST
float targetFunction(float x) {
    return std::sin(x) + std::cos(3 * x);
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

    // std::vector<double> m_q;         // parameters
    // std::vector<int> m_zb;           // additional vector

    std::vector<TArrReal> m_Fuh; // значения функций
    std::vector<int> m_Lh;       // расстояния

    std::vector<TArrReal> m_FuhSon;
    std::vector<int> m_LhSon;

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

        if (nop.get_parameters().size() < lf1 / l + 1)
            nop.get_parameters().resize(lf1 / l + 1);

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
        if (y.size() < static_cast<size_t>(totalBits))
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

    int Rast(const std::vector<double>& Fu) const
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

    // procedure TGANOP.Func0(var Fu: TArrReal);
    // var
    //   i:integer;
    // Begin
    //   NOP.RPControl;
    //   for i:=0 to nfu-1 do
    //     Fu[i]:=NOP.z[NOP.Dnum[i]];
    // End;

    void Func0(std::vector<float>& Fu, NetOper& net) {
        std::vector<float> output(2, 0);
        std::vector<float> test_input = {1,1,1};
        net.calcResult(test_input, output);

        const auto& z = net.get_z();
        const auto& outs = net.getNodesForOutput();

        // Fu.resize(outs.size());
        for (size_t i = 0; i < m_nfu; ++i)
            Fu[i] = z[outs[i]];
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
        NOP.setNodesForVars({0, 1, 2});      // Pnum
        NOP.setNodesForParams({3, 4, 5});    // Rnum
        // NOP.setNodesForOutput({22, 23});     // Dnum
        NOP.setNodesForOutput({13, 14});     // Dnum

        NOP.setCs(qc);                       // set Cs
        NOP.setPsi(NopPsiN);                 // set matrix
        std::cout<<"Matrix after variations"<<std::endl;
        NOP.printMatrix();
        
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

            std::cout<<"Matrix after variations"<<std::endl;
            NOP.printMatrix();
            GreyToVector(PopChrPar[i], NOP);
            Func0(m_Fuh[i], NOP); // функция рассчет функционала
        }

    }



    // void setQ(const std::vector<double>& q) { m_q = q; }
    // const std::vector<double>& getQ() const { return m_q; }

    // void setFuh(const std::vector<std::vector<double>>& Fuh) { m_Fuh = Fuh; }
    // const std::vector<std::vector<double>>& getFuh() const { return m_Fuh; }
    
    // const std::vector<int>& getZb() const { return m_zb; }

};
