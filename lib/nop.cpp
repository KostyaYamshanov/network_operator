#include "nop.hpp"
#include <iostream>
#include <random>


NetOper::NetOper()
{
    initUnaryFunctionsMap();
    initBinaryFunctionsMap();
}

float NetOper::getUnaryOperationResult(int operationNum, float input)
{
    return m_unaryFuncMap[operationNum](input);
}

float NetOper::getBinaryOperationResult(int operationNum, float left, float right)
{
    return m_binaryFuncMap[operationNum](left, right);
}

void NetOper::initUnaryFunctionsMap()
{
    m_unaryFuncMap[1] = ro_1;
    m_unaryFuncMap[2] = ro_2;
    m_unaryFuncMap[3] = ro_3;
    m_unaryFuncMap[4] = ro_4;
    m_unaryFuncMap[5] = ro_5;
    m_unaryFuncMap[6] = ro_6;
    m_unaryFuncMap[7] = ro_7;
    m_unaryFuncMap[8] = ro_8;
    m_unaryFuncMap[9] = ro_9;
    m_unaryFuncMap[10] = ro_10;
    m_unaryFuncMap[11] = ro_11;
    m_unaryFuncMap[12] = ro_12;
    m_unaryFuncMap[13] = ro_13;
    m_unaryFuncMap[14] = ro_14;
    m_unaryFuncMap[15] = ro_15;
    m_unaryFuncMap[16] = ro_16;
    m_unaryFuncMap[17] = ro_17;
    m_unaryFuncMap[18] = ro_18;
    m_unaryFuncMap[19] = ro_19;
    m_unaryFuncMap[20] = ro_20;
    m_unaryFuncMap[21] = ro_21;
    m_unaryFuncMap[22] = ro_22;
    m_unaryFuncMap[23] = ro_23;
    m_unaryFuncMap[24] = ro_24;
    m_unaryFuncMap[25] = ro_25;
    m_unaryFuncMap[26] = ro_26;
    m_unaryFuncMap[27] = ro_27;
    m_unaryFuncMap[28] = ro_28;
}

void NetOper::initBinaryFunctionsMap()
{
    m_binaryFuncMap[1] = xi_1;
    m_binaryFuncMap[2] = xi_2;
    m_binaryFuncMap[3] = xi_3;
    m_binaryFuncMap[4] = xi_4;
    m_binaryFuncMap[5] = xi_5;
    m_binaryFuncMap[6] = xi_6;
    m_binaryFuncMap[7] = xi_7;
    m_binaryFuncMap[8] = xi_8;
}

const std::vector<int>& NetOper::getNodesForVars()
{
    return m_nodesForVars;
}

void NetOper::setNodesForVars(const std::vector<int>& nodes)
{
    m_nodesForVars = nodes;
}

const std::vector<int>& NetOper::getNodesForParams()
{
    return m_nodesForParams;
}

void NetOper::setNodesForParams(const std::vector<int>& nodes)
{
    m_nodesForParams = nodes;
}

const std::vector<int>& NetOper::getNodesForOutput()
{
    return m_nodesForOutput;
}

void NetOper::setNodesForOutput(const std::vector<int>& nodes)
{
    m_nodesForOutput = nodes;
}

const std::vector<std::vector<int>>& NetOper::getPsi()
{
    return m_matrix;
}

const std::vector<float>& NetOper::getCs()
{
    return m_parameters;
}

void NetOper::setCs(const std::vector<float>& newParams)
{
    m_parameters = newParams;
}

void NetOper::setPsi(const std::vector<std::vector<int>>& newMatrix)
{
    // std::cout<<"set new PSI"<<std::endl;
    m_matrix = newMatrix;
    z.resize(m_matrix.size());
}
// ROControl
void NetOper::calcResult(const std::vector<float>& x_in, std::vector<float>& y_out)
{
    for(size_t i=0; i < m_matrix.size(); ++i)
    {
        if (m_matrix[i][i] == 2)
            z[i] = 1.0f;
        else if (m_matrix[i][i] == 3)
            z[i] = (-1.0f) * Infinity;
        else if (m_matrix[i][i] == 4)
            z[i] = Infinity;
        else
            z[i] = 0.0f;
    }

    for(size_t i=0; i < m_nodesForVars.size(); ++i)
    {
        // std::cout<<"test "<<x_in[i]<<std::endl;
        z[m_nodesForVars[i]] = x_in[i];
    }
    for (size_t i=0; i < m_nodesForParams.size(); ++i)
    {
        // std::cout<<"test_2 "<<m_parameters[i]<<std::endl;
        z[m_nodesForParams[i]] = m_parameters[i];
    }
    for(size_t i=0; i < m_matrix.size() - 1; ++i)
    {
        // std::cout<<"test_3"<<std::endl;
        for(size_t j=i+1; j < m_matrix.size(); ++j)
        {
            // std::cout<<"test_4"<<std::endl;
            if (m_matrix[i][j] == 0)
                continue;
            
            auto zz = getUnaryOperationResult(m_matrix[i][j], z[i]);
            // std::cout<<"test_5"<<std::endl;
            // std::cout<<"j = "<<j<<" z[j] = "<<z[j]<<" m_matrix[j][j]= "<<m_matrix[j][j]<<std::endl;
            // if (m_matrix[j][j] == 0)
            //     std::cout<<"SEGFAULT!!!!"<<std::endl;
            z[j] = getBinaryOperationResult(m_matrix[j][j], z[j], zz);
            // std::cout<<"test_6"<<std::endl;
        }
    }
    for(size_t i = 0; i < m_nodesForOutput.size(); ++i)
        y_out[i] = z[m_nodesForOutput[i]];
    
}

NOPMatrixReader& NetOper::getReader()
{
    return m_reader;
}

// todo update
std::vector<float>& NetOper::get_z()
{
    return z;
}

std::vector<float>& NetOper::get_parameters()
{
    return m_parameters;
}

bool NetOper::TestSource(int j)
{
    for (int node : m_nodesForVars)
        if (j == node) return false;

    for (int node : m_nodesForParams)
        if (j == node) return false;

    return true;
}

void NetOper::GenVar(std::vector<int>& w)
{
    // Элементарные операции
    if (w.size() < 4) w.resize(4);

    int L = static_cast<int>(m_matrix.size()); // количество узлов = размер Psi
    int kW = static_cast<int>(m_unaryFuncMap.size());
    int kV = static_cast<int>(m_binaryFuncMap.size());

    w[0] = rand() % 4; // random(4)

    switch (w[0])
    {
    case 0:
    case 2:
    case 3: // замена недиагонального элемента, добавление и удаление дуги
        w[1] = rand() % (L - 1);
        w[2] = rand() % (L - w[1] - 1) + w[1] + 1;
        // w[3] = (rand() % (kW - 1)) + 1; // Ensure w[3] is in [1, kW-1] // rand() % kW;
        w[3] = rand() % kW;
        if (w[3] == 0)
            w[3] = 1;
        break;

    case 1: // замена диагонального элемента
        w[1] = rand() % L;

        while (w[1] < L && !TestSource(w[1]))
            w[1]++;

        w[2] = w[1];
        // w[3] = (rand() % (kW - 1)) + 1; // Ensure w[3] is in [1, kW-1] // rand() % kV;
        w[3] = rand() % kV;
        if (w[3] == 0)
            w[3] = 1;
        break;
    }
}

// void NetOper::GenVar(std::vector<int>& w)
// {
//     static std::mt19937 gen(std::random_device{}());
//     std::uniform_int_distribution<int> dist4(0, 3);
//     std::uniform_int_distribution<int> distL(0, m_matrix.size() - 1);
//     std::uniform_int_distribution<int> distKW(1, m_unaryFuncMap.size() - 1);
//     std::uniform_int_distribution<int> distKV(1, m_binaryFuncMap.size() - 1);

//     if (w.size() < 4) w.resize(4);

//     int L = static_cast<int>(m_matrix.size());
//     if (m_unaryFuncMap.size() <= 1 || m_binaryFuncMap.size() <= 1) {
//         throw std::invalid_argument("m_unaryFuncMap and m_binaryFuncMap must have at least 2 elements");
//     }

//     w[0] = dist4(gen);

//     switch (w[0])
//     {
//     case 0:
//     case 2:
//     case 3:
//         w[1] = distL(gen);
//         w[2] = w[1] + 1 + (rand() % (L - w[1] - 1)); // Note: Consider replacing rand() here too
//         w[3] = distKW(gen); // Generates [1, kW-1]
//         break;

//     case 1:
//         w[1] = distL(gen);
//         while (w[1] < L && !TestSource(w[1]))
//             w[1]++;
//         w[2] = w[1];
//         w[3] = distKV(gen); // Generates [1, kV-1]
//         break;
//     }
// }

// приминение вариации
void NetOper::Variations(const std::vector<int>& w)
{
    if (w.size() < 4) return; // safety

    // std::cout<<"Apply Variations"<<std::endl;
    // for (const auto& i : w)
    // {
        // std::cout<<i<<" ";
    // }
    // std::cout<<std::endl;

    if (w[0] != 0 || w[1] != 0 || w[2] != 0)
    {
        switch (w[0])
        {
        case 0: // замена недиагонального элемента
            if (m_matrix[w[1]][w[2]] != 0)
                m_matrix[w[1]][w[2]] = w[3];
            break;

        case 1: // замена диагонального элемента
            if (m_matrix[w[1]][w[1]] != 0)
                m_matrix[w[1]][w[1]] = w[3];
            break;

        case 2: // добавление дуги
            if (m_matrix[w[1]][w[2]] == 0)
                if (m_matrix[w[2]][w[2]] != 0)
                    m_matrix[w[1]][w[2]] = w[3];
            break;

        case 3: // удаление дуги
        {
            int s1 = 0;
            for (int i = 0; i < w[2]; i++)
            {
                if (m_matrix[i][w[2]] != 0)
                    s1++;
            }

            int s2 = 0;
            for (int j = w[1] + 1; j < static_cast<int>(m_matrix.size()); j++)
            {
                if (m_matrix[w[1]][j] != 0)
                    s2++;
            }

            if (s1 > 1 && s2 > 1)
                m_matrix[w[1]][w[2]] = 0;
            break;
        }
        }
    }
    // std::cout<<"Variations"<<std::endl;
}

void NetOper::printMatrix() const
{
    for (const auto& row : m_matrix)
    {
        for (auto val : row)
            std::cout << val << " ";
        std::cout << "\n";
    }
    std::cout << "---------------------\n";
}

// only for local tests
void NetOper::setLocalTestsParameters()
{
    NOPMatrixReader& reader = this->getReader();
    reader.readMatrix("/home/user/catkin_ws/src/rosbot_nop_controller/data/24_NOP_461");
    reader.readParams("/home/user/catkin_ws/src/rosbot_nop_controller/data/q_461.txt");

    this->setNodesForVars({0, 1, 2});   // Pnum
    this->setNodesForParams({3, 4, 5}); // Rnum
    this->setNodesForOutput({22, 23});  // Dnum
    this->setCs(reader.getParams());
    this->setPsi(reader.getMatrix());
}

// NopPsiN - Naive NetOper trained without constraits
// OK super default
// from q_461.txt
// const std::vector<std::vector<int>> NopPsiN =
//   {{0,0,0,0,  0,0,1,10,  0,0,12,1,  0,0,0,0,  0,0,0,0,   0,0,0,10},
//    {0,0,0,0,  0,0,0, 1,  0,0,0,0,   0,0,0,0,  0,0,0,12,  0,0,0,0},
//    {0,0,0,0,  0,0,0, 0,  1,0,0,0,   0,0,2,9,  0,0,0,0,   10,0,0,0},
//    {0,0,0,0,  0,0,1, 0,  0,0,0,0,   0,0,0,13, 0,0,0,0,   0,0,0,0},

//    {0,0,0,0,  0,0,0, 1,  0,0,0,0,   0,0,0,0,   0,0,1,0,   0,0,0,0},
//    {0,0,0,0,  0,0,0, 0,  1,0,0,0,   0,0,0,0,   0,0,0,0,   0,0,0,19},
//    {0,0,0,0,  0,0,2, 0,  0,8,0,5,   0,4,13,10, 0,0,0,14,  15,0,0,0},
//    {0,0,0,0,  0,0,0, 2,  0,1,10,9,  0,0,0,0,  0,0,0,0,   0,0,0,0},

//    {0,0,0,0,  0,0,0,0,  2,1,0,0,   8,0,0,0,  12,0,0,0,  19,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,1,1,8,   0,0,0,1,  8,0,0,0,   14,12,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,1,1,   0,5,4,23, 1,0,0,0,   15,0,0,23},
//    {0,0,0,0,  0,0,0,0,  0,0,0,1,   17,10,10,0,  0,0,0,16,   0,16,0,16},

//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   1,0,15,0, 14,0,0,0,  0,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,1,9,0,  0,0,0,0,   0,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,1,1,  10,0,0,0,  0,12,0,13},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,1,  8,0,0,16,   0,0,0,0},

//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  1,1,0,0,   0,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,2,1,0,   15,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,2,0,   17,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,1,   5,0,0,17},

//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   1,1,0,13},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,1,1,17},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,0,1,4},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,0,0,1}};

// random
// const std::vector<std::vector<int>> NopPsiN =
//   {{0,0,0,0,  0,0,1,10,  0,0,12,1,  0,0,0,0,  0,0,0,0,   0,0,0,10},
//    {0,0,0,0,  0,0,0, 1,  0,0,0,0,   0,0,0,0,  0,0,0,12,  0,0,0,0},
//    {0,0,0,0,  0,0,0, 0,  1,0,0,0,   0,0,2,9,  0,0,0,0,   10,0,0,0},
//    {0,0,0,0,  0,0,1, 0,  0,0,6,0,   0,0,0,13, 0,0,0,0,   0,0,0,0},

//    {0,0,0,0,  0,0,0, 1,  0,0,0,0,   0,0,0,0,   0,0,1,0,   0,0,0,0},
//    {0,0,0,0,  0,0,0, 0,  1,0,0,0,   0,0,0,0,   0,0,0,0,   0,0,0,19},
//    {0,0,1,0,  0,0,2, 0,  0,8,0,5,   0,4,13,10, 0,0,0,14,  15,0,0,0},
//    {0,0,0,0,  0,0,0, 2,  0,1,10,9,  0,0,0,0,  0,0,0,0,   0,0,0,0},

//    {0,0,0,0,  0,0,0,0,  2,1,0,0,   8,0,0,0,  12,0,0,0,  19,0,0,0},
//    {0,1,0,0,  0,2,0,0,  0,1,1,8,   0,0,0,1,  8,0,0,0,   14,12,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,1,1,   0,5,4,23, 1,0,0,0,   15,0,0,23},
//    {0,0,0,0,  0,0,0,0,  0,0,0,1,   17,10,10,0,  0,0,0,16,   0,16,0,16},

//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   1,0,0,0, 14,0,0,0,  0,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,1,9,0,  0,0,0,0,   0,0,0,0},
//    {0,0,0,0,  0,1,0,0,  0,0,0,0,   0,0,1,1,  10,0,0,0,  0,12,0,13},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,1,  8,0,0,16,   0,0,0,0},

//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  1,1,0,0,   0,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,2,1,0,   15,0,0,0},
//    {0,7,0,0,  0,0,0,0,  0,0,17,0,   0,0,1,0,  0,0,2,0,   17,0,0,0},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,1,   5,0,0,17},

//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   1,1,0,13},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,9,0,0,  0,0,0,0,   0,1,1,17},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,0,1,4},
//    {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,0,0,1}};

// good for NN
// const std::vector<std::vector<int>> NopPsiN = {
//     {0, 0, 0, 0, 0, 0, 1, 10, 0, 0, 12, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
//     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2, 9, 0, 0, 0, 0, 10, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 6, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 22, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0, 19},
//     {0, 0, 1, 0, 0, 0, 6, 0, 0, 8, 0, 5, 0, 4, 13, 10, 0, 0, 0, 14, 15, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 10, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 8, 0, 0, 0, 12, 0, 0, 1, 19, 0, 0, 0},
//     {0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 1, 8, 0, 0, 0, 1, 8, 0, 0, 0, 14, 12, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 5, 4, 23, 0, 0, 0, 0, 15, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 17, 10, 10, 0, 0, 0, 0, 16, 0, 16, 0, 16},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 14, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 10, 0, 0, 0, 0, 12, 0, 13},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 8, 0, 0, 27, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 15, 1, 0, 0},
//     {0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 1, 0, 0, 0, 2, 0, 17, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 21, 4, 16, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 7, 16, 17},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4}
// };


// const std::vector<std::vector<int>> NopPsiN = 
//  {
//         {0, 0, 0, 0, 0, 0, 1, 10, 0, 0, 12, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
//         {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2, 9, 0, 0, 0, 0, 10, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 6, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 22, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0, 19},
//         {0, 0, 1, 0, 0, 0, 6, 0, 0, 8, 0, 5, 0, 4, 13, 10, 0, 0, 0, 14, 15, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 10, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 8, 0, 0, 0, 12, 0, 0, 1, 19, 0, 0, 0},
//         {0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 1, 8, 0, 0, 0, 1, 8, 0, 0, 0, 14, 12, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 5, 4, 23, 0, 0, 0, 0, 23, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 17, 10, 10, 0, 0, 0, 20, 16, 0, 16, 0, 16},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 14, 0, 25, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 10, 0, 0, 0, 0, 0, 0, 13},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 8, 0, 0, 27, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 15, 1, 0, 0},
//         {0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 1, 0, 0, 0, 2, 0, 23, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 21, 4, 16, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 7, 16, 13},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4},
//         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6}
//     };
    
// Goood
const std::vector<std::vector<int>> NopPsiN = 
{
    {0, 0, 0, 0, 0, 0, 1, 10, 0, 0, 12, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2, 9, 0, 0, 0, 0, 10, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 6, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 22, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0, 19},
    {0, 0, 1, 0, 0, 0, 6, 0, 0, 8, 0, 5, 0, 4, 13, 10, 0, 0, 0, 14, 15, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 10, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 8, 0, 0, 0, 12, 0, 0, 1, 19, 0, 0, 0},
    {0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 1, 8, 0, 0, 0, 1, 8, 0, 0, 0, 14, 12, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 5, 4, 23, 0, 0, 0, 0, 15, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 17, 10, 10, 0, 0, 0, 0, 16, 0, 16, 0, 16},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 14, 0, 25, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 10, 0, 0, 0, 0, 12, 0, 13},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 8, 0, 0, 27, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 15, 1, 0, 0},
    {0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 1, 0, 0, 0, 2, 0, 17, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 21, 4, 16, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 7, 16, 13},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6}
};

// from q_461.txt
// std::vector<float> qc = {12.86841, 3.82666, 6.94312};

// const std::vector<std::vector<int>> NopPsiN =
//   { {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 7, 10, 6},
//     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 12, 0, 0, 0, 11, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 19, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 2, 19, 0, 12, 0, 0, 23, 15, 10, 0, 23, 0, 0, 5},
//     {0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 1, 12, 19, 0, 0, 0, 0, 1, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 19, 0, 0, 0, 8, 0, 0, 0, 19},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 13, 4, 0, 1, 12, 1, 23, 19, 8},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 18, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 12, 0, 0, 0, 19, 14, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 23, 18, 0, 14, 6},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 13, 4, 19, 3, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 10, 10, 0, 12, 14},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4, 14, 16, 18},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 4},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 16, 1},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 7},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};

// random test
// std::vector<float> qc = {29104.4, 39545.1, 43488.3, 1};
std::vector<float> qc = {33879.9, 43369.9, 52114.8, 27901.6};


// Base NetOper matrix to represent simple desiredFucntion from nop_test and controller_test
// for simple sin/cos task
// std::vector<float> qc = {1.0, 1.0, 1.0};
// const std::vector<std::vector<int>> Psi =
//    {{0,0,0,0,  0,1,1,1,  0,2,0,0, 0,0},
//     {0,0,0,0,  0,0,1,0,  2,0,0,0, 0,0},
//     {0,0,0,0,  0,1,0,0,  0,0,0,0, 0,0},
//     {0,0,0,0,  0,0,0,0,  0,0,1,0, 0,0},

//     {0,0,0,0,  1,0,0,3,  0,0,0,0, 0,0},
//     {0,0,0,0,  0,1,0,0,  0,0,1,0, 0,0},
//     {0,0,0,0,  0,0,1,0,  0,0,0,1, 0,0},
//     {0,0,0,0,  0,0,0,1,  0,0,0,6, 0,0},

//     {0,0,0,0,  0,0,0,0,  1,3,0,0, 0,0},
//     {0,0,0,0,  0,0,0,0,  0,1,0,0, 1,0},
//     {0,0,0,0,  0,0,0,0,  0,0,1,0, 11,0},
//     {0,0,0,0,  0,0,0,0,  0,0,0,1, 0,1},

//     {0,0,0,0,  0,0,0,0,  0,0,0,0, 1,1},
//     {0,0,0,0,  0,0,0,0,  0,0,0,0, 0,1}};


// optimal for simple
// const std::vector<std::vector<int>> Psi = {
//     {0, 0, 0, 0, 0, 1, 1, 26, 0, 2, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
//     {0, 0, 0, 0, 4, 0, 0, 3, 0, 22, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 12},
//     {0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 16, 0},
//     {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 8, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 1, 14, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 1, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 11, 8},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 13},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 18},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
// };


// const std::vector<std::vector<int>> NopPsiN = Psi;