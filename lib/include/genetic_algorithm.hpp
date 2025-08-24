#pragma once

#include "nop.hpp"
#include "model.hpp"
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>
#include <chrono> // For timing


// Represents a chromosome (encoding of a NetOper configuration)
struct Chromosome {
    // std::vector<std::vector<int>> psi_matrix;
    std::vector<float> parameters = {12.86841, 3.82666, 6.94312}; // Equivalent to Cs
    // Add other parameters from NetOper if they are part of the chromosome
    std::vector<float> fitness_values; // To store functional values
    int pareto_rank; // Distance to Pareto set

    std::vector<std::vector<int>> psi_matrix =
    {{0,0,0,0,  0,0,1,10,  0,0,12,1,  0,0,0,0,  0,0,0,0,   0,0,0,10},
     {0,0,0,0,  0,0,0, 1,  0,0,0,0,   0,0,0,0,  0,0,0,12,  0,0,0,0},
     {0,0,0,0,  0,0,0, 0,  1,0,0,0,   0,0,2,9,  0,0,0,0,   10,0,0,0},
     {0,0,0,0,  0,0,1, 0,  0,0,0,0,   0,0,0,13, 0,0,0,0,   0,0,0,0},
  
     {0,0,0,0,  0,0,0, 1,  0,0,0,0,   0,0,0,0,   0,0,1,0,   0,0,0,0},
     {0,0,0,0,  0,0,0, 0,  1,0,0,0,   0,0,0,0,   0,0,0,0,   0,0,0,19},
     {0,0,0,0,  0,0,2, 0,  0,8,0,5,   0,4,13,10, 0,0,0,14,  15,0,0,0},
     {0,0,0,0,  0,0,0, 2,  0,1,10,9,  0,0,0,0,  0,0,0,0,   0,0,0,0},
  
     {0,0,0,0,  0,0,0,0,  2,1,0,0,   8,0,0,0,  12,0,0,0,  19,0,0,0},
     {0,0,0,0,  0,0,0,0,  0,1,1,8,   0,0,0,1,  8,0,0,0,   14,12,0,0},
     {0,0,0,0,  0,0,0,0,  0,0,1,1,   0,5,4,23, 1,0,0,0,   15,0,0,23},
     {0,0,0,0,  0,0,0,0,  0,0,0,1,   17,10,10,0,  0,0,0,16,   0,16,0,16},
  
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   1,0,15,0, 14,0,0,0,  0,0,0,0},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,1,9,0,  0,0,0,0,   0,0,0,0},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,1,1,  10,0,0,0,  0,12,0,13},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,1,  8,0,0,16,   0,0,0,0},
  
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  1,1,0,0,   0,0,0,0},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,2,1,0,   15,0,0,0},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,2,0,   17,0,0,0},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,1,   5,0,0,17},
  
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   1,1,0,13},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,1,1,17},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,0,1,4},
     {0,0,0,0,  0,0,0,0,  0,0,0,0,   0,0,0,0,  0,0,0,0,   0,0,0,1}};

    // Constructor (you might need to adjust parameters based on how chromosomes are created)
    Chromosome(size_t psi_dim, size_t num_params) :
        fitness_values(2), // Assuming 2 functionals as in TUser.Func
        pareto_rank(0)
    {}
};

class GeneticAlgorithm {
public:
    GeneticAlgorithm(
        int population_size,
        int num_generations,
        // Add other GA parameters as needed (crossover rate, mutation rate, etc.)
        double crossover_rate,
        double mutation_rate,
        // Add parameters for the system model and network operator
        const Model::State& initial_state,
        const Model::State& target_state,
        float time_step,
        float terminal_threshold,
        size_t psi_matrix_dimension,
        size_t num_network_operator_parameters,
        const std::vector<float>& qy_min,
        const std::vector<float>& qy_max,
        const std::vector<float>& qy_step,
        size_t num_undefined_params
    );

    void run(); // Main GA loop
    
    const std::vector<Chromosome>& getPopulation() const {
        return m_population;
    }

private:
    void initializePopulation();
    void evaluateFitness(Chromosome& chromosome, NetOper& network_operator, Model& system_model);
    void selectParents(std::vector<int>& parent_indices);
    Chromosome crossover(const Chromosome& parent1, const Chromosome& parent2);
    void mutate(Chromosome& chromosome);
    void calculateParetoRanks();
    void selectNextGeneration(const std::vector<Chromosome>& offspring_population); // Selection and replacement
    // Helper function to calculate distance to Pareto front (equivalent to Rast in Pascal)
    int calculateParetoRank(const std::vector<float>& fitness, const std::vector<Chromosome>& population);

private:
    int m_populationSize;
    int m_numGenerations;
    double m_crossoverRate;
    double m_mutationRate;

    Model::State m_initialState;
    Model::State m_targetState;
    float m_timeStep;
    float m_terminalThreshold;
    size_t m_psiMatrixDimension;
    size_t m_numNetworkOperatorParameters;


    std::vector<float> m_qyMin;
    std::vector<float> m_qyMax;
    std::vector<float> m_qyStep;
    size_t m_numUndefinedParams; // To store ny1

    std::vector<Chromosome> m_population;
    std::default_random_engine m_randomEngine;
    // Add other member variables for GA operations

    // Constants from Pascal code (adjust values based on your needs)
    const float Infinity = 1e8; // Based on baseFunctions.hpp
    const float EpsTerm = 0.1; // Based on unit1.pas
};
