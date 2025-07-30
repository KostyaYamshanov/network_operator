#include "genetic_algorithm.hpp"


// GeneticAlgorithm class method implementations

GeneticAlgorithm::GeneticAlgorithm(
    int population_size,
    int num_generations,
    double crossover_rate,
    double mutation_rate,
    const Model::State& initial_state,
    const Model::State& target_state,
    float time_step,
    float terminal_threshold,
    size_t psi_matrix_dimension,
    size_t num_network_operator_parameters
) :
    m_populationSize(population_size),
    m_numGenerations(num_generations),
    m_crossoverRate(crossover_rate),
    m_mutationRate(mutation_rate),
    m_initialState(initial_state),
    m_targetState(target_state),
    m_timeStep(time_step),
    m_terminalThreshold(terminal_threshold),
    m_psiMatrixDimension(psi_matrix_dimension),
    m_numNetworkOperatorParameters(num_network_operator_parameters),
    m_population(population_size, Chromosome(psi_matrix_dimension, num_network_operator_parameters))
{
    std::random_device rd;
    m_randomEngine.seed(rd());
}

void GeneticAlgorithm::run() {
    // Create NetOper and Model objects for fitness evaluation
    NetOper network_operator;
    // You might need to set initial parameters for the network_operator here
    // based on your application's requirements and the Pascal code defaults.
    // For example:
    // network_operator.setNodesForVars({...});
    // network_operator.setNodesForParams({...});
    // network_operator.setNodesForOutput({...});

    network_operator.setNodesForVars({0, 1, 2});   // Pnum
    network_operator.setNodesForParams({3, 4, 5}); // Rnum
    network_operator.setNodesForOutput({22, 23});  // Dnum

    Model system_model(m_initialState, m_timeStep);

    // Initialize the population
    initializePopulation();

    // Genetic algorithm main loop
    for (int generation = 0; generation < m_numGenerations; ++generation) {
        std::cout << "Generation: " << generation + 1 << "/" << m_numGenerations << std::endl;

        // Evaluate fitness of the current population
        for (int i = 0; i < m_populationSize; ++i) {
            evaluateFitness(m_population[i], network_operator, system_model);
        }

        // Calculate Pareto ranks
        calculateParetoRanks();

        // Select parents and generate offspring
        std::vector<Chromosome> offspring_population;
        offspring_population.reserve(m_populationSize); // Assuming same number of offspring as population size

        for (int i = 0; i < m_populationSize / 2; ++i) // Generate pairs of offspring
        {
            std::vector<int> parent_indices;
            selectParents(parent_indices);
            const Chromosome& parent1 = m_population[parent_indices[0]];
            const Chromosome& parent2 = m_population[parent_indices[1]];

            // Perform crossover with crossover rate
            if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_crossoverRate) {
                Chromosome offspring1 = crossover(parent1, parent2);
                Chromosome offspring2 = crossover(parent2, parent1); // Generate two offspring

                // Mutate offspring with mutation rate
                if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_mutationRate) {
                    mutate(offspring1);
                }
                 if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_mutationRate) {
                    mutate(offspring2);
                }

                offspring_population.push_back(offspring1);
                offspring_population.push_back(offspring2);

            } else {
                 // If no crossover, offspring are copies of parents (optional, depends on GA variant)
                 // For simplicity, let's just add mutated copies of parents as offspring if no crossover
                 Chromosome offspring1 = parent1;
                 Chromosome offspring2 = parent2;

                 if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_mutationRate) {
                    mutate(offspring1);
                 }
                 if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_mutationRate) {
                    mutate(offspring2);
                 }

                 offspring_population.push_back(offspring1);
                 offspring_population.push_back(offspring2);
            }
        }

        // Select the next generation from the combined population of parents and offspring
        selectNextGeneration(offspring_population);

        // Print or log the best fitness values in each generation
        auto best_chromosome_it = std::min_element(m_population.begin(), m_population.end(), [](const Chromosome& a, const Chromosome& b) {
            return a.pareto_rank < b.pareto_rank;
        });

        if (best_chromosome_it != m_population.end()) {
             std::cout << "Generation " << generation + 1 << " - Best fitness (Time, Distance): ("
                       << best_chromosome_it->fitness_values[0] << ", "
                       << best_chromosome_it->fitness_values[1] << ")" << std::endl;
        }
    }

    // Training is complete. The m_population now contains the final generation
    // You can access the best chromosomes from here (e.g., those with Pareto rank 0)
}

void GeneticAlgorithm::initializePopulation() {
    std::random_device rd;
    m_randomEngine.seed(rd()); // Seed the random number generator

    // Fill chromosomes with random data (similar to Pascal's GenAlgorithm)
    for (int i = 0; i < m_populationSize; ++i) {
        // Initialize psi_matrix randomly
        for (size_t row = 0; row < m_psiMatrixDimension; ++row) {
            for (size_t col = 0; col < m_psiMatrixDimension; ++col) {
                // Generate random integer for matrix element
                // **Adjust the range based on the Pascal code and baseFunctions.hpp**
                m_population[i].psi_matrix[row][col] = std::uniform_int_distribution<int>(0, 28)(m_randomEngine); // Example range
            }
        }

        // Initialize parameters (Cs) randomly
        for (size_t j = 0; j < m_numNetworkOperatorParameters; ++j) {
            // Generate random float for parameters
            // **Adjust the range based on the Pascal code (e.g., qc range)**
            m_population[i].parameters[j] = std::uniform_real_distribution<float>(-10.0, 10.0)(m_randomEngine); // Example range
        }

        // Fitness values will be calculated in evaluateFitness
        m_population[i].fitness_values.assign(2, 0.0); // Assuming 2 functionals
        m_population[i].pareto_rank = 0;
    }
}

void GeneticAlgorithm::evaluateFitness(Chromosome& chromosome, NetOper& network_operator, Model& system_model) {
    // Configure the network operator with the chromosome's data
    network_operator.setPsi(chromosome.psi_matrix);
    network_operator.setCs(chromosome.parameters);

    // Reset the system model to the initial state for simulation
    system_model.setState(m_initialState);

    // Simulate the system over time
    auto start_time = std::chrono::high_resolution_clock::now();
    float current_time = 0.0;

    // Simulation loop (similar to the loop in TUser.Func)
    // You might need to adjust the simulation duration or termination condition
    // based on the Pascal code's tf1 and epsterm.
    while (current_time <= m_numGenerations * m_timeStep) // Simulate for a fixed duration or until goal
    {
        // Get current state
        Model::State current_state = system_model.getState();

        // Check if the goal is reached
        if (current_state.dist(m_targetState) < m_terminalThreshold) {
            break; // Goal reached
        }

        // Calculate inputs for the network operator (difference from target state)
        std::vector<float> nop_inputs = {
            m_targetState.x - current_state.x,
            m_targetState.y - current_state.y,
            m_targetState.yaw - current_state.yaw
        };

        // Get control outputs from the network operator
        std::vector<float> control_outputs(network_operator.getNodesForOutput().size()); // Assuming output size matches Dnum
        network_operator.calcResult(nop_inputs, control_outputs);

        // Create Control struct from network operator outputs
        Model::Control control_input = {control_outputs[0], control_outputs[1]}; // Assuming Dnum maps to left/right control

        // Apply control constraints (if any, based on Pascal OgrUpr)
        // You might need to add this logic here.
        // For simplicity, we'll omit it for now.

        // Update system state using the model
        system_model.setState(system_model.nextStateFromControl(control_input));

        // Increment time
        current_time += m_timeStep;

        // Prevent infinite loops in case of unstable systems
        if (current_time > m_numGenerations * m_timeStep * 2) { // Example: terminate if simulation runs too long
             current_time = m_numGenerations * m_timeStep + 1; // Mark as exceeding time limit
             break;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    // Calculate fitness values (functionals)
    // Based on TUser.Func: time to reach goal and distance to goal
    chromosome.fitness_values[0] = elapsed.count(); // Time elapsed
    chromosome.fitness_values[1] = system_model.getState().dist(m_targetState); // Final distance to target

    // If goal was not reached within the simulation time, penalize fitness
    if (current_time > m_numGenerations * m_timeStep)
    {
         // Penalize based on the final distance or other criteria
         // **Adjust the penalty based on the Pascal Shtraf1**
        chromosome.fitness_values[0] += chromosome.fitness_values[1] * 100; // Example penalty
    }
}

void GeneticAlgorithm::selectParents(std::vector<int>& parent_indices) {
    parent_indices.resize(2); // Select two parents

    // Implement a selection mechanism similar to the Pascal code (tournament selection)
    int ksearch = 8; // **Replace with a member variable initialized from Pascal code**

    // Select first parent
    int k1 = std::uniform_int_distribution<int>(0, m_populationSize - 1)(m_randomEngine);
    int best_k1 = k1;
    int lh_min1 = m_population[k1].pareto_rank;

    for (int i = 0; i < ksearch; ++i) {
        int ks1 = std::uniform_int_distribution<int>(0, m_populationSize - 1)(m_randomEngine);
        if (m_population[ks1].pareto_rank < lh_min1) { // Assuming lower rank is better
            best_k1 = ks1;
            lh_min1 = m_population[ks1].pareto_rank;
        }
    }
    parent_indices[0] = best_k1;

    // Select second parent (similar to the first)
    int k2 = std::uniform_int_distribution<int>(0, m_populationSize - 1)(m_randomEngine);
    int best_k2 = k2;
    int lh_min2 = m_population[k2].pareto_rank;

    for (int i = 0; i < ksearch; ++i) {
        int ks2 = std::uniform_int_distribution<int>(0, m_populationSize - 1)(m_randomEngine);
        if (m_population[ks2].pareto_rank < lh_min2) { // Assuming lower rank is better
            best_k2 = ks2;
            lh_min2 = m_population[ks2].pareto_rank;
        }
    }
    parent_indices[1] = best_k2;
}

Chromosome GeneticAlgorithm::crossover(const Chromosome& parent1, const Chromosome& parent2) {
    // Create a new offspring chromosome
    Chromosome offspring(m_psiMatrixDimension, m_numNetworkOperatorParameters);

    // Perform crossover on the psi_matrix (structural part)
    // Assuming a single-point crossover for simplicity
    size_t crossover_point_matrix = std::uniform_int_distribution<size_t>(0, m_psiMatrixDimension * m_psiMatrixDimension - 1)(m_randomEngine);

    for (size_t i = 0; i < m_psiMatrixDimension; ++i) {
        for (size_t j = 0; j < m_psiMatrixDimension; ++j) {
            size_t linear_index = i * m_psiMatrixDimension + j;
            if (linear_index < crossover_point_matrix) {
                offspring.psi_matrix[i][j] = parent1.psi_matrix[i][j];
            } else {
                offspring.psi_matrix[i][j] = parent2.psi_matrix[i][j];
            }
        }
    }

    // Perform crossover on the parameters (parametric part)
    size_t crossover_point_params = std::uniform_int_distribution<size_t>(0, m_numNetworkOperatorParameters - 1)(m_randomEngine);

    for (size_t i = 0; i < m_numNetworkOperatorParameters; ++i) {
        if (i < crossover_point_params) {
            offspring.parameters[i] = parent1.parameters[i];
        } else {
            offspring.parameters[i] = parent2.parameters[i];
        }
    }

    return offspring;
}

void GeneticAlgorithm::mutate(Chromosome& chromosome) {
    // Mutate psi_matrix
    for (size_t i = 0; i < m_psiMatrixDimension; ++i) {
        for (size_t j = 0; j < m_psiMatrixDimension; ++j) {
            if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_mutationRate) {
                // Mutate the matrix element
                // Generate a random integer for the new value
                // **Adjust the range based on the possible values in Psi matrix**
                chromosome.psi_matrix[i][j] = std::uniform_int_distribution<int>(0, 28)(m_randomEngine); // Example range
            }
        }
    }

    // Mutate parameters
    for (size_t i = 0; i < m_numNetworkOperatorParameters; ++i) {
        if (std::uniform_real_distribution<double>(0.0, 1.0)(m_randomEngine) < m_mutationRate) {
            // Mutate the parameter value
            // Generate a random float for the new value
            // **Adjust the range based on the expected range of parameters**
            chromosome.parameters[i] = std::uniform_real_distribution<float>(-10.0, 10.0)(m_randomEngine); // Example range
        }
    }
}

void GeneticAlgorithm::calculateParetoRanks() {
    for (int i = 0; i < m_populationSize; ++i) {
        m_population[i].pareto_rank = calculateParetoRank(m_population[i].fitness_values, m_population);
    }
}

int GeneticAlgorithm::calculateParetoRank(const std::vector<float>& fitness, const std::vector<Chromosome>& population) {
    int count = 0;
    size_t nfu = fitness.size(); // Number of functionals

    for (const auto& other_chromosome : population) {
        const std::vector<float>& other_fitness = other_chromosome.fitness_values;

        // Check for dominance: is 'fitness' >= 'other_fitness' for all functionals?
        bool not_dominated = true;
        for (size_t j = 0; j < nfu; ++j) {
            if (fitness[j] < other_fitness[j]) {
                not_dominated = false;
                break;
            }
        }

        if (not_dominated) {
            // Check if 'fitness' is strictly better than 'other_fitness' in at least one functional
            bool are_equal = true;
            for(size_t k = 0; k < nfu; ++k) {
                if (fitness[k] != other_fitness[k]) {
                    are_equal = false;
                    break;
                }
            }

            if (!are_equal) {
                count++;
            }
        }
    }

    return count;
}

void GeneticAlgorithm::selectNextGeneration(const std::vector<Chromosome>& offspring_population) {
    // Combine parent and offspring populations
    std::vector<Chromosome> combined_population;
    combined_population.reserve(m_populationSize + offspring_population.size());

    // Add parent population
    combined_population.insert(combined_population.end(), m_population.begin(), m_population.end());

    // Add offspring population
    combined_population.insert(combined_population.end(), offspring_population.begin(), offspring_population.end());

    // Calculate Pareto ranks for the combined population
    for (auto& chrom : combined_population) {
        chrom.pareto_rank = calculateParetoRank(chrom.fitness_values, combined_population);
    }

    // Sort the combined population based on Pareto rank
    std::sort(combined_population.begin(), combined_population.end(), [](const Chromosome& a, const Chromosome& b) {
        return a.pareto_rank < b.pareto_rank;
    });

    // Select the best chromosomes to form the next generation
    m_population.clear();
    m_population.reserve(m_populationSize);
    for (int i = 0; i < m_populationSize; ++i) {
        m_population.push_back(combined_population[i]);
    }
}
