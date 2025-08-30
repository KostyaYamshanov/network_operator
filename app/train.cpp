// #include "genetic_algorithm.hpp"
// #include "nop.hpp"
// #include "model.hpp"
// #include "baseFunctions.hpp" // To potentially use constants like Infinity

// #include <iostream>
// #include <vector>
// #include <cmath> // For M_PI (pi)
// #include <algorithm> // For sorting

// int main() {
//     // --- Parameters based on Pascal unit1.pas and unitadaptobject.pas ---

//     // Genetic Algorithm Parameters (from unit1.pas and TGANOP)
//     int population_size = 128;      // HH1
//     int num_generations = 128;       // PP1
//     double crossover_rate = 0.4;     // alfa1
//     double mutation_rate = 0.7;      // pmut1
//     int ksearch = 8;                 // ksearch1
//     int num_functionals = 2;         // nfu1
//     int num_parameters = 3;          // p1 (number of searching parameters in NOP)
//     int bits_integer_part = 4;       // c1
//     int bits_fractional_part = 12;   // d1
//     int generations_per_epoch = 10;  // Epo1
//     int num_elite_chromosomes = 10; // kel1

//     // System Model Parameters (from unit1.pas and TModel/TUser)
//     Model::State initial_state = {2.0, 2.0, 0.0}; // x0c
//     Model::State target_state = {0.0, 0.0, 1.0};  // xfc 
//     float time_step = 0.01;          // dt1
//     float terminal_threshold = 0.1;  // epsterm
//     float terminal_time = 5.0;       // tf1
//     float penalty_coefficient = 2.0; // Shtraf1

//     // Network Operator Parameters (from unit1.pas and TNetOper)
//     size_t psi_matrix_dimension = 24; // L1
//     size_t num_vars = 3;              // kp1 (cardinal of set of variables)
//     size_t num_params_nop = 3;        // kr1 (cardinal of set of parameters in NOP)
//     size_t num_unary_ops = 20;        // kw1 (cardinal of set of unary operations)
//     size_t num_binary_ops = 2;       // kv1 (cardinal of set of binary operations)
//     size_t num_outputs_nop = 2;       // Mout1 (number of outputs from NOP)
//     size_t num_undefined_params = 3;  // ny1 (dimension of undefined parameters, relevant for TModel's Integr)

//     // Node mappings (Pnumc, Rnumc, Dnumc from unit1.pas)
//     std::vector<int> nodes_for_vars = {0, 1, 2};     // Pnumc
//     std::vector<int> nodes_for_params = {3, 4, 5};   // Rnumc
//     std::vector<int> nodes_for_output = {22, 23};  // Dnumc

//     // Initial parameters for the Network Operator (qc from unit1.pas)
//     std::vector<float> initial_nop_params = {1.0, 1.0, 1.0}; // qc (Note: qc is also used for q in TGANOP)

//     // Range for undefined parameters (qyminc, qymaxc, stepsqyc from unit1.pas)
//     std::vector<float> qy_min = {-3, -3, -50 * M_PI / 12.0};
//     std::vector<float> qy_max = {3, 3, 50 * M_PI / 12.0};
//     std::vector<float> qy_step = {0.25, 0.25, 5.0 * M_PI / 12.0};


//     // --- Create and Configure Genetic Algorithm ---

//     GeneticAlgorithm ga(
//         population_size,
//         num_generations,
//         crossover_rate,
//         mutation_rate,
//         initial_state,
//         target_state,
//         time_step,
//         terminal_threshold,
//         psi_matrix_dimension,
//         num_parameters,
//         qy_min,
//         qy_max,
//         qy_step,
//         num_undefined_params // Using num_parameters (p1) for the chromosome's parameter size
//     );

//     // Set additional GA parameters (if you added them as members)
//     // ga.setKsearch(ksearch);
//     // ga.setGenerationsPerEpoch(generations_per_epoch);
//     // ga.setNumEliteChromosomes(num_elite_chromosomes);
//     // ga.setPenaltyCoefficient(penalty_coefficient);


//     // --- Set fixed parameters for the Network Operator within the GA ---
//     // This assumes your GeneticAlgorithm class has methods to access/set
//     // the parameters of its internal NetOper object. If not, you might need
//     // to pass these to the GeneticAlgorithm constructor and store them.

//     // Example (assuming you add these setters to GeneticAlgorithm):
//     // ga.setNetworkOperatorNodesForVars(nodes_for_vars);
//     // ga.setNetworkOperatorNodesForParams(nodes_for_params);
//     // ga.setNetworkOperatorNodesForOutput(nodes_for_output);

//     // The initial_nop_params (qc) are part of the chromosome, so they are
//     // initialized randomly in initializePopulation. If you want to start
//     // with the specific qc from Pascal, you would need to modify
//     // initializePopulation to include this initial chromosome.

//     // --- Run the Genetic Algorithm ---
//     std::cout << "Starting Genetic Algorithm training..." << std::endl;
//     ga.run();
//     std::cout << "Genetic Algorithm training finished." << std::endl;

//     // --- Access and Output Results ---
//     // After run() completes, the trained population is in ga.m_population.
//     // You can access the best chromosomes (e.g., those with Pareto rank 0).

//     std::cout << "\n--- Best Chromosomes (Pareto Front) ---" << std::endl;
//     // Change the vector type to store const Chromosome*
//     std::vector<const Chromosome*> pareto_front;
//     // Access the population using the public getter method
//     for (const auto& chrom : ga.getPopulation()) {
//         if (chrom.pareto_rank == 0) {
//             // Now you can push back a const Chromosome*
//             pareto_front.push_back(&chrom);
//         }
//     }

//     // You might want to sort the Pareto front based on one of the objectives
//     // For example, sorting by the first functional (time):
//     std::sort(pareto_front.begin(), pareto_front.end(), [](const Chromosome* a, const Chromosome* b) {
//         return a->fitness_values[0] < b->fitness_values[0];
//     });

//     if (pareto_front.empty()) {
//         std::cout << "No chromosomes on the Pareto front found." << std::endl;
//     } else {
//         std::cout << "Number of chromosomes on Pareto front: " << pareto_front.size() << std::endl;
//         // Print details of the chromosomes on the Pareto front
//         for (const auto& chrom : pareto_front) {
//             std::cout << "  Fitness (Time, Distance): ("
//                       << chrom->fitness_values[0] << ", "
//                       << chrom->fitness_values[1] << ")" << std::endl;
//             // You can also print the psi_matrix and parameters of the best chromosomes
//             // std::cout << "  Psi Matrix: ..." << std::endl;
//             // std::cout << "  Parameters: ..." << std::endl;
//         }

//         // You can save the best chromosome's matrix and parameters to a file
//         // for later use.
//     }


//     return 0;
// }


#include "ganop.hpp"

int main() {
    GANOP ga(3, 4, 8, 7, 3, 2); // p=3, c=4, d=8, lchr=5, HH=3, nfu=1
    ga.GenAlgorithm();
    return 0;
}