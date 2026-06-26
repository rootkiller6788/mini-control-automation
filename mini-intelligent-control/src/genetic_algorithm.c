#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* genetic_algorithm.c - GA for Controller Parameter Optimization */

int ic_ga_init(ic_genetic_alg_t *ga, size_t pop_size, size_t chromo_length,
                size_t max_gen, double cross_rate, double mut_rate) {
    if (!ga || pop_size == 0 || chromo_length == 0) return -1;
    memset(ga, 0, sizeof(ic_genetic_alg_t));
    ga->pop_size = pop_size; ga->chromo_length = chromo_length;
    ga->max_generations = max_gen;
    ga->crossover_rate = cross_rate; ga->mutation_rate = mut_rate;
    ga->elite_fraction = 0.1;
    ga->selection_method = 1;
    ga->tournament_size = 3;
    ga->population = (double*)calloc(pop_size * chromo_length, sizeof(double));
    ga->fitness = (double*)calloc(pop_size, sizeof(double));
    ga->lower_bound = (double*)calloc(chromo_length, sizeof(double));
    ga->upper_bound = (double*)calloc(chromo_length, sizeof(double));
    ga->best_chromosome = (double*)calloc(chromo_length, sizeof(double));
    if (!ga->population || !ga->fitness || !ga->lower_bound ||
        !ga->upper_bound || !ga->best_chromosome) {
        ic_ga_free(ga); return -1;
    }
    for (size_t i = 0; i < chromo_length; i++) {
        ga->lower_bound[i] = -10.0; ga->upper_bound[i] = 10.0;
    }
    ga->best_fitness = -DBL_MAX;
    /* Random initialization */
    for (size_t i = 0; i < pop_size; i++) {
        for (size_t j = 0; j < chromo_length; j++) {
            double range = ga->upper_bound[j] - ga->lower_bound[j];
            ga->population[i * chromo_length + j] =
                ga->lower_bound[j] + ic_random_uniform() * range;
        }
    }
    return 0;
}

void ic_ga_free(ic_genetic_alg_t *ga) {
    if (!ga) return;
    free(ga->population); free(ga->fitness);
    free(ga->lower_bound); free(ga->upper_bound); free(ga->best_chromosome);
    memset(ga, 0, sizeof(ic_genetic_alg_t));
}

void ic_ga_evaluate_population(ic_genetic_alg_t *ga,
                                double (*fitness_func)(const double*, void*),
                                void *user_data) {
    if (!ga || !fitness_func) return;
    for (size_t i = 0; i < ga->pop_size; i++) {
        ga->fitness[i] = fitness_func(&ga->population[i * ga->chromo_length], user_data);
    }
    /* Track best */
    for (size_t i = 0; i < ga->pop_size; i++) {
        if (ga->fitness[i] > ga->best_fitness) {
            ga->best_fitness = ga->fitness[i];
            memcpy(ga->best_chromosome, &ga->population[i * ga->chromo_length],
                   ga->chromo_length * sizeof(double));
        }
    }
}

/* Tournament selection */
static size_t ga_tournament_select(const ic_genetic_alg_t *ga) {
    size_t best = (size_t)(ic_random_uniform() * (double)ga->pop_size) % ga->pop_size;
    for (size_t t = 1; t < ga->tournament_size; t++) {
        size_t candidate = (size_t)(ic_random_uniform() * (double)ga->pop_size) % ga->pop_size;
        if (ga->fitness[candidate] > ga->fitness[best]) best = candidate;
    }
    return best;
}

/* Single-point crossover */
static void ga_crossover(const ic_genetic_alg_t *ga,
                          const double *parent1, const double *parent2,
                          double *child1, double *child2) {
    if (ic_random_uniform() < ga->crossover_rate) {
        size_t point = (size_t)(ic_random_uniform() * (double)(ga->chromo_length - 1)) + 1;
        for (size_t j = 0; j < point; j++) {
            child1[j] = parent1[j]; child2[j] = parent2[j];
        }
        for (size_t j = point; j < ga->chromo_length; j++) {
            child1[j] = parent2[j]; child2[j] = parent1[j];
        }
    } else {
        memcpy(child1, parent1, ga->chromo_length * sizeof(double));
        memcpy(child2, parent2, ga->chromo_length * sizeof(double));
    }
}

/* Gaussian mutation */
static void ga_mutate(const ic_genetic_alg_t *ga, double *chromosome) {
    for (size_t j = 0; j < ga->chromo_length; j++) {
        if (ic_random_uniform() < ga->mutation_rate) {
            double delta = ic_random_gaussian() * 0.1 *
                           (ga->upper_bound[j] - ga->lower_bound[j]);
            chromosome[j] += delta;
            if (chromosome[j] < ga->lower_bound[j]) chromosome[j] = ga->lower_bound[j];
            if (chromosome[j] > ga->upper_bound[j]) chromosome[j] = ga->upper_bound[j];
        }
    }
}

void ic_ga_evolve(ic_genetic_alg_t *ga) {
    if (!ga || !ga->fitness) return;
    size_t pop = ga->pop_size, cl = ga->chromo_length;
    double *new_pop = (double*)calloc(pop * cl, sizeof(double));
    if (!new_pop) return;
    /* Elitism: keep best individuals */
    size_t n_elite = (size_t)(ga->elite_fraction * (double)pop);
    if (n_elite < 1) n_elite = 1;
    /* Sort by fitness (simple bubble for small pop) */
    size_t *indices = (size_t*)malloc(pop * sizeof(size_t));
    if (!indices) { free(new_pop); return; }
    for (size_t i = 0; i < pop; i++) indices[i] = i;
    for (size_t i = 0; i < pop - 1; i++) {
        for (size_t j = 0; j < pop - 1 - i; j++) {
            if (ga->fitness[indices[j]] < ga->fitness[indices[j + 1]]) {
                size_t tmp = indices[j]; indices[j] = indices[j + 1]; indices[j + 1] = tmp;
            }
        }
    }
    /* Copy elites */
    for (size_t i = 0; i < n_elite; i++) {
        memcpy(&new_pop[i * cl], &ga->population[indices[i] * cl], cl * sizeof(double));
    }
    /* Generate offspring */
    for (size_t i = n_elite; i < pop; i += 2) {
        size_t p1 = ga_tournament_select(ga);
        size_t p2 = ga_tournament_select(ga);
        double *parent1 = &ga->population[p1 * cl];
        double *parent2 = &ga->population[p2 * cl];
        double *child1 = &new_pop[i * cl];
        double *child2 = (i + 1 < pop) ? &new_pop[(i + 1) * cl] : NULL;
        ga_crossover(ga, parent1, parent2, child1,
                      child2 ? child2 : child1);
        ga_mutate(ga, child1);
        if (child2) ga_mutate(ga, child2);
    }
    memcpy(ga->population, new_pop, pop * cl * sizeof(double));
    free(new_pop); free(indices);
}

void ic_ga_get_best(const ic_genetic_alg_t *ga, double *best_chromosome,
                     double *best_fitness) {
    if (!ga) return;
    if (best_chromosome)
        memcpy(best_chromosome, ga->best_chromosome,
               ga->chromo_length * sizeof(double));
    if (best_fitness) *best_fitness = ga->best_fitness;
}

/* ---- L7: GA-based PID Tuning ---- */

typedef struct {
    double Kp, Ki, Kd;
    double setpoint;
    double e_prev, integral;
    double u_min, u_max;
} ic_pid_params_t;

/* Fitness function for PID tuning: minimize ITAE */
double ic_ga_pid_fitness(const double *chromosome, void *user_data) {
    ic_pid_params_t *pid = (ic_pid_params_t*)user_data;
    if (!pid || !chromosome) return -1e100;
    double Kp = chromosome[0], Ki = chromosome[1], Kd = chromosome[2];
    double plant_state = 0.0, plant_vel = 0.0;
    double dt = 0.01;
    double total_error = 0.0;
    double e_prev = 0.0, integral = 0.0;
    for (size_t step = 0; step < 200; step++) {
        double t = (double)step * dt;
        double sp = pid->setpoint;
        double y = plant_state;
        double e = sp - y;
        integral += e * dt;
        double de = (e - e_prev) / dt;
        double u = Kp * e + Ki * integral + Kd * de;
        if (u > pid->u_max) u = pid->u_max;
        if (u < pid->u_min) u = pid->u_min;
        /* Second-order plant: y'' + 2*zeta*wn*y' + wn^2*y = wn^2*u */
        double wn = 10.0, zeta = 0.5;
        double y_ddot = wn * wn * (u - plant_state) - 2.0 * zeta * wn * plant_vel;
        plant_vel += y_ddot * dt;
        plant_state += plant_vel * dt;
        total_error += t * fabs(e) * dt;
        e_prev = e;
    }
    return -total_error;
}

/* Optimize PID gains using GA */
int ic_ga_tune_pid(ic_genetic_alg_t *ga, ic_pid_params_t *pid) {
    if (!ga || !pid) return -1;
    ga->lower_bound[0] = 0.0; ga->upper_bound[0] = 100.0;
    ga->lower_bound[1] = 0.0; ga->upper_bound[1] = 50.0;
    ga->lower_bound[2] = 0.0; ga->upper_bound[2] = 20.0;
    for (size_t gen = 0; gen < ga->max_generations; gen++) {
        ic_ga_evaluate_population(ga, ic_ga_pid_fitness, pid);
        ic_ga_evolve(ga);
    }
    ic_ga_get_best(ga, NULL, NULL);
    pid->Kp = ga->best_chromosome[0];
    pid->Ki = ga->best_chromosome[1];
    pid->Kd = ga->best_chromosome[2];
    return 0;
}

/* ---- L8: Multi-Objective GA (Pareto Optimization) ---- */

typedef struct {
    double *objectives;
    size_t num_objectives;
    int dominated;
    size_t domination_count;
    size_t *dominates;
    size_t pareto_rank;
    double crowding_distance;
} ic_pareto_individual_t;

/* Pareto dominance check: A dominates B if A is better in at least
   one objective and no worse in all others */
int ic_pareto_dominates(const double *obj_a, const double *obj_b,
                         size_t num_obj) {
    int better_in_one = 0;
    for (size_t i = 0; i < num_obj; i++) {
        if (obj_a[i] < obj_b[i]) return 0;
        if (obj_a[i] > obj_b[i]) better_in_one = 1;
    }
    return better_in_one;
}

/* Compute crowding distance for NSGA-II selection */
void ic_crowding_distance(ic_pareto_individual_t *individuals, size_t n,
                           size_t num_obj) {
    for (size_t i = 0; i < n; i++) individuals[i].crowding_distance = 0.0;
    for (size_t m = 0; m < num_obj; m++) {
        /* Sort by objective m (bubble for simplicity) */
        for (size_t i = 0; i < n - 1; i++) {
            for (size_t j = 0; j < n - 1 - i; j++) {
                if (individuals[j].objectives[m] < individuals[j+1].objectives[m]) {
                    ic_pareto_individual_t tmp = individuals[j];
                    individuals[j] = individuals[j+1];
                    individuals[j+1] = tmp;
                }
            }
        }
        individuals[0].crowding_distance = INFINITY;
        individuals[n-1].crowding_distance = INFINITY;
        double obj_range = individuals[0].objectives[m] - individuals[n-1].objectives[m];
        if (fabs(obj_range) < 1e-12) continue;
        for (size_t i = 1; i < n - 1; i++) {
            individuals[i].crowding_distance +=
                (individuals[i-1].objectives[m] - individuals[i+1].objectives[m]) / obj_range;
        }
    }
}
