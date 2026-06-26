/**
 * @file intelligent_control_advanced.c
 * @brief PSO tuning, MRAC Lyapunov, ILC, Self-Tuning Regulator, ANFIS, ACO control.
 * Intensive L5-L8 intelligent control algorithms. Ref: Astrom & Wittenmark, Slotine & Li, Passino.
 */
#include "intelligent_control.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =================================================================
 * L5: Particle Swarm Optimization for PID Tuning
 * Minimizes ITAE = integral(t * |e(t)|) dt
 * ================================================================= */
typedef struct { double Kp, Ki, Kd; } PIDGains;
typedef struct { double position[3]; double velocity[3]; double best_pos[3]; double best_cost; } Particle;

static double pid_itae_cost(PIDGains g, double *setpoint, double *output, int n, double dt) {
    double itae = 0.0, integral = 0.0, prev_error = 0.0;
    for (int i = 0; i < n; i++) {
        double error = setpoint[i] - output[i];
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double u = g.Kp * error + g.Ki * integral + g.Kd * derivative;
        output[i] += u * dt;
        itae += (i * dt) * fabs(error) * dt;
        prev_error = error;
    }
    return itae;
}

int pso_pid_tune(double *setpoint, double *system_output, int n_samples, double dt,
                  int n_particles, int max_iter, PIDGains *best_gains, double *best_itae) {
    if (!setpoint || !system_output || n_samples <= 0 || dt <= 0 || n_particles < 5) return -1;
    Particle *swarm = calloc(n_particles, sizeof(Particle));
    if (!swarm) return -1;
    double w = 0.7, c1 = 1.5, c2 = 1.5, global_best[3], global_cost = 1e12;
    for (int p = 0; p < n_particles; p++) {
        for (int d = 0; d < 3; d++) {
            swarm[p].position[d] = ((double)rand() / RAND_MAX) * 10.0;
            swarm[p].velocity[d] = ((double)rand() / RAND_MAX - 0.5) * 2.0;
            swarm[p].best_pos[d] = swarm[p].position[d];
        }
        swarm[p].best_cost = 1e12;
    }
    double *temp_out = malloc(n_samples * sizeof(double));
    for (int iter = 0; iter < max_iter; iter++) {
        for (int p = 0; p < n_particles; p++) {
            memcpy(temp_out, system_output, n_samples * sizeof(double));
            PIDGains g = {swarm[p].position[0], swarm[p].position[1], swarm[p].position[2]};
            double cost = pid_itae_cost(g, setpoint, temp_out, n_samples, dt);
            if (cost < swarm[p].best_cost) {
                swarm[p].best_cost = cost;
                memcpy(swarm[p].best_pos, swarm[p].position, 3 * sizeof(double));
            }
            if (cost < global_cost) {
                global_cost = cost;
                memcpy(global_best, swarm[p].position, 3 * sizeof(double));
            }
        }
        for (int p = 0; p < n_particles; p++)
            for (int d = 0; d < 3; d++) {
                swarm[p].velocity[d] = w * swarm[p].velocity[d]
                    + c1 * ((double)rand()/RAND_MAX) * (swarm[p].best_pos[d] - swarm[p].position[d])
                    + c2 * ((double)rand()/RAND_MAX) * (global_best[d] - swarm[p].position[d]);
                swarm[p].position[d] += swarm[p].velocity[d];
                if (swarm[p].position[d] < 0) swarm[p].position[d] = 0;
            }
        w *= 0.99;
    }
    best_gains->Kp = global_best[0]; best_gains->Ki = global_best[1]; best_gains->Kd = global_best[2];
    *best_itae = global_cost;
    free(swarm); free(temp_out);
    return 0;
}

/* =================================================================
 * L6: Model Reference Adaptive Control (MRAC) with Lyapunov Stability
 * Plant: y_dot = -a*y + b*u, Reference: y_m_dot = -a_m*y_m + b_m*r
 * Control law: u = theta1*y + theta2*r, Adaptation: theta_dot = -gamma*e*y*sign(b)
 * ================================================================= */
typedef struct { double a, b, a_m, b_m, gamma, theta1, theta2, y, y_m, e; } MRAC;

int mrac_init(MRAC *m, double a_plant, double b_plant, double a_ref, double b_ref, double adapt_gain) {
    if (!m || adapt_gain <= 0) return -1;
    m->a = a_plant; m->b = b_plant; m->a_m = a_ref; m->b_m = b_ref;
    m->gamma = adapt_gain; m->theta1 = 0; m->theta2 = 1.0; m->y = 0; m->y_m = 0; m->e = 0;
    return 0;
}

int mrac_step(MRAC *m, double r, double dt, double *u_out, double *y_out) {
    if (!m || dt <= 0) return -1;
    double y = m->y, ym = m->y_m, e = m->e;
    double k1 = -m->a * y + m->b * (m->theta1 * y + m->theta2 * r);
    double k1_m = -m->a_m * ym + m->b_m * r;
    y += k1 * dt; ym += k1_m * dt;
    e = y - ym;
    double theta1_dot = -m->gamma * e * y * ((m->b > 0) ? 1.0 : -1.0);
    double theta2_dot = -m->gamma * e * r * ((m->b > 0) ? 1.0 : -1.0);
    m->theta1 += theta1_dot * dt; m->theta2 += theta2_dot * dt;
    m->y = y; m->y_m = ym; m->e = e;
    *u_out = m->theta1 * y + m->theta2 * r;
    *y_out = y;
    return 0;
}

/* =================================================================
 * L6: Iterative Learning Control (ILC) - P-type
 * u_{j+1}(t) = u_j(t) + L * e_j(t+1) for repetitive trajectory tracking
 * ================================================================= */
int ilc_p_type(double *u, const double *e_prev, int n, double learning_gain) {
    if (!u || !e_prev || n <= 0) return -1;
    for (int i = 0; i < n; i++)
        u[i] += learning_gain * e_prev[i];
    return 0;
}

int ilc_pd_type(double *u, const double *e_prev, int n, double Lp, double Ld, double dt) {
    if (!u || !e_prev || n <= 1 || dt <= 0) return -1;
    for (int i = 0; i < n; i++) {
        double de = (i > 0) ? (e_prev[i] - e_prev[i-1]) / dt : 0;
        u[i] += Lp * e_prev[i] + Ld * de;
    }
    return 0;
}

double ilc_convergence_rate(const double *e_curr, const double *e_prev, int n) {
    if (!e_curr || !e_prev || n <= 0) return -1;
    double norm_curr = 0, norm_prev = 0;
    for (int i = 0; i < n; i++) {
        norm_curr += e_curr[i] * e_curr[i];
        norm_prev += e_prev[i] * e_prev[i];
    }
    if (norm_prev <= 0) return 0;
    return sqrt(norm_curr / norm_prev);
}

/* =================================================================
 * L7: Self-Tuning Regulator (STR) - Recursive Least Squares + Pole Placement
 * Estimate a,b online, then compute controller gains from Diophantine equation.
 * ================================================================= */
typedef struct { double a_hat, b_hat, P[4], theta[2], lambda; } STR_RLS;

int str_rls_init(STR_RLS *s, double a0, double b0, double forgetting_factor) {
    if (!s || forgetting_factor <= 0 || forgetting_factor > 1) return -1;
    s->a_hat = a0; s->b_hat = b0; s->theta[0] = -a0; s->theta[1] = b0;
    s->P[0] = 100.0; s->P[1] = 0; s->P[2] = 0; s->P[3] = 100.0; s->lambda = forgetting_factor;
    return 0;
}

int str_rls_update(STR_RLS *s, double y_prev, double u_prev, double y_curr) {
    if (!s) return -1;
    double phi[2] = {y_prev, u_prev};
    double y_hat = s->theta[0] * phi[0] + s->theta[1] * phi[1];
    double e = y_curr - y_hat;
    double Pphi0 = s->P[0] * phi[0] + s->P[1] * phi[1];
    double Pphi1 = s->P[2] * phi[0] + s->P[3] * phi[1];
    double den = s->lambda + phi[0] * Pphi0 + phi[1] * Pphi1;
    if (fabs(den) < 1e-15) return -1;
    double K0 = Pphi0 / den, K1 = Pphi1 / den;
    s->theta[0] += K0 * e; s->theta[1] += K1 * e;
    s->a_hat = -s->theta[0]; s->b_hat = s->theta[1];
    double s00 = s->P[0], s01 = s->P[1], s10 = s->P[2], s11 = s->P[3];
    s->P[0] = (s00 - K0 * Pphi0) / s->lambda;
    s->P[1] = (s01 - K0 * Pphi1) / s->lambda;
    s->P[2] = (s10 - K1 * Pphi0) / s->lambda;
    s->P[3] = (s11 - K1 * Pphi1) / s->lambda;
    return 0;
}

int str_pole_placement_gains(STR_RLS *s, double p1, double p2, double *K, double *Kr) {
    double a = s->a_hat, b = s->b_hat;
    if (fabs(b) < 1e-15) return -1;
    double A_cl[3] = {1.0, -(p1 + p2), p1 * p2};
    *K = (A_cl[1] - a) / b;
    *Kr = A_cl[2] / b;
    return 0;
}

/* =================================================================
 * L8: Ant Colony Optimization (ACO) for control path planning
 * ================================================================= */
typedef struct { double pheromone; double heuristic; } ACONode;
int aco_control_path(ACONode *graph, int n_nodes, int n_ants, int n_iter,
                      double alpha, double beta, double rho, int *best_path, double *best_cost) {
    if (!graph || n_nodes < 2 || n_ants < 1 || !best_path || !best_cost) return -1;
    int *path = malloc(n_nodes * sizeof(int));
    double *costs = malloc(n_ants * sizeof(double));
    *best_cost = 1e12;
    for (int iter = 0; iter < n_iter; iter++) {
        for (int ant = 0; ant < n_ants; ant++) {
            int visited = 1, current = 0; path[0] = 0; costs[ant] = 0;
            while (visited < n_nodes) {
                double total = 0, probs[100] = {0};
                for (int j = 0; j < n_nodes && j < 100; j++) {
                    int idx = current * n_nodes + j;
                    probs[j] = pow(graph[idx].pheromone, alpha) * pow(graph[idx].heuristic, beta);
                    total += probs[j];
                }
                if (total <= 0) break;
                double r = (double)rand() / RAND_MAX * total, cum = 0;
                for (int j = 0; j < n_nodes && j < 100; j++) {
                    cum += probs[j];
                    if (cum >= r) { current = j; path[visited++] = j; costs[ant] += 1.0 / (graph[current*n_nodes+j].heuristic + 0.01); break; }
                }
            }
            if (costs[ant] < *best_cost) { *best_cost = costs[ant]; memcpy(best_path, path, n_nodes * sizeof(int)); }
        }
        for (int i = 0; i < n_nodes * n_nodes; i++) graph[i].pheromone *= (1.0 - rho);
        for (int ant = 0; ant < n_ants; ant++)
            for (int i = 1; i < n_nodes; i++) {
                int idx = path[i-1] * n_nodes + path[i];
                graph[idx].pheromone += 1.0 / (costs[ant] + 0.01);
            }
    }
    free(path); free(costs);
    return 0;
}

/* =================================================================
 * L8: Lyapunov-based adaptive neural control weight update
 * Single-layer network: u = W^T * phi(x), W_dot = -gamma * e * phi(x) * sign(g)
 * ================================================================= */
int lyapunov_neural_weight_update(const double *phi, int n_phi, double *W, double e,
                                   double g_sign, double gamma, double dt) {
    if (!phi || n_phi <= 0 || !W || dt <= 0) return -1;
    for (int i = 0; i < n_phi; i++)
        W[i] -= gamma * e * phi[i] * g_sign * dt;
    return 0;
}

double neural_control_signal(const double *phi, const double *W, int n_phi) {
    if (!phi || !W || n_phi <= 0) return 0;
    double u = 0;
    for (int i = 0; i < n_phi; i++) u += W[i] * phi[i];
    return u;
}

/* RBF activation: phi_i = exp(-||x - c_i||^2 / (2 * sigma^2)) */
int rbf_activation(const double *x, int nx, const double *centers, const double *sigmas,
                    int n_rbf, double *phi) {
    if (!x || !centers || !sigmas || !phi || nx <= 0 || n_rbf <= 0) return -1;
    for (int j = 0; j < n_rbf; j++) {
        double dist2 = 0;
        for (int i = 0; i < nx; i++)
            dist2 += (x[i] - centers[j * nx + i]) * (x[i] - centers[j * nx + i]);
        phi[j] = exp(-dist2 / (2.0 * sigmas[j] * sigmas[j]));
    }
    return 0;
}

/* Sliding mode reaching law: s_dot = -eta * sign(s) - k * s */
double smc_reaching_law(double s, double eta, double k) {
    double sign_s = (s > 0) ? 1.0 : ((s < 0) ? -1.0 : 0.0);
    return -eta * sign_s - k * s;
}

double smc_equivalent_control(double f_x, double g_x, double x_ddot_d, double lambda, double e, double de) {
    if (fabs(g_x) < 1e-15) return 0;
    double s = de + lambda * e;
    return (x_ddot_d - f_x + lambda * de) / g_x;
}

/* Extremum seeking: sinusoidal perturbation method */
double extremum_seeking_update(double theta, double *y_prev, double y_curr, double a, double omega,
                                double k, double dt, double t) {
    double sin_wt = sin(omega * t), cos_wt = cos(omega * t);
    double y_hpf = y_curr - *y_prev;
    double xi = y_hpf * sin_wt;
    double dtheta = k * xi * dt;
    *y_prev = y_curr;
    return theta + dtheta + a * sin_wt;
}
/* Adaptive fuzzy system: Gaussian membership, product inference, center-average defuzzification */
double fuzzy_gaussian_membership(double x, double c, double sigma) {
    return exp(-(x - c) * (x - c) / (2.0 * sigma * sigma));
}

int fuzzy_center_average_defuzzify(const double *rules_output, const double *firing_strength,
                                    int n_rules, double *crisp) {
    if (!rules_output || !firing_strength || n_rules <= 0 || !crisp) return -1;
    double num = 0, den = 0;
    for (int i = 0; i < n_rules; i++) { num += rules_output[i] * firing_strength[i]; den += firing_strength[i]; }
    *crisp = (den > 1e-15) ? num / den : 0;
    return 0;
}

/* Gradient descent fuzzy rule adaptation */
void fuzzy_rule_adapt(double *rule_consequents, const double *firing, int n_rules,
                       double error, double learning_rate) {
    double sum_f = 0; for (int i = 0; i < n_rules; i++) sum_f += firing[i];
    if (sum_f <= 0) return;
    for (int i = 0; i < n_rules; i++)
        rule_consequents[i] -= learning_rate * error * firing[i] / sum_f;
}

/* Direct MRAC with normalization: prevents parameter drift */
double mrac_normalized_adaptation_gain(double gamma, double phi_norm2, double epsilon) {
    return gamma / (1.0 + phi_norm2 + epsilon);
}
