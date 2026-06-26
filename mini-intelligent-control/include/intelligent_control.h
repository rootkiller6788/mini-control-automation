#ifndef INTELLIGENT_CONTROL_H
#define INTELLIGENT_CONTROL_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif

/*
 * mini-intelligent-control -- Intelligent Control Theory
 * Covers: Fuzzy Logic, Neural Network, Adaptive, MPC, RL, SMC, GA
 * Reference: Astrom & Wittenmark (2013), Slotine & Li (1991),
 *            Sutton & Barto (2018), Wang (1997), Maciejowski (2002)
 */

/* ====== L1: Core Enumerations ====== */

typedef enum {
    IC_STRATEGY_FUZZY          = 0,
    IC_STRATEGY_NEURAL         = 1,
    IC_STRATEGY_ADAPTIVE_MRAC  = 2,
    IC_STRATEGY_ADAPTIVE_STR   = 3,
    IC_STRATEGY_MPC            = 4,
    IC_STRATEGY_QLEARNING      = 5,
    IC_STRATEGY_SARSA          = 6,
    IC_STRATEGY_SLIDING_MODE   = 7,
    IC_STRATEGY_ITERATIVE      = 8,
    IC_STRATEGY_GENETIC        = 9
} ic_strategy_t;

typedef enum {
    IC_MODE_OFFLINE_TRAIN  = 0,
    IC_MODE_ONLINE_ADAPT   = 1,
    IC_MODE_INFERENCE      = 2,
    IC_MODE_HYBRID         = 3
} ic_mode_t;

typedef enum {
    IC_ERROR_L1    = 0,
    IC_ERROR_L2    = 1,
    IC_ERROR_ITAE  = 2,
    IC_ERROR_ISE   = 3,
    IC_ERROR_ITSE  = 4
} ic_error_criterion_t;

typedef enum {
    IC_ACT_SIGMOID    = 0,
    IC_ACT_TANH       = 1,
    IC_ACT_RELU       = 2,
    IC_ACT_LEAKY_RELU = 3,
    IC_ACT_LINEAR     = 4,
    IC_ACT_SOFTMAX    = 5,
    IC_ACT_GAUSSIAN   = 6
} ic_activation_t;

typedef enum {
    IC_DEFUZZ_COA      = 0,
    IC_DEFUZZ_BISECTOR = 1,
    IC_DEFUZZ_MOM      = 2,
    IC_DEFUZZ_SOM      = 3,
    IC_DEFUZZ_LOM      = 4
} ic_defuzz_method_t;

typedef enum {
    IC_IMPL_MAMDANI_MIN  = 0,
    IC_IMPL_MAMDANI_PROD = 1,
    IC_IMPL_LARSEN       = 2
} ic_implication_t;

typedef enum {
    IC_TNORM_MIN         = 0,
    IC_TNORM_PROD        = 1,
    IC_TNORM_LUKASIEWICZ = 2
} ic_tnorm_t;

typedef enum {
    IC_SNORM_MAX         = 0,
    IC_SNORM_PROB_SUM    = 1,
    IC_SNORM_LUKASIEWICZ = 2
} ic_snorm_t;

typedef enum {
    IC_MF_TRIANGULAR  = 0,
    IC_MF_TRAPEZOIDAL = 1,
    IC_MF_GAUSSIAN    = 2,
    IC_MF_BELL        = 3,
    IC_MF_SIGMOID     = 4
} ic_mf_shape_t;

/* ====== L1: Membership Function Structs ====== */

typedef struct {
    double a, b, c;
    char   name[32];
} ic_mf_triangular_t;

typedef struct {
    double a, b, c, d;
    char   name[32];
} ic_mf_trapezoidal_t;

typedef struct {
    double center, sigma;
    char   name[32];
} ic_mf_gaussian_t;

typedef struct {
    double a, b, c;
    char   name[32];
} ic_mf_bell_t;

typedef struct {
    double a, c;
    char   name[32];
} ic_mf_sigmoid_t;

typedef struct {
    ic_mf_shape_t shape;
    char          name[32];
    union {
        ic_mf_triangular_t  tri;
        ic_mf_trapezoidal_t trap;
        ic_mf_gaussian_t    gauss;
        ic_mf_bell_t        bell;
        ic_mf_sigmoid_t     sig;
    } params;
} ic_membership_func_t;

typedef struct {
    char                  name[32];
    double                umin, umax;
    size_t                num_terms;
    ic_membership_func_t *terms;
} ic_linguistic_var_t;

typedef struct {
    size_t      num_inputs;
    size_t     *antecedent_indices;
    int         connective;
    size_t      consequent_index;
    double      weight;
    ic_tnorm_t  tnorm;
    ic_snorm_t  snorm;
} ic_fuzzy_rule_t;

typedef struct {
    ic_strategy_t       strategy;
    ic_defuzz_method_t  defuzz_method;
    ic_implication_t    implication;
    ic_tnorm_t          tnorm;
    ic_snorm_t          snorm;
    size_t              num_inputs;
    size_t              num_outputs;
    ic_linguistic_var_t *inputs;
    ic_linguistic_var_t *outputs;
    size_t              num_rules;
    ic_fuzzy_rule_t    *rules;
    double             *tsk_coeffs;
    int                 is_tsk;
} ic_fuzzy_system_t;

/* ====== L1: Neural Network Structs ====== */

typedef struct {
    size_t          num_inputs;
    double         *weights;
    double          bias;
    double          output;
    double          delta;
    ic_activation_t activation;
} ic_neuron_t;

typedef struct {
    size_t          num_neurons;
    size_t          num_inputs;
    ic_neuron_t    *neurons;
    double         *outputs;
    ic_activation_t activation;
} ic_nn_layer_t;

typedef struct {
    size_t          num_layers;
    ic_nn_layer_t  *layers;
    double           learning_rate;
    double           momentum;
    double          *prev_dw;
    size_t           total_weights;
    ic_activation_t  output_activation;
    ic_mode_t        mode;
} ic_neural_network_t;

typedef struct {
    size_t   num_centers;
    size_t   input_dim;
    double  *centers;
    double  *sigmas;
    double  *weights;
    double   bias;
    double   learning_rate;
} ic_rbf_network_t;

/* ====== L1: MRAC Struct ====== */

typedef struct {
    double zeta_ref, wn_ref;
    double a0_nom, a1_nom, b_nom;
    double gamma_theta[3];
    double theta[3];
    double ym, ym_dot, ym_ddot;
    double y, y_dot, y_ddot;
    double error, error_dot;
    double u;
    int    use_lyapunov;
    double sigma_mod;
} ic_mrac_t;

/* ====== L1: STR Struct ====== */

typedef struct {
    size_t   na, nb, nc, nk;
    double  *A, *B, *C;
    double   lambda_forget;
    double  *theta_hat;
    double  *P_matrix;
    size_t   param_dim;
    double  *phi_vector;
    double   u, y;
    double  *R, *S, *T;
    size_t   nr, ns, nt;
} ic_str_t;

/* ====== L1: Gain Scheduling Struct ====== */

typedef struct {
    size_t   num_sched_vars;
    double  *sched_vars;
    double  *sched_ranges;
    size_t   poly_order;
    double  *coeff_table;
    size_t   num_gains;
    double  *current_gains;
} ic_gain_schedule_t;

/* ====== L1: MPC Structs ====== */

typedef struct {
    size_t   nx, nu, ny;
    size_t   Np, Nc;
    double   Ts;
    double  *A, *B, *C;
    double  *Q, *R;
    double  *umin, *umax;
    double  *dumin, *dumax;
    double  *ymin, *ymax;
    size_t   max_qp_iter;
    double   qp_tol;
    double  *H, *Phi, *Gamma;
} ic_mpc_config_t;

typedef struct {
    const ic_mpc_config_t *config;
    double  *x, *x_pred, *u_opt, *u_prev, *ref, *y_pred;
    double   cost;
    size_t   qp_iterations;
    int      qp_converged;
} ic_mpc_state_t;

/* ====== L1: RL Structs ====== */

typedef struct {
    size_t   num_states, num_actions;
    double   alpha, gamma, epsilon, epsilon_decay, epsilon_min;
    double  *Q_table;
} ic_qlearning_t;

typedef struct {
    size_t   num_states, num_actions;
    double   alpha, gamma, epsilon, epsilon_decay, epsilon_min;
    double  *Q_table;
} ic_sarsa_t;

typedef struct {
    size_t   capacity, size, state_dim, action_dim;
    double  *states, *actions, *rewards, *next_states;
    int     *terminals;
    size_t   head;
} ic_replay_buffer_t;

/* ====== L1: SMC Struct ====== */

typedef struct {
    double lambda, K, eta, phi_boundary;
    double s, e, e_dot, e_int;
    double u_eq, u_sw, u;
    double a0, a1, b;
    int    use_saturation;
} ic_smc_t;

/* ====== L1: Performance Structs ====== */

typedef struct {
    double rise_time, settling_time, overshoot, steady_state_error;
    double ise, iae, itae;
    double control_effort_rms, robustness_margin;
} ic_performance_t;

typedef struct {
    double learning_curve[100];
    size_t curve_length;
    double asymptotic_error, convergence_rate;
    size_t convergence_samples;
    int    converged;
    double lyapunov_derivative, lyapunov_value;
} ic_convergence_t;

/* ====== L1: Genetic Algorithm Struct ====== */

typedef struct {
    size_t   pop_size, chromo_length, max_generations;
    double   crossover_rate, mutation_rate, elite_fraction;
    double  *population, *fitness;
    double  *lower_bound, *upper_bound;
    double   best_fitness;
    double  *best_chromosome;
    int      selection_method;
    size_t   tournament_size;
} ic_genetic_alg_t;

/* ====== L2: Hybrid Structs ====== */

typedef struct {
    ic_fuzzy_system_t   fis;
    ic_neural_network_t nn;
    size_t              num_mf;
    double             *premise_params;
    double             *consequent_params;
    double              learning_rate_premise;
    double              learning_rate_consequent;
} ic_anfis_t;

typedef struct {
    double Kp, Ki, Kd, Kp0, Ki0, Kd0;
    ic_fuzzy_system_t *fis_kp, *fis_ki, *fis_kd;
    double e_prev, integral, e, de;
    double u_min, u_max;
    int    anti_windup;
} ic_fuzzy_pid_t;

/* ====== API DECLARATIONS ====== */

/* Lifecycle */
int  ic_system_init(ic_strategy_t strategy, void *controller);
void ic_system_free(ic_strategy_t strategy, void *controller);

/* Fuzzy Logic Control */
int  ic_fuzzy_init(ic_fuzzy_system_t *fis, size_t num_inputs, size_t num_outputs, int is_tsk);
void ic_fuzzy_free(ic_fuzzy_system_t *fis);
int  ic_fuzzy_add_input(ic_fuzzy_system_t *fis, const char *name, double umin, double umax);
int  ic_fuzzy_add_output(ic_fuzzy_system_t *fis, const char *name, double umin, double umax);
int  ic_fuzzy_add_mf(ic_linguistic_var_t *var, ic_mf_shape_t shape, const void *params, const char *name);
int  ic_fuzzy_add_rule(ic_fuzzy_system_t *fis, const size_t *antecedents, size_t consequent, double weight, int connective);
double ic_fuzzy_membership(const ic_membership_func_t *mf, double x);
int  ic_fuzzy_fuzzify(const ic_fuzzy_system_t *fis, const double *inputs, double **firing_strengths);
int  ic_fuzzy_infer_mamdani(const ic_fuzzy_system_t *fis, const double **firing_strengths, double *output_samples, size_t num_samples, double *output_range);
int  ic_fuzzy_infer_tsk(const ic_fuzzy_system_t *fis, const double *inputs, double *outputs);
int  ic_fuzzy_defuzzify(const ic_fuzzy_system_t *fis, const double *aggregated, double *crisp_out);
int  ic_fuzzy_control_step(ic_fuzzy_system_t *fis, const double *inputs, double *outputs);

/* Neural Network Control */
int  ic_nn_create(ic_neural_network_t *nn, const size_t *layer_sizes, size_t num_layers, ic_activation_t hidden_act, ic_activation_t output_act);
void ic_nn_free(ic_neural_network_t *nn);
void ic_nn_forward(ic_neural_network_t *nn, const double *input, double *output);
void ic_nn_backprop(ic_neural_network_t *nn, const double *input, const double *target, double *output);
void ic_nn_train_batch(ic_neural_network_t *nn, const double **inputs, const double **targets, size_t batch_size, size_t epochs);
void ic_nn_set_weights(ic_neural_network_t *nn, const double *flat_weights);
void ic_nn_get_weights(const ic_neural_network_t *nn, double *flat_weights);
size_t ic_nn_count_weights(const ic_neural_network_t *nn);

/* RBF Network Control */
int  ic_rbf_create(ic_rbf_network_t *rbf, size_t num_centers, size_t input_dim);
void ic_rbf_free(ic_rbf_network_t *rbf);
double ic_rbf_kernel(const ic_rbf_network_t *rbf, const double *x, size_t ci);
double ic_rbf_output(const ic_rbf_network_t *rbf, const double *x);
void ic_rbf_train_lms(ic_rbf_network_t *rbf, const double *x, double target);

/* MRAC */
int  ic_mrac_init(ic_mrac_t *mrac, double zeta_ref, double wn_ref, double a0, double a1, double b, const double *gamma, int use_lyapunov);
void ic_mrac_step(ic_mrac_t *mrac, double r, double dt, double y_meas, double *u_out, double *y_out);
void ic_mrac_reset(ic_mrac_t *mrac);
int  ic_mrac_lyapunov_check(const ic_mrac_t *mrac);

/* STR */
int  ic_str_init(ic_str_t *str, size_t na, size_t nb, size_t nc, size_t nk, double lambda);
void ic_str_free(ic_str_t *str);
void ic_str_identify_rls(ic_str_t *str, double u, double y);
int  ic_str_solve_diophantine(ic_str_t *str);
double ic_str_compute_control(ic_str_t *str, double ref);

/* Gain Scheduling */
int  ic_gain_sched_init(ic_gain_schedule_t *gs, size_t num_vars, size_t poly_order, size_t num_gains);
void ic_gain_sched_free(ic_gain_schedule_t *gs);
void ic_gain_sched_interpolate(ic_gain_schedule_t *gs, const double *sched_vars, double *gains);

/* MPC */
int  ic_mpc_init(ic_mpc_config_t *cfg, size_t nx, size_t nu, size_t ny, size_t Np, size_t Nc, double Ts);
void ic_mpc_free(ic_mpc_config_t *cfg);
int  ic_mpc_state_init(ic_mpc_state_t *state, const ic_mpc_config_t *cfg);
void ic_mpc_state_free(ic_mpc_state_t *state);
int  ic_mpc_compute_prediction_matrices(ic_mpc_config_t *cfg);
int  ic_mpc_build_qp(const ic_mpc_state_t *state, double *H_out, double *f_out, double *A_out, double *lb_out, double *ub_out);
int  ic_mpc_solve_qp_activeset(double *H, double *f, size_t n, const double *lb, const double *ub, double *x_opt, size_t max_iter, double tol);
int  ic_mpc_control_step(ic_mpc_state_t *state, const double *x_meas, const double *ref, double *u_out);

/* Sliding Mode Control */
int  ic_smc_init(ic_smc_t *smc, double lambda, double K, double eta, double phi, int use_sat);
void ic_smc_step(ic_smc_t *smc, double ref, double y_meas, double dt, double *u_out);
double ic_smc_sliding_surface(const ic_smc_t *smc);
double ic_smc_reaching_condition(const ic_smc_t *smc);

/* Q-Learning */
int  ic_ql_init(ic_qlearning_t *ql, size_t num_states, size_t num_actions, double alpha, double gamma, double epsilon);
void ic_ql_free(ic_qlearning_t *ql);
size_t ic_ql_select_action(ic_qlearning_t *ql, size_t state);
void ic_ql_update(ic_qlearning_t *ql, size_t state, size_t action, double reward, size_t next_state);
size_t ic_ql_best_action(const ic_qlearning_t *ql, size_t state);

/* SARSA */
int  ic_sarsa_init(ic_sarsa_t *sarsa, size_t num_states, size_t num_actions, double alpha, double gamma, double epsilon);
void ic_sarsa_free(ic_sarsa_t *sarsa);
size_t ic_sarsa_select_action(ic_sarsa_t *sarsa, size_t state);
void ic_sarsa_update(ic_sarsa_t *sarsa, size_t state, size_t action, double reward, size_t next_state, size_t next_action);

/* Experience Replay Buffer */
int  ic_replay_buffer_init(ic_replay_buffer_t *buf, size_t capacity, size_t state_dim, size_t action_dim);
void ic_replay_buffer_free(ic_replay_buffer_t *buf);
void ic_replay_buffer_push(ic_replay_buffer_t *buf, const double *state, const double *action, double reward, const double *next_state, int terminal);
size_t ic_replay_buffer_sample(const ic_replay_buffer_t *buf, double *states, double *actions, double *rewards, double *next_states, int *terminals, size_t batch_size);

/* Genetic Algorithm */
int  ic_ga_init(ic_genetic_alg_t *ga, size_t pop_size, size_t chromo_length, size_t max_gen, double cross_rate, double mut_rate);
void ic_ga_free(ic_genetic_alg_t *ga);
void ic_ga_evaluate_population(ic_genetic_alg_t *ga, double (*fitness_func)(const double*, void*), void *user_data);
void ic_ga_evolve(ic_genetic_alg_t *ga);
void ic_ga_get_best(const ic_genetic_alg_t *ga, double *best_chromosome, double *best_fitness);

/* Fuzzy-PID Hybrid */
int  ic_fuzzy_pid_init(ic_fuzzy_pid_t *fpid, double Kp0, double Ki0, double Kd0, double u_min, double u_max);
void ic_fuzzy_pid_free(ic_fuzzy_pid_t *fpid);
double ic_fuzzy_pid_control(ic_fuzzy_pid_t *fpid, double setpoint, double measurement, double dt);

/* ANFIS */
int  ic_anfis_init(ic_anfis_t *anfis, size_t num_inputs, size_t num_rules);
void ic_anfis_free(ic_anfis_t *anfis);
double ic_anfis_output(ic_anfis_t *anfis, const double *inputs);
void ic_anfis_train_step(ic_anfis_t *anfis, const double *inputs, double target);

/* Performance Evaluation */
void ic_evaluate_performance(const double *setpoint, const double *output, const double *control, size_t n, double dt, ic_performance_t *perf);
int  ic_check_convergence(const double *error_sequence, size_t n, double tol, ic_convergence_t *conv);

/* Activation Functions */
double ic_sigmoid(double x);
double ic_sigmoid_derivative(double x);
double ic_tanh_custom(double x);
double ic_tanh_derivative(double x);
double ic_relu(double x);
double ic_relu_derivative(double x);
double ic_leaky_relu(double x, double alpha);
double ic_leaky_relu_derivative(double x, double alpha);
double ic_linear(double x);
double ic_linear_derivative(double x);
double ic_activation(ic_activation_t type, double x);
double ic_activation_derivative(ic_activation_t type, double x);
void  ic_softmax(const double *x, size_t n, double *out);

/* Matrix/Vector Utilities */
void ic_matrix_vector_mul(const double *A, const double *x, size_t m, size_t n, double *y);
void ic_matrix_transpose(const double *A, size_t m, size_t n, double *AT);
int  ic_matrix_inverse_cholesky(const double *A, size_t n, double *Ainv);
int  ic_solve_linear_system(const double *A, const double *b, size_t n, double *x);
void ic_vector_add(const double *a, const double *b, size_t n, double *c);
void ic_vector_sub(const double *a, const double *b, size_t n, double *c);
double ic_vector_dot(const double *a, const double *b, size_t n);
double ic_vector_norm(const double *a, size_t n);
void ic_vector_scale(double *a, double alpha, size_t n);

/* Lyapunov Theoretic Analysis */
double ic_lyapunov_quadratic(const double *x, const double *P, size_t n);
int  ic_lyapunov_solve(const double *A, size_t n, double *P);

/* Random Number Generation */
double ic_random_uniform(void);
double ic_random_gaussian(void);
void  ic_random_seed(uint64_t seed);

/* Iterative Learning Control (L8) */
typedef struct {
    size_t   n_samples;
    size_t   n_inputs;
    size_t   n_outputs;
    double   learning_gain;
    double  *error_memory;
    double  *control_memory;
    double  *L_matrix;
} ic_ilc_t;
int  ic_ilc_init(ic_ilc_t *ilc, size_t n_samples, size_t n_inputs, size_t n_outputs, double gain);
void ic_ilc_free(ic_ilc_t *ilc);
void ic_ilc_update(ic_ilc_t *ilc, const double *error, double *control);
void ic_ilc_reset(ic_ilc_t *ilc);

/* Policy Iteration (L5-L8) */
typedef struct {
    size_t   num_states, num_actions;
    double   gamma, theta;
    double  *V;
    double  *policy;
    double **P;     /* transition probabilities [action][state*state] */
    double **R;     /* rewards [action][state] */
} ic_policy_iter_t;
int  ic_policy_iter_init(ic_policy_iter_t *pi, size_t num_states, size_t num_actions, double gamma, double theta);
void ic_policy_iter_free(ic_policy_iter_t *pi);
void ic_policy_evaluation(ic_policy_iter_t *pi);
void ic_policy_improvement(ic_policy_iter_t *pi);
int  ic_policy_iteration(ic_policy_iter_t *pi, size_t max_iter);

#ifdef __cplusplus
}
#endif

#endif /* INTELLIGENT_CONTROL_H */
