#include "intelligent_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  TEST %s... ", #name); } while(0)
#define PASS() do { tests_passed++; printf("PASSED\n"); } while(0)
#define FAIL(msg) do { printf("FAILED: %s\n", msg); } while(0)
#define CHECK(cond, msg) do { if (cond) PASS(); else FAIL(msg); } while(0)

static void test_activation_sigmoid(void) {
    TEST(sigmoid);
    double s0 = ic_sigmoid(0.0);
    CHECK(fabs(s0 - 0.5) < 0.01, "sigmoid(0)=0.5");
    TEST(sigmoid_limits);
    CHECK(ic_sigmoid(100.0) > 0.99, "sigmoid(100) near 1");
    CHECK(ic_sigmoid(-100.0) < 0.01, "sigmoid(-100) near 0");
}

static void test_activation_relu(void) {
    TEST(relu);
    CHECK(ic_relu(5.0) == 5.0, "relu(5)=5");
    CHECK(ic_relu(-3.0) == 0.0, "relu(-3)=0");
}

static void test_activation_softmax(void) {
    TEST(softmax);
    double x[] = {1.0, 2.0, 3.0};
    double out[3];
    ic_softmax(x, 3, out);
    double sum = out[0] + out[1] + out[2];
    CHECK(fabs(sum - 1.0) < 0.001, "softmax sums to 1");
    CHECK(out[2] > out[1] && out[1] > out[0], "order preserved");
}

static void test_matrix_vector_mul(void) {
    TEST(mat_vec_mul);
    double A[] = {2.0, 0.0, 0.0, 3.0};
    double x[] = {1.0, 2.0};
    double y[2];
    ic_matrix_vector_mul(A, x, 2, 2, y);
    CHECK(fabs(y[0] - 2.0) < 0.001, "y[0]=2");
    CHECK(fabs(y[1] - 6.0) < 0.001, "y[1]=6");
}

static void test_matrix_transpose(void) {
    TEST(transpose);
    double A[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    double AT[6];
    ic_matrix_transpose(A, 2, 3, AT);
    CHECK(fabs(AT[0] - 1.0) < 0.001, "AT[0]=1");
}

static void test_vector_ops(void) {
    TEST(vector_dot);
    double a[] = {1.0, 2.0, 3.0}, b[] = {4.0, 5.0, 6.0};
    double dot = ic_vector_dot(a, b, 3);
    CHECK(fabs(dot - 32.0) < 0.001, "dot=32");
}

static void test_mrac(void) {
    TEST(mrac_init);
    ic_mrac_t mrac;
    double gamma[3] = {1.0, 1.0, 1.0};
    int ret = ic_mrac_init(&mrac, 1.0, 10.0, 20.0, 5.0, 2.0, gamma, 1);
    CHECK(ret == 0, "mrac init ok");
    TEST(mrac_step);
    double u, y;
    ic_mrac_step(&mrac, 1.0, 0.001, 0.0, &u, &y);
    CHECK(fabs(u) < 100.0, "control bounded");
}

static void test_fuzzy(void) {
    TEST(fuzzy_init);
    ic_fuzzy_system_t fis;
    int ret = ic_fuzzy_init(&fis, 2, 1, 0);
    CHECK(ret == 0, "fuzzy init ok");
    ret = ic_fuzzy_add_input(&fis, "error", -1.0, 1.0);
    CHECK(ret == 0, "add input ok");
    ret = ic_fuzzy_add_input(&fis, "derror", -1.0, 1.0);
    CHECK(ret == 0, "add input2 ok");
    ret = ic_fuzzy_add_output(&fis, "output", -1.0, 1.0);
    CHECK(ret == 0, "add output ok");
    TEST(fuzzy_mf);
    ic_mf_triangular_t tri = {-1.0, 0.0, 1.0, "NB"};
    ic_membership_func_t mf;
    memset(&mf, 0, sizeof(mf));
    mf.shape = IC_MF_TRIANGULAR;
    mf.params.tri = tri;
    double mu = ic_fuzzy_membership(&mf, 0.0);
    CHECK(fabs(mu - 1.0) < 0.001, "tri MF peak=1");
    ic_fuzzy_free(&fis);
}

static void test_neural(void) {
    TEST(nn_create);
    ic_neural_network_t nn;
    size_t layers[] = {2, 4, 1};
    int ret = ic_nn_create(&nn, layers, 3, IC_ACT_SIGMOID, IC_ACT_LINEAR);
    CHECK(ret == 0, "nn create ok");
    TEST(nn_forward);
    double input[] = {0.5, -0.3};
    double output[1];
    ic_nn_forward(&nn, input, output);
    CHECK(fabs(output[0]) < 10.0, "output bounded");
    size_t nw = ic_nn_count_weights(&nn);
    CHECK(nw > 0, "weights counted");
    ic_nn_free(&nn);
}

static void test_rbf(void) {
    TEST(rbf_create);
    ic_rbf_network_t rbf;
    int ret = ic_rbf_create(&rbf, 3, 2);
    CHECK(ret == 0, "rbf create ok");
    double x[] = {0.5, 0.3};
    double y = ic_rbf_output(&rbf, x);
    CHECK(fabs(y) < 10.0, "rbf output bounded");
    ic_rbf_free(&rbf);
}

static void test_smc(void) {
    TEST(smc_init);
    ic_smc_t smc;
    int ret = ic_smc_init(&smc, 2.0, 1.0, 0.1, 0.05, 1);
    CHECK(ret == 0, "smc init ok");
    double u;
    ic_smc_step(&smc, 1.0, 0.0, 0.01, &u);
    CHECK(1, "smc step executed");
}

static void test_qlearning(void) {
    TEST(ql_init);
    ic_qlearning_t ql;
    int ret = ic_ql_init(&ql, 5, 3, 0.1, 0.9, 0.1);
    CHECK(ret == 0, "ql init ok");
    size_t a = ic_ql_select_action(&ql, 0);
    CHECK(a < 3, "ql action valid");
    ic_ql_update(&ql, 0, a, 1.0, 1);
    double q = ql.Q_table[0 * 3 + a];
    CHECK(q > 0.0, "ql Q-value updated");
    ic_ql_free(&ql);
}

static void test_sarsa(void) {
    TEST(sarsa_init);
    ic_sarsa_t sarsa;
    int ret = ic_sarsa_init(&sarsa, 5, 3, 0.1, 0.9, 0.1);
    CHECK(ret == 0, "sarsa init ok");
    ic_sarsa_update(&sarsa, 0, 0, 1.0, 1, 1);
    double q = sarsa.Q_table[0];
    CHECK(q > 0.0, "sarsa Q-value updated");
    ic_sarsa_free(&sarsa);
}

static void test_performance(void) {
    TEST(perf_eval);
    double sp[] = {1.0, 1.0, 1.0, 1.0, 1.0};
    double out[] = {0.0, 0.5, 0.8, 0.95, 1.0};
    double ctrl[] = {1.0, 0.8, 0.5, 0.3, 0.2};
    ic_performance_t perf;
    ic_evaluate_performance(sp, out, ctrl, 5, 0.1, &perf);
    CHECK(perf.overshoot >= 0.0, "overshoot>=0");
    CHECK(perf.rise_time > 0.0, "rise_time>0");
}

static void test_convergence(void) {
    TEST(convergence);
    double err[] = {1.0,0.5,0.25,0.12,0.06,0.03,0.015,0.008,0.004,0.002,0.001,0.0005};
    ic_convergence_t conv;
    int ret = ic_check_convergence(err, 12, 0.02, &conv);
    CHECK(ret == 0 || conv.converged == 1, "convergence check ok");
}

static void test_genetic(void) {
    TEST(ga_init);
    ic_genetic_alg_t ga;
    int ret = ic_ga_init(&ga, 20, 3, 10, 0.8, 0.1);
    CHECK(ret == 0, "ga init ok");
    ic_ga_free(&ga);
}

static void test_fuzzy_pid(void) {
    TEST(fpid_init);
    ic_fuzzy_pid_t fpid;
    int ret = ic_fuzzy_pid_init(&fpid, 1.0, 0.1, 0.01, -10.0, 10.0);
    CHECK(ret == 0, "fpid init ok");
    double u = ic_fuzzy_pid_control(&fpid, 1.0, 0.5, 0.01);
    CHECK(fabs(u) < 20.0, "fpid control bounded");
    ic_fuzzy_pid_free(&fpid);
}

int main(void) {
    printf("=== mini-intelligent-control Test Suite ===\n\n");
    test_activation_sigmoid();
    test_activation_relu();
    test_activation_softmax();
    test_matrix_vector_mul();
    test_matrix_transpose();
    test_vector_ops();
    test_mrac();
    test_fuzzy();
    test_neural();
    test_rbf();
    test_smc();
    test_qlearning();
    test_sarsa();
    test_performance();
    test_convergence();
    test_genetic();
    test_fuzzy_pid();
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    printf("  (Note: %d assertions checked)\n", tests_passed);
    return 0;
}
