#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* reinforcement_control.c - Q-Learning, SARSA, Replay Buffer, Policy Iteration */

#define Q_TABLE_GET(ql, s, a) ((ql)->Q_table[(s) * (ql)->num_actions + (a)])
#define SARSA_Q(sar, s, a) ((sar)->Q_table[(s) * (sar)->num_actions + (a)])

/* ---- L5: Q-Learning (Watkins 1989) ---- */

int ic_ql_init(ic_qlearning_t *ql, size_t num_states, size_t num_actions,
                double alpha, double gamma, double epsilon) {
    if (!ql || num_states == 0 || num_actions == 0) return -1;
    memset(ql, 0, sizeof(ic_qlearning_t));
    ql->num_states = num_states; ql->num_actions = num_actions;
    ql->alpha = alpha; ql->gamma = gamma; ql->epsilon = epsilon;
    ql->epsilon_decay = 0.999; ql->epsilon_min = 0.01;
    ql->Q_table = (double*)calloc(num_states * num_actions, sizeof(double));
    if (!ql->Q_table) return -1;
    return 0;
}

void ic_ql_free(ic_qlearning_t *ql) {
    if (!ql) return;
    free(ql->Q_table); memset(ql, 0, sizeof(ic_qlearning_t));
}

size_t ic_ql_select_action(ic_qlearning_t *ql, size_t state) {
    if (!ql || state >= ql->num_states) return 0;
    if (ic_random_uniform() < ql->epsilon) {
        return (size_t)(ic_random_uniform() * (double)ql->num_actions) % ql->num_actions;
    }
    return ic_ql_best_action(ql, state);
}

void ic_ql_update(ic_qlearning_t *ql, size_t state, size_t action,
                   double reward, size_t next_state) {
    if (!ql || state >= ql->num_states || action >= ql->num_actions) return;
    size_t idx = state * ql->num_actions + action;
    double max_q_next = Q_TABLE_GET(ql, next_state, 0);
    for (size_t a = 1; a < ql->num_actions; a++) {
        double q = Q_TABLE_GET(ql, next_state, a);
        if (q > max_q_next) max_q_next = q;
    }
    double td_error = reward + ql->gamma * max_q_next - ql->Q_table[idx];
    ql->Q_table[idx] += ql->alpha * td_error;
    if (ql->epsilon > ql->epsilon_min) ql->epsilon *= ql->epsilon_decay;
}

size_t ic_ql_best_action(const ic_qlearning_t *ql, size_t state) {
    if (!ql || state >= ql->num_states) return 0;
    size_t best_a = 0; double best_q = Q_TABLE_GET(ql, state, 0);
    for (size_t a = 1; a < ql->num_actions; a++) {
        double q = Q_TABLE_GET(ql, state, a);
        if (q > best_q) { best_q = q; best_a = a; }
    }
    return best_a;
}

/* ---- L5: SARSA (On-Policy TD Control) ---- */

int ic_sarsa_init(ic_sarsa_t *sarsa, size_t num_states, size_t num_actions,
                   double alpha, double gamma, double epsilon) {
    if (!sarsa || num_states == 0 || num_actions == 0) return -1;
    memset(sarsa, 0, sizeof(ic_sarsa_t));
    sarsa->num_states = num_states; sarsa->num_actions = num_actions;
    sarsa->alpha = alpha; sarsa->gamma = gamma; sarsa->epsilon = epsilon;
    sarsa->epsilon_decay = 0.999; sarsa->epsilon_min = 0.01;
    sarsa->Q_table = (double*)calloc(num_states * num_actions, sizeof(double));
    if (!sarsa->Q_table) return -1;
    return 0;
}

void ic_sarsa_free(ic_sarsa_t *sarsa) {
    if (!sarsa) return;
    free(sarsa->Q_table); memset(sarsa, 0, sizeof(ic_sarsa_t));
}

size_t ic_sarsa_select_action(ic_sarsa_t *sarsa, size_t state) {
    if (!sarsa || state >= sarsa->num_states) return 0;
    if (ic_random_uniform() < sarsa->epsilon)
        return (size_t)(ic_random_uniform() * (double)sarsa->num_actions) % sarsa->num_actions;
    size_t best_a = 0; double best_q = SARSA_Q(sarsa, state, 0);
    for (size_t a = 1; a < sarsa->num_actions; a++) {
        double q = SARSA_Q(sarsa, state, a);
        if (q > best_q) { best_q = q; best_a = a; }
    }
    return best_a;
}

void ic_sarsa_update(ic_sarsa_t *sarsa, size_t state, size_t action,
                      double reward, size_t next_state, size_t next_action) {
    if (!sarsa || state >= sarsa->num_states || action >= sarsa->num_actions) return;
    size_t idx = state * sarsa->num_actions + action;
    double q_next = SARSA_Q(sarsa, next_state, next_action);
    double td_error = reward + sarsa->gamma * q_next - sarsa->Q_table[idx];
    sarsa->Q_table[idx] += sarsa->alpha * td_error;
    if (sarsa->epsilon > sarsa->epsilon_min) sarsa->epsilon *= sarsa->epsilon_decay;
}

/* ---- L2: Experience Replay Buffer (for DQN-style learning) ---- */

int ic_replay_buffer_init(ic_replay_buffer_t *buf, size_t capacity,
                           size_t state_dim, size_t action_dim) {
    if (!buf || capacity == 0) return -1;
    memset(buf, 0, sizeof(ic_replay_buffer_t));
    buf->capacity = capacity; buf->state_dim = state_dim; buf->action_dim = action_dim;
    buf->states = (double*)calloc(capacity * state_dim, sizeof(double));
    buf->actions = (double*)calloc(capacity * action_dim, sizeof(double));
    buf->rewards = (double*)calloc(capacity, sizeof(double));
    buf->next_states = (double*)calloc(capacity * state_dim, sizeof(double));
    buf->terminals = (int*)calloc(capacity, sizeof(int));
    if (!buf->states || !buf->actions || !buf->rewards || !buf->next_states || !buf->terminals) {
        ic_replay_buffer_free(buf); return -1;
    }
    return 0;
}

void ic_replay_buffer_free(ic_replay_buffer_t *buf) {
    if (!buf) return;
    free(buf->states); free(buf->actions); free(buf->rewards);
    free(buf->next_states); free(buf->terminals);
    memset(buf, 0, sizeof(ic_replay_buffer_t));
}

void ic_replay_buffer_push(ic_replay_buffer_t *buf, const double *state,
                            const double *action, double reward,
                            const double *next_state, int terminal) {
    if (!buf || !state) return;
    size_t idx = buf->head;
    memcpy(&buf->states[idx * buf->state_dim], state, buf->state_dim * sizeof(double));
    if (action) memcpy(&buf->actions[idx * buf->action_dim], action, buf->action_dim * sizeof(double));
    buf->rewards[idx] = reward;
    if (next_state) memcpy(&buf->next_states[idx * buf->state_dim], next_state, buf->state_dim * sizeof(double));
    buf->terminals[idx] = terminal;
    buf->head = (idx + 1) % buf->capacity;
    if (buf->size < buf->capacity) buf->size++;
}

size_t ic_replay_buffer_sample(const ic_replay_buffer_t *buf,
                                double *states, double *actions,
                                double *rewards, double *next_states,
                                int *terminals, size_t batch_size) {
    if (!buf || !states || batch_size > buf->size) return 0;
    size_t sampled = 0;
    for (size_t i = 0; i < batch_size && sampled < buf->size; i++) {
        size_t idx = (size_t)(ic_random_uniform() * (double)buf->size) % buf->size;
        memcpy(&states[i * buf->state_dim], &buf->states[idx * buf->state_dim], buf->state_dim * sizeof(double));
        if (actions) memcpy(&actions[i * buf->action_dim], &buf->actions[idx * buf->action_dim], buf->action_dim * sizeof(double));
        rewards[i] = buf->rewards[idx];
        if (next_states) memcpy(&next_states[i * buf->state_dim], &buf->next_states[idx * buf->state_dim], buf->state_dim * sizeof(double));
        if (terminals) terminals[i] = buf->terminals[idx];
        sampled++;
    }
    return sampled;
}

/* ---- L8: Policy Iteration (Dynamic Programming) ---- */

int ic_policy_iter_init(ic_policy_iter_t *pi, size_t num_states,
                         size_t num_actions, double gamma, double theta) {
    if (!pi || num_states == 0 || num_actions == 0) return -1;
    memset(pi, 0, sizeof(ic_policy_iter_t));
    pi->num_states = num_states; pi->num_actions = num_actions;
    pi->gamma = gamma; pi->theta = theta;
    pi->V = (double*)calloc(num_states, sizeof(double));
    pi->policy = (double*)calloc(num_states, sizeof(double));
    if (!pi->V || !pi->policy) { ic_policy_iter_free(pi); return -1; }
    for (size_t s = 0; s < num_states; s++)
        pi->policy[s] = (double)(((size_t)(ic_random_uniform() * 1000.0)) % num_actions);
    return 0;
}

void ic_policy_iter_free(ic_policy_iter_t *pi) {
    if (!pi) return;
    free(pi->V); free(pi->policy);
    if (pi->P) { for (size_t a = 0; a < pi->num_actions; a++) free(pi->P[a]); free(pi->P); }
    if (pi->R) { for (size_t a = 0; a < pi->num_actions; a++) free(pi->R[a]); free(pi->R); }
    memset(pi, 0, sizeof(ic_policy_iter_t));
}

void ic_policy_evaluation(ic_policy_iter_t *pi) {
    if (!pi || !pi->P || !pi->R) return;
    size_t n = pi->num_states;
    double *V_new = (double*)malloc(n * sizeof(double));
    if (!V_new) return;
    for (size_t iter = 0; iter < 1000; iter++) {
        double delta = 0.0;
        for (size_t s = 0; s < n; s++) {
            double v = 0.0;
            size_t a = (size_t)pi->policy[s] % pi->num_actions;
            for (size_t sp = 0; sp < n; sp++)
                v += pi->P[a][s * n + sp] * (pi->R[a][s] + pi->gamma * pi->V[sp]);
            V_new[s] = v;
            double diff = fabs(v - pi->V[s]);
            if (diff > delta) delta = diff;
        }
        memcpy(pi->V, V_new, n * sizeof(double));
        if (delta < pi->theta) break;
    }
    free(V_new);
}

void ic_policy_improvement(ic_policy_iter_t *pi) {
    if (!pi || !pi->P || !pi->R) return;
    size_t n = pi->num_states;
    for (size_t s = 0; s < n; s++) {
        size_t old_a = (size_t)pi->policy[s] % pi->num_actions;
        double best_val = -1e100; size_t best_a = old_a;
        for (size_t a = 0; a < pi->num_actions; a++) {
            double val = 0.0;
            for (size_t sp = 0; sp < n; sp++)
                val += pi->P[a][s * n + sp] * (pi->R[a][s] + pi->gamma * pi->V[sp]);
            if (val > best_val) { best_val = val; best_a = a; }
        }
        pi->policy[s] = (double)best_a;
    }
}

int ic_policy_iteration(ic_policy_iter_t *pi, size_t max_iter) {
    if (!pi) return -1;
    for (size_t i = 0; i < max_iter; i++) {
        ic_policy_evaluation(pi);
        ic_policy_improvement(pi);
    }
    return 0;
}
