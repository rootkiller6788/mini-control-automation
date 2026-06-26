#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* fuzzy_control.c - Fuzzy Logic Control: Mamdani & TSK inference */

double ic_fuzzy_membership(const ic_membership_func_t *mf, double x) {
    switch (mf->shape) {
    case IC_MF_TRIANGULAR: {
        double a = mf->params.tri.a, b = mf->params.tri.b, c = mf->params.tri.c;
        if (x <= a || x >= c) return 0.0;
        if (x <= b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }
    case IC_MF_TRAPEZOIDAL: {
        double a = mf->params.trap.a, b = mf->params.trap.b;
        double c = mf->params.trap.c, d = mf->params.trap.d;
        if (x <= a || x >= d) return 0.0;
        if (x >= b && x <= c) return 1.0;
        if (x < b) return (x - a) / (b - a);
        return (d - x) / (d - c);
    }
    case IC_MF_GAUSSIAN: {
        double center = mf->params.gauss.center;
        double sigma = mf->params.gauss.sigma;
        double diff = (x - center) / sigma;
        return exp(-0.5 * diff * diff);
    }
    case IC_MF_BELL: {
        double a = mf->params.bell.a, b = mf->params.bell.b, c = mf->params.bell.c;
        double diff = fabs((x - c) / a);
        return 1.0 / (1.0 + pow(diff, 2.0 * b));
    }
    case IC_MF_SIGMOID: {
        double a = mf->params.sig.a, c = mf->params.sig.c;
        return 1.0 / (1.0 + exp(-a * (x - c)));
    }
    default: return 0.0;
    }
}

static double ic_tnorm_compute(ic_tnorm_t tnorm, double a, double b) {
    switch (tnorm) {
    case IC_TNORM_MIN: return a < b ? a : b;
    case IC_TNORM_PROD: return a * b;
    case IC_TNORM_LUKASIEWICZ: { double v = a + b - 1.0; return v > 0.0 ? v : 0.0; }
    default: return a < b ? a : b;
    }
}

static double ic_snorm_compute(ic_snorm_t snorm, double a, double b) {
    switch (snorm) {
    case IC_SNORM_MAX: return a > b ? a : b;
    case IC_SNORM_PROB_SUM: return a + b - a * b;
    case IC_SNORM_LUKASIEWICZ: { double v = a + b; return v < 1.0 ? v : 1.0; }
    default: return a > b ? a : b;
    }
}

int ic_fuzzy_fuzzify(const ic_fuzzy_system_t *fis, const double *inputs,
                      double **firing_strengths) {
    if (!fis || !inputs || !firing_strengths) return -1;
    size_t i, j;
    for (i = 0; i < fis->num_rules; i++) {
        const ic_fuzzy_rule_t *rule = &fis->rules[i];
        double strength = 1.0;
        int first = 1;
        for (j = 0; j < rule->num_inputs; j++) {
            size_t term_idx = rule->antecedent_indices[j];
            if (term_idx >= fis->inputs[j].num_terms) { strength = 0.0; break; }
            double mu = ic_fuzzy_membership(&fis->inputs[j].terms[term_idx], inputs[j]);
            if (rule->connective == 0) {
                if (first) { strength = mu; first = 0; }
                else strength = ic_tnorm_compute(rule->tnorm, strength, mu);
            } else {
                if (first) { strength = mu; first = 0; }
                else strength = ic_snorm_compute(rule->snorm, strength, mu);
            }
        }
        firing_strengths[i][0] = strength * rule->weight;
    }
    return 0;
}

int ic_fuzzy_infer_mamdani(const ic_fuzzy_system_t *fis,
                             const double **firing_strengths,
                             double *output_samples, size_t num_samples,
                             double *output_range) {
    if (!fis || !firing_strengths || !output_samples || num_samples < 2) return -1;
    size_t oi, s, r;
    double step = (output_range[1] - output_range[0]) / (double)(num_samples - 1);
    for (oi = 0; oi < fis->num_outputs; oi++) {
        const ic_linguistic_var_t *outvar = &fis->outputs[oi];
        for (s = 0; s < num_samples; s++) {
            double y = output_range[0] + (double)s * step;
            double aggregated = 0.0;
            for (r = 0; r < fis->num_rules; r++) {
                double fs = firing_strengths[r][oi];
                if (fs < 1e-12) continue;
                size_t ci = fis->rules[r].consequent_index;
                double mu_cons = ic_fuzzy_membership(&outvar->terms[ci], y);
                double implied = (fis->implication == IC_IMPL_MAMDANI_MIN)
                    ? (fs < mu_cons ? fs : mu_cons) : fs * mu_cons;
                if (implied > aggregated) aggregated = implied;
            }
            output_samples[oi * num_samples + s] = aggregated;
        }
    }
    return 0;
}

int ic_fuzzy_defuzzify(const ic_fuzzy_system_t *fis,
                        const double *aggregated, double *crisp_out) {
    if (!fis || !aggregated || !crisp_out) return -1;
    size_t oi, s;
    for (oi = 0; oi < fis->num_outputs; oi++) {
        double numerator = 0.0, denominator = 0.0;
        for (s = 0; s < 100; s++) {
            double y = fis->outputs[oi].umin + (double)s *
                       (fis->outputs[oi].umax - fis->outputs[oi].umin) / 99.0;
            double val = aggregated[oi * 100 + s];
            if (fis->defuzz_method == IC_DEFUZZ_COA) {
                numerator += y * val; denominator += val;
            } else if (fis->defuzz_method == IC_DEFUZZ_BISECTOR) {
                numerator += val;
            } else if (fis->defuzz_method == IC_DEFUZZ_MOM) {
                if (val > denominator) { numerator = y; denominator = val; }
            }
        }
        if (fis->defuzz_method == IC_DEFUZZ_COA) {
            crisp_out[oi] = (denominator > 1e-15) ? numerator / denominator :
                            (fis->outputs[oi].umin + fis->outputs[oi].umax) / 2.0;
        } else if (fis->defuzz_method == IC_DEFUZZ_BISECTOR) {
            double total = numerator, half = total / 2.0, accum = 0.0;
            crisp_out[oi] = fis->outputs[oi].umin;
            for (s = 0; s < 100; s++) {
                double y = fis->outputs[oi].umin + (double)s *
                           (fis->outputs[oi].umax - fis->outputs[oi].umin) / 99.0;
                accum += aggregated[oi * 100 + s];
                if (accum >= half) { crisp_out[oi] = y; break; }
            }
        } else if (fis->defuzz_method == IC_DEFUZZ_MOM) {
            crisp_out[oi] = numerator;
        } else if (fis->defuzz_method == IC_DEFUZZ_SOM) {
            crisp_out[oi] = (fis->outputs[oi].umin + fis->outputs[oi].umax) / 2.0;
            for (s = 0; s < 100; s++) {
                double y = fis->outputs[oi].umin + (double)s *
                           (fis->outputs[oi].umax - fis->outputs[oi].umin) / 99.0;
                if (aggregated[oi * 100 + s] > 0.5) { crisp_out[oi] = y; break; }
            }
        } else {
            crisp_out[oi] = (fis->outputs[oi].umin + fis->outputs[oi].umax) / 2.0;
        }
    }
    return 0;
}

int ic_fuzzy_infer_tsk(const ic_fuzzy_system_t *fis,
                        const double *inputs, double *outputs) {
    if (!fis || !inputs || !outputs || !fis->is_tsk) return -1;
    size_t r, i, j;
    double *strengths = (double*)calloc(fis->num_rules, sizeof(double));
    if (!strengths) return -1;
    for (r = 0; r < fis->num_rules; r++) {
        const ic_fuzzy_rule_t *rule = &fis->rules[r];
        double strength = 1.0;
        for (i = 0; i < rule->num_inputs; i++) {
            size_t ti = rule->antecedent_indices[i];
            double mu = ic_fuzzy_membership(&fis->inputs[i].terms[ti], inputs[i]);
            strength = ic_tnorm_compute(rule->tnorm, strength, mu);
        }
        strengths[r] = strength * rule->weight;
    }
    double sum_strengths = 0.0;
    for (r = 0; r < fis->num_rules; r++) sum_strengths += strengths[r];
    if (sum_strengths < 1e-15) sum_strengths = 1.0;
    for (i = 0; i < fis->num_outputs; i++) outputs[i] = 0.0;
    for (r = 0; r < fis->num_rules; r++) {
        double ws = strengths[r] / sum_strengths;
        for (i = 0; i < fis->num_outputs; i++) {
            double z = fis->tsk_coeffs[r * (fis->num_inputs + 1) + fis->num_inputs];
            for (j = 0; j < fis->num_inputs; j++)
                z += fis->tsk_coeffs[r * (fis->num_inputs + 1) + j] * inputs[j];
            outputs[i] += ws * z;
        }
    }
    free(strengths);
    return 0;
}

int ic_fuzzy_control_step(ic_fuzzy_system_t *fis,
                           const double *inputs, double *outputs) {
    if (!fis || !inputs || !outputs) return -1;
    if (fis->is_tsk) return ic_fuzzy_infer_tsk(fis, inputs, outputs);
    size_t r, oi;
    double **firing = (double**)malloc(fis->num_rules * sizeof(double*));
    if (!firing) return -1;
    for (r = 0; r < fis->num_rules; r++) {
        firing[r] = (double*)calloc(fis->num_outputs, sizeof(double));
        if (!firing[r]) {
            for (size_t rr = 0; rr < r; rr++) free(firing[rr]);
            free(firing); return -1;
        }
    }
    ic_fuzzy_fuzzify(fis, inputs, firing);
    for (oi = 0; oi < fis->num_outputs; oi++) {
        double range[2] = {fis->outputs[oi].umin, fis->outputs[oi].umax};
        double *samples = (double*)calloc(fis->num_outputs * 100, sizeof(double));
        if (!samples) continue;
        ic_fuzzy_infer_mamdani(fis, (const double**)firing, samples, 100, range);
        ic_fuzzy_defuzzify(fis, samples, outputs);
        free(samples);
    }
    for (r = 0; r < fis->num_rules; r++) free(firing[r]);
    free(firing);
    return 0;
}

int ic_fuzzy_init(ic_fuzzy_system_t *fis, size_t num_inputs,
                   size_t num_outputs, int is_tsk) {
    if (!fis) return -1;
    memset(fis, 0, sizeof(ic_fuzzy_system_t));
    fis->num_inputs = num_inputs; fis->num_outputs = num_outputs;
    fis->is_tsk = is_tsk;
    fis->defuzz_method = IC_DEFUZZ_COA;
    fis->implication = IC_IMPL_MAMDANI_MIN;
    fis->tnorm = IC_TNORM_MIN; fis->snorm = IC_SNORM_MAX;
    fis->inputs = (ic_linguistic_var_t*)calloc(num_inputs, sizeof(ic_linguistic_var_t));
    fis->outputs = (ic_linguistic_var_t*)calloc(num_outputs, sizeof(ic_linguistic_var_t));
    if (!fis->inputs || !fis->outputs) { ic_fuzzy_free(fis); return -1; }
    return 0;
}

void ic_fuzzy_free(ic_fuzzy_system_t *fis) {
    if (!fis) return;
    size_t i;
    for (i = 0; i < fis->num_inputs; i++) free(fis->inputs[i].terms);
    for (i = 0; i < fis->num_outputs; i++) free(fis->outputs[i].terms);
    free(fis->inputs); free(fis->outputs);
    for (i = 0; i < fis->num_rules; i++) free(fis->rules[i].antecedent_indices);
    free(fis->rules); free(fis->tsk_coeffs);
    memset(fis, 0, sizeof(ic_fuzzy_system_t));
}

int ic_fuzzy_add_input(ic_fuzzy_system_t *fis, const char *name,
                        double umin, double umax) {
    if (!fis || !name) return -1;
    size_t idx = 0;
    while (idx < fis->num_inputs && fis->inputs[idx].name[0]) idx++;
    if (idx >= fis->num_inputs) return -1;
    strncpy(fis->inputs[idx].name, name, 31);
    fis->inputs[idx].umin = umin; fis->inputs[idx].umax = umax;
    return 0;
}

int ic_fuzzy_add_output(ic_fuzzy_system_t *fis, const char *name,
                         double umin, double umax) {
    if (!fis || !name) return -1;
    size_t idx = 0;
    while (idx < fis->num_outputs && fis->outputs[idx].name[0]) idx++;
    if (idx >= fis->num_outputs) return -1;
    strncpy(fis->outputs[idx].name, name, 31);
    fis->outputs[idx].umin = umin; fis->outputs[idx].umax = umax;
    return 0;
}

int ic_fuzzy_add_mf(ic_linguistic_var_t *var, ic_mf_shape_t shape,
                     const void *params, const char *name) {
    if (!var || !params || !name) return -1;
    size_t count = var->num_terms;
    ic_membership_func_t *new_terms = (ic_membership_func_t*)
        realloc(var->terms, (count + 1) * sizeof(ic_membership_func_t));
    if (!new_terms) return -1;
    var->terms = new_terms;
    ic_membership_func_t *mf = &var->terms[count];
    memset(mf, 0, sizeof(ic_membership_func_t));
    mf->shape = shape; strncpy(mf->name, name, 31);
    switch (shape) {
    case IC_MF_TRIANGULAR: memcpy(&mf->params.tri, params, sizeof(ic_mf_triangular_t)); break;
    case IC_MF_TRAPEZOIDAL: memcpy(&mf->params.trap, params, sizeof(ic_mf_trapezoidal_t)); break;
    case IC_MF_GAUSSIAN: memcpy(&mf->params.gauss, params, sizeof(ic_mf_gaussian_t)); break;
    case IC_MF_BELL: memcpy(&mf->params.bell, params, sizeof(ic_mf_bell_t)); break;
    case IC_MF_SIGMOID: memcpy(&mf->params.sig, params, sizeof(ic_mf_sigmoid_t)); break;
    default: return -1;
    }
    var->num_terms = count + 1;
    return 0;
}

int ic_fuzzy_add_rule(ic_fuzzy_system_t *fis, const size_t *antecedents,
                       size_t consequent, double weight, int connective) {
    if (!fis || !antecedents) return -1;
    size_t count = fis->num_rules;
    ic_fuzzy_rule_t *new_rules = (ic_fuzzy_rule_t*)
        realloc(fis->rules, (count + 1) * sizeof(ic_fuzzy_rule_t));
    if (!new_rules) return -1;
    fis->rules = new_rules;
    ic_fuzzy_rule_t *rule = &fis->rules[count];
    memset(rule, 0, sizeof(ic_fuzzy_rule_t));
    rule->num_inputs = fis->num_inputs;
    rule->antecedent_indices = (size_t*)malloc(fis->num_inputs * sizeof(size_t));
    if (!rule->antecedent_indices) return -1;
    memcpy(rule->antecedent_indices, antecedents, fis->num_inputs * sizeof(size_t));
    rule->connective = connective; rule->consequent_index = consequent;
    rule->weight = weight; rule->tnorm = fis->tnorm; rule->snorm = fis->snorm;
    fis->num_rules = count + 1;
    return 0;
}
