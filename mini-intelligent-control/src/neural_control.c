#include "intelligent_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* neural_control.c - Neural Network Control: MLP + RBF training and inference */

/* ---- L2/L5: MLP Neural Network ---- */

int ic_nn_create(ic_neural_network_t *nn, const size_t *layer_sizes,
                  size_t num_layers, ic_activation_t hidden_act,
                  ic_activation_t output_act) {
    if (!nn || !layer_sizes || num_layers < 2) return -1;
    memset(nn, 0, sizeof(ic_neural_network_t));
    nn->num_layers = num_layers;
    nn->learning_rate = 0.01;
    nn->momentum = 0.0;
    nn->output_activation = output_act;
    nn->mode = IC_MODE_OFFLINE_TRAIN;
    nn->layers = (ic_nn_layer_t*)calloc(num_layers, sizeof(ic_nn_layer_t));
    if (!nn->layers) return -1;
    nn->total_weights = 0;
    size_t i, j;
    for (i = 0; i < num_layers; i++) {
        ic_nn_layer_t *layer = &nn->layers[i];
        layer->num_neurons = layer_sizes[i];
        layer->num_inputs = (i == 0) ? layer_sizes[0] : layer_sizes[i - 1];
        layer->activation = (i == num_layers - 1) ? output_act : hidden_act;
        layer->neurons = (ic_neuron_t*)calloc(layer->num_neurons, sizeof(ic_neuron_t));
        layer->outputs = (double*)calloc(layer->num_neurons, sizeof(double));
        if (!layer->neurons || !layer->outputs) { ic_nn_free(nn); return -1; }
        for (j = 0; j < layer->num_neurons; j++) {
            ic_neuron_t *neuron = &layer->neurons[j];
            neuron->num_inputs = layer->num_inputs;
            neuron->weights = (double*)calloc(layer->num_inputs, sizeof(double));
            if (!neuron->weights) { ic_nn_free(nn); return -1; }
            neuron->bias = ((double)rand() / RAND_MAX - 0.5) * 0.1;
            neuron->activation = layer->activation;
            for (size_t k = 0; k < layer->num_inputs; k++) {
                neuron->weights[k] = ((double)rand() / RAND_MAX - 0.5) * 0.1;
            }
            nn->total_weights += layer->num_inputs + 1;
        }
    }
    nn->prev_dw = (double*)calloc(nn->total_weights, sizeof(double));
    if (!nn->prev_dw && nn->momentum > 0.0) { /* ok if null */ }
    return 0;
}

void ic_nn_free(ic_neural_network_t *nn) {
    if (!nn) return;
    for (size_t i = 0; i < nn->num_layers; i++) {
        ic_nn_layer_t *layer = &nn->layers[i];
        if (layer->neurons) {
            for (size_t j = 0; j < layer->num_neurons; j++) {
                free(layer->neurons[j].weights);
            }
            free(layer->neurons);
        }
        free(layer->outputs);
    }
    free(nn->layers);
    free(nn->prev_dw);
    memset(nn, 0, sizeof(ic_neural_network_t));
}

void ic_nn_forward(ic_neural_network_t *nn, const double *input, double *output) {
    if (!nn || !input || !output) return;
    const double *layer_input = input;
    size_t i, j, k;
    for (i = 0; i < nn->num_layers; i++) {
        ic_nn_layer_t *layer = &nn->layers[i];
        for (j = 0; j < layer->num_neurons; j++) {
            ic_neuron_t *neuron = &layer->neurons[j];
            double sum = neuron->bias;
            for (k = 0; k < layer->num_inputs; k++) {
                sum += neuron->weights[k] * layer_input[k];
            }
            neuron->output = ic_activation(neuron->activation, sum);
            layer->outputs[j] = neuron->output;
        }
        layer_input = layer->outputs;
    }
    ic_nn_layer_t *last = &nn->layers[nn->num_layers - 1];
    memcpy(output, last->outputs, last->num_neurons * sizeof(double));
}

void ic_nn_backprop(ic_neural_network_t *nn, const double *input,
                     const double *target, double *output) {
    if (!nn || !input || !target) return;
    ic_nn_forward(nn, input, output);
    size_t i, j, k;
    /* Output layer delta */
    ic_nn_layer_t *last = &nn->layers[nn->num_layers - 1];
    for (j = 0; j < last->num_neurons; j++) {
        double o = last->neurons[j].output;
        double error = target[j] - o;
        last->neurons[j].delta = error * ic_activation_derivative(last->activation, o);
    }
    /* Backpropagate */
    for (i = nn->num_layers - 1; i > 0; i--) {
        ic_nn_layer_t *layer = &nn->layers[i];
        ic_nn_layer_t *prev = &nn->layers[i - 1];
        for (j = 0; j < prev->num_neurons; j++) {
            double error = 0.0;
            for (k = 0; k < layer->num_neurons; k++) {
                error += layer->neurons[k].weights[j] * layer->neurons[k].delta;
            }
            prev->neurons[j].delta = error * ic_activation_derivative(
                prev->activation, prev->neurons[j].output);
        }
    }
    /* Update weights */
    const double *layer_input = input;
    for (i = 0; i < nn->num_layers; i++) {
        ic_nn_layer_t *layer = &nn->layers[i];
        for (j = 0; j < layer->num_neurons; j++) {
            ic_neuron_t *neuron = &layer->neurons[j];
            double delta = neuron->delta * nn->learning_rate;
            neuron->bias += delta;
            for (k = 0; k < layer->num_inputs; k++) {
                neuron->weights[k] += delta * layer_input[k];
            }
        }
        layer_input = layer->outputs;
    }
}

void ic_nn_train_batch(ic_neural_network_t *nn, const double **inputs,
                        const double **targets, size_t batch_size, size_t epochs) {
    if (!nn || !inputs || !targets) return;
    size_t e, b;
    for (e = 0; e < epochs; e++) {
        for (b = 0; b < batch_size; b++) {
            double *out_buf = (double*)malloc(
                nn->layers[nn->num_layers - 1].num_neurons * sizeof(double));
            if (!out_buf) continue;
            ic_nn_backprop(nn, inputs[b], targets[b], out_buf);
            free(out_buf);
        }
    }
}

void ic_nn_set_weights(ic_neural_network_t *nn, const double *flat_weights) {
    if (!nn || !flat_weights) return;
    size_t idx = 0;
    for (size_t i = 0; i < nn->num_layers; i++) {
        ic_nn_layer_t *layer = &nn->layers[i];
        for (size_t j = 0; j < layer->num_neurons; j++) {
            ic_neuron_t *neuron = &layer->neurons[j];
            neuron->bias = flat_weights[idx++];
            for (size_t k = 0; k < layer->num_inputs; k++) {
                neuron->weights[k] = flat_weights[idx++];
            }
        }
    }
}

void ic_nn_get_weights(const ic_neural_network_t *nn, double *flat_weights) {
    if (!nn || !flat_weights) return;
    size_t idx = 0;
    for (size_t i = 0; i < nn->num_layers; i++) {
        ic_nn_layer_t *layer = &nn->layers[i];
        for (size_t j = 0; j < layer->num_neurons; j++) {
            ic_neuron_t *neuron = &layer->neurons[j];
            flat_weights[idx++] = neuron->bias;
            for (size_t k = 0; k < layer->num_inputs; k++) {
                flat_weights[idx++] = neuron->weights[k];
            }
        }
    }
}

size_t ic_nn_count_weights(const ic_neural_network_t *nn) {
    if (!nn) return 0;
    return nn->total_weights;
}

/* ---- L5: RBF Network ---- */

int ic_rbf_create(ic_rbf_network_t *rbf, size_t num_centers, size_t input_dim) {
    if (!rbf || num_centers == 0 || input_dim == 0) return -1;
    memset(rbf, 0, sizeof(ic_rbf_network_t));
    rbf->num_centers = num_centers;
    rbf->input_dim = input_dim;
    rbf->learning_rate = 0.01;
    rbf->centers = (double*)calloc(num_centers * input_dim, sizeof(double));
    rbf->sigmas = (double*)calloc(num_centers, sizeof(double));
    rbf->weights = (double*)calloc(num_centers, sizeof(double));
    if (!rbf->centers || !rbf->sigmas || !rbf->weights) {
        ic_rbf_free(rbf); return -1;
    }
    for (size_t i = 0; i < num_centers; i++) rbf->sigmas[i] = 1.0;
    rbf->bias = 0.0;
    return 0;
}

void ic_rbf_free(ic_rbf_network_t *rbf) {
    if (!rbf) return;
    free(rbf->centers); free(rbf->sigmas); free(rbf->weights);
    memset(rbf, 0, sizeof(ic_rbf_network_t));
}

/* Gaussian RBF kernel: phi(r) = exp(-r^2 / (2*sigma^2)) */
double ic_rbf_kernel(const ic_rbf_network_t *rbf, const double *x, size_t ci) {
    if (!rbf || !x || ci >= rbf->num_centers) return 0.0;
    double dist_sq = 0.0;
    for (size_t d = 0; d < rbf->input_dim; d++) {
        double diff = x[d] - rbf->centers[ci * rbf->input_dim + d];
        dist_sq += diff * diff;
    }
    double sigma = rbf->sigmas[ci];
    if (sigma < 1e-10) sigma = 1e-10;
    return exp(-dist_sq / (2.0 * sigma * sigma));
}

double ic_rbf_output(const ic_rbf_network_t *rbf, const double *x) {
    if (!rbf || !x) return 0.0;
    double y = rbf->bias;
    for (size_t i = 0; i < rbf->num_centers; i++) {
        y += rbf->weights[i] * ic_rbf_kernel(rbf, x, i);
    }
    return y;
}

/* LMS training for RBF output weights */
void ic_rbf_train_lms(ic_rbf_network_t *rbf, const double *x, double target) {
    if (!rbf || !x) return;
    double output = ic_rbf_output(rbf, x);
    double error = target - output;
    rbf->bias += rbf->learning_rate * error;
    for (size_t i = 0; i < rbf->num_centers; i++) {
        double phi = ic_rbf_kernel(rbf, x, i);
        rbf->weights[i] += rbf->learning_rate * error * phi;
    }
}

/* ---- L6: K-Means Clustering for RBF Center Initialization ---- */

void ic_rbf_init_centers_kmeans(ic_rbf_network_t *rbf,
                                 const double *data, size_t n_samples) {
    if (!rbf || !data || n_samples < rbf->num_centers) return;
    size_t i, j, d;
    /* Random initialization from data samples */
    for (i = 0; i < rbf->num_centers; i++) {
        size_t idx = (size_t)(ic_random_uniform() * (double)n_samples);
        if (idx >= n_samples) idx = n_samples - 1;
        for (d = 0; d < rbf->input_dim; d++) {
            rbf->centers[i * rbf->input_dim + d] =
                data[idx * rbf->input_dim + d];
        }
    }
    /* Lloyd iteration */
    size_t *assignments = (size_t*)calloc(n_samples, sizeof(size_t));
    size_t *counts = (size_t*)calloc(rbf->num_centers, sizeof(size_t));
    if (!assignments || !counts) { free(assignments); free(counts); return; }
    for (size_t iter = 0; iter < 10; iter++) {
        /* Assign samples to nearest center */
        for (i = 0; i < n_samples; i++) {
            double min_dist = DBL_MAX;
            size_t best = 0;
            for (j = 0; j < rbf->num_centers; j++) {
                double dist = 0.0;
                for (d = 0; d < rbf->input_dim; d++) {
                    double diff = data[i * rbf->input_dim + d] -
                                  rbf->centers[j * rbf->input_dim + d];
                    dist += diff * diff;
                }
                if (dist < min_dist) { min_dist = dist; best = j; }
            }
            assignments[i] = best;
        }
        /* Recompute centers */
        memset(rbf->centers, 0, rbf->num_centers * rbf->input_dim * sizeof(double));
        memset(counts, 0, rbf->num_centers * sizeof(size_t));
        for (i = 0; i < n_samples; i++) {
            size_t c = assignments[i];
            counts[c]++;
            for (d = 0; d < rbf->input_dim; d++) {
                rbf->centers[c * rbf->input_dim + d] +=
                    data[i * rbf->input_dim + d];
            }
        }
        for (j = 0; j < rbf->num_centers; j++) {
            if (counts[j] > 0) {
                for (d = 0; d < rbf->input_dim; d++) {
                    rbf->centers[j * rbf->input_dim + d] /= (double)counts[j];
                }
            }
        }
    }
    /* Set sigmas to average distance to nearest neighbor */
    for (j = 0; j < rbf->num_centers; j++) {
        double min_dist = DBL_MAX;
        for (i = 0; i < rbf->num_centers; i++) {
            if (i == j) continue;
            double dist = 0.0;
            for (d = 0; d < rbf->input_dim; d++) {
                double diff = rbf->centers[j * rbf->input_dim + d] -
                              rbf->centers[i * rbf->input_dim + d];
                dist += diff * diff;
            }
            if (dist < min_dist) min_dist = dist;
        }
        rbf->sigmas[j] = sqrt(min_dist) * 0.5;
        if (rbf->sigmas[j] < 0.01) rbf->sigmas[j] = 0.01;
    }
    free(assignments); free(counts);
}
