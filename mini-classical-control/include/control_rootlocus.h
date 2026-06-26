/**
 * control_rootlocus.h — Root Locus Analysis
 *
 * L5: Root locus computation using Evans rules
 * L6: Root locus design interpretation
 *
 * The root locus shows how closed-loop poles move as a gain K varies
 * from 0 to ∞ in the characteristic equation: 1 + K·G(s)·H(s) = 0.
 *
 * W.R. Evans (1948, 1950) established the graphical rules.
 *
 * Refs: Ogata Ch.7, Franklin Ch.5, Dorf Ch.8.
 */

#ifndef CONTROL_ROOTLOCUS_H
#define CONTROL_ROOTLOCUS_H

#include "control_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Root Locus Data ─── */

/** A single branch of the root locus — K vs pole location */
typedef struct {
    int       num_points;
    double   *K_values;           /**< Gain at each point               */
    double complex *locus_points; /**< Closed-loop pole locations       */
    int       branch_start;       /**< Index of open-loop pole (start)  */
    int       branch_end;         /**< Index of open-loop zero or ∞     */
} rl_branch_t;

/** Complete root locus data */
typedef struct {
    int       num_branches;       /**< = max(num_poles, num_zeros)      */
    int       num_poles;
    int       num_zeros;
    double complex open_poles[CTRL_MAX_ORDER];
    double complex open_zeros[CTRL_MAX_ORDER];
    rl_branch_t *branches;        /**< Array of branches                */
} root_locus_t;

/* ─── L5: Root Locus Computation ─── */

/**
 * Compute root locus branches for G(s)·H(s).
 *
 * Algorithm:
 *   1. Start at each open-loop pole (K=0).
 *   2. Follow the angle condition ∠G(s) = (2k+1)·π.
 *   3. Increment K and solve for pole locations.
 *   4. Terminate branches at open-loop zeros or asymptotes.
 *
 * Evans Rules implemented:
 *   R1: n branches (n = max(#poles, #zeros))
 *   R2: Branches start at poles (K=0), end at zeros (K→∞)
 *   R3: Root locus on real axis to the left of an odd number
 *       of real poles+zeros
 *   R4: Asymptote angles = (2k+1)·π/(#poles-#zeros)
 *   R5: Asymptote centroid = (Σpoles - Σzeros)/(#poles-#zeros)
 *   R6: Breakaway/break-in points from dK/ds = 0
 *   R7: jω-axis crossing from Routh array
 *   R8: Angle of departure from complex poles
 *   R9: Angle of arrival at complex zeros
 */
int root_locus_compute(const transfer_function_t *G,
                       const transfer_function_t *H,
                       double K_max, int points_per_branch,
                       root_locus_t *rl);

/**
 * Free root locus data.
 */
void root_locus_free(root_locus_t *rl);

/* ─── L5: Evans Rules — Individual Computations ─── */

/**
 * Compute asymptote angles for the root locus.
 * θ_k = (2k+1)·π / (n_p - n_z),  k = 0, 1, ..., (n_p-n_z-1)
 * Valid when n_p > n_z.
 */
int rl_asymptote_angles(int n_poles, int n_zeros,
                        double *angles, int *num_angles);

/**
 * Compute asymptote centroid (center of gravity).
 * σ_a = (Σ p_i - Σ z_j) / (n_p - n_z)
 */
double rl_centroid(const double complex *poles, int n_poles,
                   const double complex *zeros, int n_zeros);

/**
 * Find breakaway and break-in points on the real axis.
 *
 * Breakaway/break-in points occur where:
 *   dK/ds = 0,  i.e., where d/ds[1/|G(s)·H(s)|] = 0
 *
 * Equivalently: solve Σ 1/(s-p_i) = Σ 1/(s-z_j)
 * for real s on segments where root locus exists.
 */
int rl_breakaway_points(const transfer_function_t *G,
                        const transfer_function_t *H,
                        double *points, int *num_points);

/**
 * Find jω-axis crossing points.
 * Uses Routh array on the closed-loop characteristic polynomial
 * parameterized by K. Finds K where a row becomes all zeros.
 *
 * Returns the crossing frequencies ω_cross and corresponding gains K_cross.
 */
int rl_jw_crossing(const transfer_function_t *G,
                   const transfer_function_t *H,
                   double *omega_cross, double *K_cross, int *num_cross);

/**
 * Compute angle of departure from a complex pole p_k.
 * θ_dep = π - Σ∠(p_k - z_j) + Σ∠(p_k - p_i) [i≠k]
 *
 * This determines the direction a branch leaves a complex pole.
 */
double rl_departure_angle(int k, const double complex *poles, int n_poles,
                           const double complex *zeros, int n_zeros);

/**
 * Compute angle of arrival at a complex zero z_k.
 * θ_arr = π + Σ∠(z_k - p_i) - Σ∠(z_k - z_j) [j≠k]
 */
double rl_arrival_angle(int k, const double complex *poles, int n_poles,
                          const double complex *zeros, int n_zeros);

/**
 * Compute the gain K at a specific point s on the root locus.
 * K = 1 / |G(s)·H(s)| = |den(s)| / |num(s)|
 *
 * Knowledge: The magnitude condition |K·G(s)·H(s)| = 1 determines K.
 */
double rl_gain_at_point(const transfer_function_t *G,
                        const transfer_function_t *H,
                        double complex s);

/**
 * Determine if a real-axis segment belongs to the root locus.
 * Rule: A point on the real axis is on the root locus iff the number
 * of real poles and zeros to its right is odd.
 */
int rl_on_real_axis(double x, const double complex *poles, int n_poles,
                     const double complex *zeros, int n_zeros);

/**
 * Find the closed-loop poles for a specific gain K.
 * Solves: 1 + K·G(s)·H(s) = 0 for all roots.
 * Uses companion matrix eigenvalue method.
 */
int rl_poles_at_gain(const transfer_function_t *G,
                     const transfer_function_t *H,
                     double K, double complex *cl_poles, int *num_poles);

/**
 * Determine the gain K that yields a specific damping ratio ζ.
 * Searches along the ζ-line in the s-plane: s = -ζω_n ± jω_n√(1-ζ²)
 * Finds intersection with the root locus.
 */
int rl_gain_for_damping(const transfer_function_t *G,
                        const transfer_function_t *H,
                        double zeta_des, double *K_result,
                        double complex *pole_result);

/**
 * Root sensitivity: ∂s/∂K at a given pole location.
 * Measures how sensitive a pole is to gain variations.
 *
 * Formula: ∂s/∂K = -G(s)·H(s) / [K·d/ds(G(s)·H(s))]
 */
double complex rl_sensitivity(const transfer_function_t *G,
                               const transfer_function_t *H,
                               double K, double complex s);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ROOTLOCUS_H */
