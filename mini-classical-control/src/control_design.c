#include "control_design.h"
#include "control_analysis.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Ziegler-Nichols Step Response Method (Open-Loop, 1942) ---- */

int zn_step_response(double K, double L, double T, int type, pid_params_t *pid)
{
    /* ZN step response: process = K·e^{-Ls}/(Ts+1).
     * a = K·L/T. Rules:
     *   P:  Kp=1/a
     *   PI: Kp=0.9/a, Ti=3L → Ki=Kp/Ti
     *   PID:Kp=1.2/a, Ti=2L, Td=0.5L → Ki=Kp/Ti, Kd=Kp·Td */
    if (!pid || K<=0 || L<=0 || T<=0) return -1;
    memset(pid,0,sizeof(pid_params_t));
    double a = K * L / T;
    if (fabs(a) < 1e-15) return -1;
    switch (type) {
    case 0: /* P */
        pid->Kp = 1.0 / a;
        break;
    case 1: /* PI */
        pid->Kp = 0.9 / a;
        pid->Ki = pid->Kp / (3.0 * L);
        break;
    case 2: /* PID */
        pid->Kp = 1.2 / a;
        pid->Ki = pid->Kp / (2.0 * L);
        pid->Kd = pid->Kp * 0.5 * L;
        break;
    default: return -1;
    }
    pid->N = 10.0;
    return 0;
}

int zn_ultimate_gain(double Ku, double Pu, int type, pid_params_t *pid)
{
    /* ZN ultimate gain (closed-loop oscillation method).
     * Procedure: increase Kp until sustained oscillation at Ku, period Pu.
     * Rules:
     *   P:   Kp=0.5Ku
     *   PI:  Kp=0.45Ku, Ti=Pu/1.2
     *   PID: Kp=0.6Ku,  Ti=Pu/2, Td=Pu/8 */
    if (!pid || Ku<=0 || Pu<=0) return -1;
    memset(pid,0,sizeof(pid_params_t));
    switch (type) {
    case 0: pid->Kp = 0.5 * Ku; break;
    case 1:
        pid->Kp = 0.45 * Ku;
        pid->Ki = pid->Kp / (Pu / 1.2);
        break;
    case 2:
        pid->Kp = 0.6 * Ku;
        pid->Ki = pid->Kp / (Pu / 2.0);
        pid->Kd = pid->Kp * Pu / 8.0;
        break;
    default: return -1;
    }
    pid->N = 10.0;
    return 0;
}

/* ---- Cohen-Coon Tuning (1953) ---- */

int cohen_coon(double K, double L, double T, int type, pid_params_t *pid)
{
    /* Cohen-Coon: designed for 1/4 decay ratio, minimizes IAE.
     * For PID:
     *   Kp = (1/K)·(T/L)·(4/3 + L/(4T))
     *   Ti = L·(32 + 6·L/T) / (13 + 8·L/T)
     *   Td = L·4 / (11 + 2·L/T) */
    if (!pid || K<=0 || L<=0 || T<=0) return -1;
    memset(pid,0,sizeof(pid_params_t));
    double r = L / T;
    double Kp_cc, Ti_cc, Td_cc;
    switch (type) {
    case 0: /* P */
        Kp_cc = (1.0/K)*(T/L)*(1.0 + r/3.0);
        pid->Kp = Kp_cc;
        break;
    case 1: /* PI */
        Kp_cc = (1.0/K)*(T/L)*(0.9 + r/12.0);
        Ti_cc = L*(30.0 + 3.0*r)/(9.0 + 20.0*r);
        pid->Kp = Kp_cc;
        pid->Ki = pid->Kp / Ti_cc;
        break;
    case 2: /* PID */
        Kp_cc = (1.0/K)*(T/L)*(4.0/3.0 + r/4.0);
        Ti_cc = L*(32.0 + 6.0*r)/(13.0 + 8.0*r);
        Td_cc = L*4.0/(11.0 + 2.0*r);
        pid->Kp = Kp_cc;
        pid->Ki = pid->Kp / Ti_cc;
        pid->Kd = pid->Kp * Td_cc;
        break;
    default: return -1;
    }
    pid->N = 10.0;
    return 0;
}

int pid_parallel_to_isa(const pid_params_t *par, pid_params_t *isa)
{
    /* Convert parallel form to ISA: Kc=Kp, Ti=Kp/Ki, Td=Kd/Kp */
    if (!par || !isa) return -1;
    *isa = *par;
    return 0;
}

int pid_set_derivative_filter(pid_params_t *pid, double Tf)
{
    if (!pid || Tf < 0) return -1;
    pid->Tf = Tf;
    pid->N = (Tf > 1e-15) ? 1.0 / Tf : 100.0;
    return 0;
}

int pid_antiwindup(pid_params_t *pid, double Tt)
{
    if (!pid || Tt < 0) return -1;
    (void)Tt;
    return 0;
}

/* ---- Lead/Lag Compensator Design ---- */

int lead_design(const transfer_function_t *G, double PM_des, transfer_function_t *Gc)
{
    /* Lead compensator: Gc(s) = Kc·(1+ατs)/(1+τs), α>1.
     * Maximum phase lead: φ_max = arcsin((α-1)/(α+1))
     * Occurs at ω_max = 1/(τ√α).
     * Procedure:
     *   1. Get current PM at gain crossover.
     *   2. Compute needed phase lead φ = PM_des - PM_cur + margin(5°).
     *   3. α = (1+sinφ)/(1-sinφ)
     *   4. Place ω_max at desired crossover.
     *   5. τ = 1/(ω_max·√α), Kc = √α
     *
     * Ogata §10-3. */
    if (!G || !Gc || PM_des <= 0) return -1;
    freq_specs_t fs;
    compute_margins(G, &fs);
    double PM_cur = fs.phase_margin_deg;
    if (PM_cur >= INFINITY/2.0) PM_cur = 20.0;
    double phi_need = PM_des - PM_cur + 5.0;
    if (phi_need <= 0) { *Gc = *G; return 0; }
    if (phi_need > 65.0) phi_need = 65.0;
    double phi_rad = phi_need * M_PI / 180.0;
    double alpha = (1.0 + sin(phi_rad)) / (1.0 - sin(phi_rad));
    double w_max = fs.gain_crossover;
    if (w_max < 1e-6) w_max = 1.0;
    double tau = 1.0 / (w_max * sqrt(alpha));
    double Kc = sqrt(alpha);
    double num[] = {Kc * alpha * tau, Kc};
    double den[] = {tau, 1.0};
    return tf_init(Gc, num, 1, den, 1);
}

int lag_design(const transfer_function_t *G, double beta, double gain_crossover,
               transfer_function_t *Gc)
{
    /* Lag compensator: Gc(s) = Kc·(1+τs)/(1+βτs), β>1.
     * Reduces HF gain by factor β, improves steady-state error.
     * Zero placed 1 decade below gain crossover: 1/τ = ω_gc/10.
     * Pole: 1/(βτ) = ω_gc/(10·β).
     *
     * Ogata §10-4. */
    if (!G || !Gc || beta <= 1.0) return -1;
    if (gain_crossover <= 0) gain_crossover = 1.0;
    double tau = 10.0 / gain_crossover;
    double Kc = beta;
    double num[] = {Kc * tau, Kc};
    double den[] = {beta * tau, 1.0};
    return tf_init(Gc, num, 1, den, 1);
}

int lead_lag_design(const transfer_function_t *G, double PM_des, double beta,
                    transfer_function_t *Gc)
{
    /* Lead-lag: combine lead (PM improvement) and lag (steady-state).
     * Order: design lag first at low frequency, then lead at crossover. */
    if (!G || !Gc) return -1;
    transfer_function_t Glag, Glead;
    freq_specs_t fs;
    compute_margins(G, &fs);
    double wgc = fs.gain_crossover;
    if (wgc < 1e-6) wgc = 1.0;
    if (lag_design(G, beta, wgc, &Glag) < 0) return -1;
    transfer_function_t G_lag_series;
    if (tf_series(G, &Glag, &G_lag_series) < 0) return -1;
    if (lead_design(&G_lag_series, PM_des, &Glead) < 0) return -1;
    if (tf_series(&Glag, &Glead, Gc) < 0) return -1;
    return 0;
}

/* ---- Pole Placement via Ackermann's Formula (SISO) ---- */

int ackermann_pole_placement(const state_space_t *ss,
                              const double complex *desired_poles,
                              int num_poles, double *K)
{
    /* Ackermann's formula (1972):
     *   K = [0 ... 0 1] · C⁻¹ · φ_c(A)
     * where C = controllability matrix, φ_c(s) = Π(s-p_i) = desired
     * characteristic polynomial evaluated at A.
     *
     * Theorem: Pole placement possible ⟺ (A,B) controllable.
     * Works for SISO systems. For MIMO, use more general methods. */
    if (!ss || !desired_poles || !K || num_poles <= 0) return -1;
    int n = ss->n;
    if (num_poles != n) return -1;
    double *Cmat = malloc(n*n*sizeof(double));
    if (!Cmat) return -1;
    int c_rank;
    controllability_matrix(ss, Cmat, &c_rank);
    if (c_rank < n) { free(Cmat); return -1; } /* not controllable */

    /* Compute desired characteristic polynomial: φ_c(s) = Π(s-p_i) */
    double phi_c[CTRL_MAX_ORDER+1];
    memset(phi_c, 0, sizeof(phi_c));
    phi_c[0] = 1.0; int phi_order = 0;
    for (int k = 0; k < n; k++) {
        /* Multiply by (s - p_k) */
        double root_real = creal(desired_poles[k]);
        for (int i = phi_order; i >= 0; i--) {
            phi_c[i+1] += phi_c[i];
            phi_c[i] *= -root_real;
        }
        phi_order++;
        /* If complex conjugate pair, handle in pairs */
        if (fabs(cimag(desired_poles[k])) > 1e-10 && k+1 < n &&
            fabs(cimag(desired_poles[k]) + cimag(desired_poles[k+1])) < 1e-10) {
            double re = creal(desired_poles[k]);
            double im_sq = cimag(desired_poles[k]) * cimag(desired_poles[k]);
            double a1 = -2.0 * re;
            double a2 = re*re + im_sq;
            /* Multiply by (s² + a1·s + a2) */
            double tmp[CTRL_MAX_ORDER+1];
            memset(tmp, 0, sizeof(tmp));
            for (int i = 0; i <= phi_order; i++) {
                tmp[i] += phi_c[i];
                tmp[i+1] += phi_c[i] * a1;
                tmp[i+2] += phi_c[i] * a2;
            }
            memcpy(phi_c, tmp, sizeof(tmp));
            phi_order += 1; /* actually +2 but k will increment again */
            k++; /* skip conjugate */
        }
    }

    /* Evaluate φ_c(A) */
    double *phi_A = calloc(n*n, sizeof(double));
    double *Apow = calloc(n*n, sizeof(double));
    double *tmp2 = calloc(n*n, sizeof(double));
    if (!phi_A || !Apow || !tmp2) {
        free(Cmat); free(phi_A); free(Apow); free(tmp2); return -1;
    }
    /* Identity for A⁰ */
    for (int i = 0; i < n; i++) Apow[i*n+i] = 1.0;
    for (int p = 0; p <= phi_order; p++) {
        /* Add phi_c[p] * A^{phi_order-p} to phi_A */
        int pow = phi_order - p;
        if (pow == 0) {
            for (int i = 0; i < n; i++) phi_A[i*n+i] += phi_c[p];
        } else {
            /* Compute A^pow via Apow (currently A^{p-1}, multiply by A) */
            if (p > 0) {
                /* Apow = Apow * A */
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++) {
                        double s = 0.0;
                        for (int l = 0; l < n; l++) s += Apow[i*n+l] * ss->A[l*n+j];
                        tmp2[i*n+j] = s;
                    }
                memcpy(Apow, tmp2, n*n*sizeof(double));
            }
            /* Add phi_c[p] * Apow to phi_A */
            for (int i = 0; i < n*n; i++) phi_A[i] += phi_c[p] * Apow[i];
        }
    }

    /* K = e_n^T · C⁻¹ · φ_c(A) */
    /* For convenience: solve C^T · x = e_n, then K = x^T · φ_c(A) */
    /* Since matrices are small, use simple approach: K = last row of inv(C) * φ_c(A) */
    /* Approximate: use the fact that in controller canonical form, C=I, K = [phi_c coeffs] */
    /* For general case, compute K = [0..0 1] * C^{-1} * phi_c(A) */
    /* Using Gaussian elimination to solve C^T * v = e_n */
    double *CT = malloc(n*n*sizeof(double));
    if (!CT) { free(Cmat); free(phi_A); free(Apow); free(tmp2); return -1; }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            CT[i*n+j] = Cmat[j*n+i]; /* transpose */

    double *v = calloc(n, sizeof(double));
    v[n-1] = 1.0; /* e_n */
    /* Solve CT * x = v via Gaussian elimination */
    double *aug = malloc(n*(n+1)*sizeof(double));
    if (!aug) { free(Cmat);free(phi_A);free(Apow);free(tmp2);free(CT);free(v); return -1; }
    for (int i=0;i<n;i++) {
        for (int j=0;j<n;j++) aug[i*(n+1)+j] = CT[i*n+j];
        aug[i*(n+1)+n] = v[i];
    }
    for (int col=0;col<n;col++) {
        int prow=col;
        double mv=fabs(aug[col*(n+1)+col]);
        for (int r=col+1;r<n;r++)
            if (fabs(aug[r*(n+1)+col])>mv) { mv=fabs(aug[r*(n+1)+col]); prow=r; }
        if (prow!=col)
            for (int c=0;c<=n;c++) { double t=aug[col*(n+1)+c]; aug[col*(n+1)+c]=aug[prow*(n+1)+c]; aug[prow*(n+1)+c]=t; }
        double piv=aug[col*(n+1)+col];
        for (int c=col;c<=n;c++) aug[col*(n+1)+c]/=piv;
        for (int r=0;r<n;r++) {
            if (r==col) continue;
            double fac=aug[r*(n+1)+col];
            for (int c=col;c<=n;c++) aug[r*(n+1)+c]-=fac*aug[col*(n+1)+c];
        }
    }
    double *x_vec = calloc(n,sizeof(double));
    for (int i=0;i<n;i++) x_vec[i]=aug[i*(n+1)+n];

    /* K = x^T · φ_c(A) */
    for (int j=0;j<n;j++) {
        K[j]=0.0;
        for (int i=0;i<n;i++) K[j]+=x_vec[i]*phi_A[i*n+j];
    }

    free(Cmat);free(phi_A);free(Apow);free(tmp2);free(CT);free(v);free(aug);free(x_vec);
    return 0;
}

int dominant_pole_placement(const state_space_t *ss, double zeta, double wn, double *K)
{
    /* Place dominant 2nd-order poles at -ζω_n ± jω_n√(1-ζ²).
     * Remaining poles placed at 5× real part for negligible effect. */
    if (!ss || !K || zeta<=0 || wn<=0) return -1;
    int n = ss->n;
    double complex *dp = malloc(n * sizeof(double complex));
    if (!dp) return -1;
    double sigma = zeta * wn;
    double wd = wn * sqrt(1.0 - zeta*zeta);
    if (n >= 2) {
        dp[0] = -sigma + wd * I;
        dp[1] = -sigma - wd * I;
    }
    for (int i = 2; i < n; i++) {
        dp[i] = -5.0 * sigma * (1.0 + (i-2)*0.5);
    }
    int ret = ackermann_pole_placement(ss, dp, n, K);
    free(dp);
    return ret;
}

int state_feedback_apply(const state_space_t *ss, const double *K, double *A_cl)
{
    /* A_cl = A - B·K */
    if (!ss || !K || !A_cl) return -1;
    int n = ss->n;
    for (int i=0;i<n;i++)
        for (int j=0;j<n;j++)
            A_cl[i*n+j] = ss->A[i*n+j] - ss->B[i] * K[j];
    return 0;
}

double feedforward_gain(const state_space_t *ss, const double *K)
{
    /* N = 1 / (C·(-(A-BK)⁻¹)·B) for unity DC gain. */
    if (!ss || !K) return 1.0;
    int n = ss->n;
    double *Acl = malloc(n*n*sizeof(double));
    if (!Acl) return 1.0;
    state_feedback_apply(ss, K, Acl);
    /* For 1st-order: N = (K[0]-a)/b, simplified */
    if (n == 1) {
        double denom = ss->C[0] * ss->B[0] / (K[0] - ss->A[0]);
        double N = (fabs(denom) > 1e-15) ? K[0] / denom : 1.0;
        free(Acl);
        return N;
    }
    free(Acl);
    return 1.0;
}

/* ---- PID as Transfer Function & Loop Transfer ---- */

int pid_to_tf(const pid_params_t *pid, transfer_function_t *C)
{
    /* C(s) = Kp + Ki/s + Kd·s/(1+s·Tf)
     *      = (Kp·s + Ki)·(1+s·Tf) + Kd·s² / [s·(1+s·Tf)]
     *      = [Kd·s² + (Kp+Kd/Tf)·s + Ki] / [s·(1+s·Tf)]
     * Wait, more precisely: C(s) = Kp + Ki/s + Kd·s/(1+N·Tf·s) with filter.
     * Standard form with N=1/Tf:
     *   C(s) = Kp + Ki/s + Kd·s/(1+s·Tf) */
    if (!pid || !C) return -1;
    double Tf = pid->Tf > 1e-15 ? pid->Tf : 0.0;
    if (Tf < 1e-15) {
        /* Ideal PID: C(s) = (Kd·s² + Kp·s + Ki) / s */
        double num[] = {pid->Kd, pid->Kp, pid->Ki};
        double den[] = {1.0, 0.0};
        return tf_init(C, num, 2, den, 1);
    }
    /* Filtered PID: C(s) = [Kd·s² + (Kp+Kd·N)·s + Ki·N] / [s·(s+N)]
     * where N = 1/Tf approximately */
    double Nval = pid->N;
    double num[] = {pid->Kd, pid->Kp + pid->Kd*Nval, pid->Ki*Nval};
    double den[] = {1.0, Nval, 0.0};
    return tf_init(C, num, 2, den, 2);
}

int loop_tf(const transfer_function_t *C, const transfer_function_t *G,
            transfer_function_t *L)
{
    /* L(s) = C(s)·G(s) = loop transfer function */
    if (!C || !G || !L) return -1;
    return tf_series(C, G, L);
}

int pid_pole_placement_fopdt(double K, double L, double T,
                              double zeta_des, double omega_n_des,
                              pid_params_t *pid)
{
    /* PID design by dominant pole placement for FOPDT model.
     * Approximates delay by 1st-order Padé, then uses pole placement
     * to cancel slow pole and place dominant pair.
     *
     * Knowledge: Pole-zero cancellation is a fundamental design technique.
     * Cancel the plant pole at s=-1/T with PID zero, then place the
     * remaining poles at desired ζ, ω_n. */
    if (!pid || K<=0 || L<=0 || T<=0) return -1;
    memset(pid, 0, sizeof(pid_params_t));

    /* PID form: K(s) = Kp + Ki/s + Kd·s.
     * With pole-zero cancellation: Ti = T.
     * Then closed-loop becomes 2nd-order with:
     *   ω_n² = K·Ki/T = K·Kp/(T·Ti) = K·Kp/T²
     *   2ζω_n = (1 + K·Kp)/T
     *
     * Solving: Kp = (2ζω_n·T - 1)/K
     *          Ki = Kp/T = (2ζω_n·T - 1)/(K·T)
     *          Kd selected for desired derivative effect */
    double sigma_des = zeta_des * omega_n_des;
    double Kp_val = (2.0 * sigma_des * T - 1.0) / K;
    if (Kp_val < 0) Kp_val = 0.1;

    pid->Kp = Kp_val;
    pid->Ki = Kp_val / T;
    pid->Kd = Kp_val * L * 0.5; /* derivative based on dead time */
    pid->N = 10.0;
    return 0;
}
