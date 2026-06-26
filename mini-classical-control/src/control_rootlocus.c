#include "control_rootlocus.h"
#include "control_analysis.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Evans Root Locus Rules (1948, 1950) ---- */

int rl_asymptote_angles(int n_poles, int n_zeros, double *angles, int *num_angles)
{
    /* Asymptote angles: θ_k = (2k+1)·π/(n_p - n_z), k=0,1,...,(n_p-n_z-1).
     * Valid only when n_p > n_z (strictly proper plant). */
    if (n_poles <= n_zeros) { *num_angles = 0; return 0; }
    int n_a = n_poles - n_zeros;
    *num_angles = n_a;
    for (int k = 0; k < n_a; k++)
        angles[k] = (2.0 * k + 1.0) * M_PI / n_a;
    return 0;
}

double rl_centroid(const double complex *poles, int n_poles,
                   const double complex *zeros, int n_zeros)
{
    /* Centroid: σ_a = (Σp_i - Σz_j) / (n_p - n_z).
     * Center of gravity of asymptotes. */
    if (n_poles <= n_zeros) return 0.0;
    double complex sum_p = 0.0, sum_z = 0.0;
    for (int i = 0; i < n_poles; i++) sum_p += poles[i];
    for (int j = 0; j < n_zeros; j++) sum_z += zeros[j];
    return creal(sum_p - sum_z) / (n_poles - n_zeros);
}

int rl_on_real_axis(double x, const double complex *poles, int n_poles,
                     const double complex *zeros, int n_zeros)
{
    /* A point x on the real axis is on the root locus iff
     * the number of real poles+zeros to the RIGHT of x is ODD. */
    int count = 0;
    for (int i = 0; i < n_poles; i++)
        if (fabs(cimag(poles[i])) < 1e-10 && creal(poles[i]) > x) count++;
    for (int j = 0; j < n_zeros; j++)
        if (fabs(cimag(zeros[j])) < 1e-10 && creal(zeros[j]) > x) count++;
    return (count % 2 == 1) ? 1 : 0;
}

double rl_departure_angle(int k, const double complex *poles, int n_poles,
                           const double complex *zeros, int n_zeros)
{
    /* Angle of departure from complex pole p_k:
     *   θ_dep = π - Σ∠(p_k - z_j) + Σ_{i≠k}∠(p_k - p_i)
     *
     * Knowledge: Departure angle determines which direction a branch
     * takes when leaving a complex pole. Important for understanding
     * how gain affects closed-loop damping. */
    if (!poles || k < 0 || k >= n_poles) return 0.0;
    double complex pk = poles[k];
    double angle = M_PI; /* 180° base */
    for (int j = 0; j < n_zeros; j++)
        angle -= carg(pk - zeros[j]);
    for (int i = 0; i < n_poles; i++)
        if (i != k) angle += carg(pk - poles[i]);
    return angle;
}

double rl_arrival_angle(int k, const double complex *poles, int n_poles,
                          const double complex *zeros, int n_zeros)
{
    /* Angle of arrival at complex zero z_k:
     *   θ_arr = π + Σ∠(z_k - p_i) - Σ_{j≠k}∠(z_k - z_j) */
    if (!zeros || k < 0 || k >= n_zeros) return 0.0;
    double complex zk = zeros[k];
    double angle = M_PI;
    for (int i = 0; i < n_poles; i++)
        angle += carg(zk - poles[i]);
    for (int j = 0; j < n_zeros; j++)
        if (j != k) angle -= carg(zk - zeros[j]);
    return angle;
}

double rl_gain_at_point(const transfer_function_t *G, const transfer_function_t *H,
                        double complex s)
{
    /* K = 1/|G(s)·H(s)| = |den(s)|/|num(s)|.
     * Magnitude condition: |K·G(s)·H(s)| = 1. */
    if (!G) return 0.0;
    double complex Gs = tf_evaluate(G, s);
    if (H) {
        double complex Hs = tf_evaluate(H, s);
        Gs *= Hs;
    }
    double mag = cabs(Gs);
    return (mag > 1e-15) ? 1.0 / mag : INFINITY;
}

double complex rl_sensitivity(const transfer_function_t *G,
                               const transfer_function_t *H,
                               double K, double complex s)
{
    /* Root sensitivity: ∂s/∂K.
     * Formula: ∂s/∂K = -G(s)H(s) / [K·d/ds(G(s)H(s))]
     * For small ΔK, Δs ≈ (∂s/∂K)·ΔK.
     * Knowledge: Root sensitivity measures robustness — how much
     * closed-loop poles move with gain variations. */
    if (!G || fabs(K) < 1e-15) return 0.0;
    /* Approximate derivative d(GH)/ds via finite difference */
    double complex ds = 1e-6;
    double complex Gs = tf_evaluate(G, s);
    if (H) Gs *= tf_evaluate(H, s);
    double complex Gs_p = tf_evaluate(G, s + ds);
    if (H) Gs_p *= tf_evaluate(H, s + ds);
    double complex dGH = (Gs_p - Gs) / ds;
    if (cabs(dGH) < 1e-15) return 0.0;
    return -Gs / (K * dGH);
}

/* ---- Breakaway/Break-in Points ---- */

int rl_breakaway_points(const transfer_function_t *G, const transfer_function_t *H,
                        double *points, int *num_points)
{
    /* Breakaway/break-in satisfy: Σ1/(s-p_i) = Σ1/(s-z_j).
     * Equivalently: dK/ds = 0 where K(s) = -1/(G(s)·H(s)).
     * Search along real-axis segments where RL exists. */
    if (!G || !points || !num_points) return -1;
    *num_points = 0;
    pole_zero_t pz_p, pz_z;
    tf_find_poles(G, &pz_p);
    /* For loop TF, zeros are from G·H */
    transfer_function_t GH;
    if (H) {
        tf_series(G, H, &GH);
        tf_find_zeros(&GH, &pz_z);
    } else {
        tf_find_zeros(G, &pz_z);
    }
    /* Collect all real poles and zeros, find segments with odd count to right */
    double reals[CTRL_MAX_ORDER*2];
    int n_reals = 0;
    for (int i=0;i<pz_p.num_poles;i++)
        if (fabs(cimag(pz_p.poles[i]))<1e-10) reals[n_reals++] = creal(pz_p.poles[i]);
    for (int j=0;j<pz_z.num_zeros;j++)
        if (fabs(cimag(pz_z.zeros[j]))<1e-10) reals[n_reals++] = creal(pz_z.zeros[j]);
    /* Sort */
    for (int i=0;i<n_reals;i++)
        for (int j=i+1;j<n_reals;j++)
            if (reals[i]>reals[j]) { double t=reals[i]; reals[i]=reals[j]; reals[j]=t; }
    /* For each segment between consecutive real points, check if RL exists */
    for (int i=0;i<n_reals-1;i++) {
        double x_mid = (reals[i]+reals[i+1])/2.0;
        if (rl_on_real_axis(x_mid, pz_p.poles, pz_p.num_poles, pz_z.zeros, pz_z.num_zeros)) {
            /* Solve Σ1/(s-p_i)=Σ1/(s-z_j) on this segment via bisection */
            /* Define f(x) = Σ1/(x-p_i) - Σ1/(x-z_j) */
            double xl=reals[i]+1e-6, xr=reals[i+1]-1e-6;
            double fl=0, fr=0;
            for (int k=0;k<pz_p.num_poles;k++) {
                double d=xl-creal(pz_p.poles[k]);
                if (fabs(d)>1e-10) fl+=1.0/d;
            }
            for (int k=0;k<pz_z.num_zeros;k++) {
                double d=xl-creal(pz_z.zeros[k]);
                if (fabs(d)>1e-10) fl-=1.0/d;
            }
            for (int k=0;k<pz_p.num_poles;k++) {
                double d=xr-creal(pz_p.poles[k]);
                if (fabs(d)>1e-10) fr+=1.0/d;
            }
            for (int k=0;k<pz_z.num_zeros;k++) {
                double d=xr-creal(pz_z.zeros[k]);
                if (fabs(d)>1e-10) fr-=1.0/d;
            }
            if (fl*fr<0) {
                for (int iter=0;iter<50;iter++) {
                    double xm=(xl+xr)/2.0, fm=0;
                    for (int k=0;k<pz_p.num_poles;k++) {
                        double d=xm-creal(pz_p.poles[k]);
                        if (fabs(d)>1e-10) fm+=1.0/d;
                    }
                    for (int k=0;k<pz_z.num_zeros;k++) {
                        double d=xm-creal(pz_z.zeros[k]);
                        if (fabs(d)>1e-10) fm-=1.0/d;
                    }
                    if (fl*fm<0) { xr=xm; fr=fm; }
                    else { xl=xm; fl=fm; }
                }
                if (*num_points<CTRL_MAX_ORDER) points[(*num_points)++]=(xl+xr)/2.0;
            }
        }
    }
    return 0;
}

/* ---- jω-axis Crossing ---- */

int rl_jw_crossing(const transfer_function_t *G, const transfer_function_t *H,
                   double *omega_cross, double *K_cross, int *num_cross)
{
    /* Find jω-axis crossing using Routh array parameterized by K.
     * For 1+K·G(s)·H(s)=0, form Routh array and find K where a row
     * becomes all zeros. Then solve auxiliary polynomial for ω. */
    if (!G||!omega_cross||!K_cross||!num_cross) return -1;
    *num_cross=0;
    transfer_function_t L;
    if (H) tf_series(G,H,&L); else L=*G;
    double Kcrit = routh_critical_gain(&L);
    if (Kcrit<=0||Kcrit>=1e6) return 0;
    /* Find ω where angle condition is satisfied at this K */
    /* For simplicity, use the Routh auxiliary polynomial:
       For 3rd order: a0s³+a1s²+a2s+a3+K·b(s)=0.
       Auxiliary eqn from row before zero row: gives ω. */
    int n=L.den_order;
    /* For order 3: auxiliary is usually a1·s² + (a3+K·b3) = 0 → ω=√((a3+Kb3)/a1) */
    if (n==3) {
        /* Form characteristic: den+Kn=0 */
        double apk[CTRL_MAX_ORDER+1], kn[CTRL_MAX_ORDER+1]; int oa;
        for (int i=0;i<=L.num_order;i++) kn[i]=Kcrit*L.num[i];
        poly_add(L.den, L.den_order, kn, L.num_order, apk, &oa);
        /* Auxiliary: row s²: a1·s² + a3' = 0 → ω = √(-a3'/a1) */
        /* a1 = apk[1], a3' = apk[3] */
        double ratio = -apk[3]/apk[1];
        if (ratio>0) {
            omega_cross[0]=sqrt(ratio);
            K_cross[0]=Kcrit;
            *num_cross=1;
        }
    } else if (n>=2) {
        /* For higher order, use angle condition search */
        /* Binary search for ω where phase = -180° at K=Kcrit */
        double wl=1e-6, wh=1e6;
        for (int iter=0;iter<60;iter++) {
            double wm=sqrt(wl*wh);
            double mag,phase;
            transfer_function_t tmp; tmp=L; tf_scale(&tmp,Kcrit);
            tf_freq_response(&tmp,wm,&mag,&phase);
            if (phase<-180.0) wh=wm; else wl=wm;
            if (wh-wl<1e-6) break;
        }
        omega_cross[0]=(wl+wh)/2.0;
        K_cross[0]=Kcrit;
        *num_cross=1;
    }
    return 0;
}

/* ---- Poles at Specific Gain ---- */

int rl_poles_at_gain(const transfer_function_t *G, const transfer_function_t *H,
                     double K, double complex *cl_poles, int *num_poles)
{
    if (!G || !cl_poles || !num_poles) return -1;
    transfer_function_t L;
    if (H) { if (tf_series(G, H, &L) < 0) return -1; }
    else L = *G;
    double Knum[CTRL_MAX_ORDER+1], poly[CTRL_MAX_ORDER+1]; int op;
    for (int i=0;i<=L.num_order;i++) Knum[i]=K*L.num[i];
    poly_add(L.den, L.den_order, Knum, L.num_order, poly, &op);
    int n = op;
    if (n <= 0) { *num_poles = 0; return 0; }
    double *C = calloc(n*n, sizeof(double));
    if (!C) return -1;
    for (int i=0;i<n;i++) {
        if (i<n-1) C[(i+1)*n+i] = 1.0;
        C[i] = -poly[i+1]/poly[0];
    }
    for (int iter=0;iter<100;iter++) {
        int changed=0;
        for (int i=0;i<n-1;i++) {
            double a11=C[i*n+i],a12=C[i*n+i+1],a21=C[(i+1)*n+i],a22=C[(i+1)*n+i+1];
            double tr=a11+a22,det=a11*a22-a12*a21, disc=tr*tr-4.0*det, mu;
            if (disc>=0) {
                double r1=(tr+sqrt(disc))/2.0, r2=(tr-sqrt(disc))/2.0;
                mu=(fabs(r1-a22)<fabs(r2-a22))?r1:r2;
            } else mu=tr/2.0;
            double x=a11-mu,y=a21,r=hypot(x,y);
            if (r<1e-15) continue;
            double cs=x/r,sn=y/r;
            for (int col=0;col<n;col++) {
                double t1=C[col*n+i],t2=C[col*n+i+1];
                C[col*n+i]=cs*t1+sn*t2; C[col*n+i+1]=-sn*t1+cs*t2;
            }
            for (int row=0;row<n;row++) {
                double t1=C[i*n+row],t2=C[(i+1)*n+row];
                C[i*n+row]=cs*t1+sn*t2; C[(i+1)*n+row]=-sn*t1+cs*t2;
            }
            changed=1;
        }
        if (!changed) break;
    }
    for (int i=0;i<n;i++) {
        if (i<n-1 && fabs(C[(i+1)*n+i])>1e-10) {
            double a=C[i*n+i],b=C[i*n+i+1],c=C[(i+1)*n+i],d=C[(i+1)*n+i+1];
            double tr=a+d,dt=a*d-b*c,disc2=tr*tr-4.0*dt;
            if (disc2<0) {
                double re=tr/2,im=sqrt(-disc2)/2;
                cl_poles[i]=re+im*I; cl_poles[i+1]=re-im*I;
            } else {
                double sd=sqrt(disc2);
                cl_poles[i]=(tr+sd)/2; cl_poles[i+1]=(tr-sd)/2;
            }
            i++;
        } else cl_poles[i]=C[i*n+i];
    }
    *num_poles = n;
    free(C);
    return 0;
}

/* ---- Gain for Specific Damping Ratio ---- */

int rl_gain_for_damping(const transfer_function_t *G, const transfer_function_t *H,
                        double zeta_des, double *K_result, double complex *pole_result)
{
    if (!G || !K_result || !pole_result || zeta_des<=0 || zeta_des>=1) return -1;
    double r_low=1e-6, r_high=1e6;
    double complex sh = -r_high*zeta_des + r_high*sqrt(1-zeta_des*zeta_des)*I;
    double complex Gs = tf_evaluate(G, sh); if (H) Gs *= tf_evaluate(H, sh);
    double ah = carg(Gs);
    double complex sl = -r_low*zeta_des + r_low*sqrt(1-zeta_des*zeta_des)*I;
    Gs = tf_evaluate(G, sl); if (H) Gs *= tf_evaluate(H, sl);
    double al = carg(Gs);
    double target = M_PI;
    while (al>target+M_PI) al-=2*M_PI;
    while (al<target-M_PI) al+=2*M_PI;
    while (ah>target+M_PI) ah-=2*M_PI;
    while (ah<target-M_PI) ah+=2*M_PI;
    if ((al-target)*(ah-target)>0) return -1;
    for (int iter=0;iter<60;iter++) {
        double rm=sqrt(r_low*r_high);
        double complex sm=-rm*zeta_des+rm*sqrt(1-zeta_des*zeta_des)*I;
        Gs=tf_evaluate(G,sm); if (H) Gs*=tf_evaluate(H,sm);
        double am=carg(Gs);
        while (am>target+M_PI) am-=2*M_PI;
        while (am<target-M_PI) am+=2*M_PI;
        if ((al-target)*(am-target)<0) { r_high=rm; ah=am; }
        else { r_low=rm; al=am; }
        if (r_high-r_low<1e-8) break;
    }
    double rr=(r_low+r_high)/2.0;
    *pole_result=-rr*zeta_des+rr*sqrt(1-zeta_des*zeta_des)*I;
    *K_result=rl_gain_at_point(G,H,*pole_result);
    return 0;
}

/* ---- Root Locus Data Management ---- */

void root_locus_free(root_locus_t *rl)
{
    if (!rl) return;
    if (rl->branches) {
        for (int i=0;i<rl->num_branches;i++) {
            free(rl->branches[i].K_values);
            free(rl->branches[i].locus_points);
        }
        free(rl->branches);
    }
    memset(rl,0,sizeof(root_locus_t));
}

int root_locus_compute(const transfer_function_t *G, const transfer_function_t *H,
                       double K_max, int pts_per_branch, root_locus_t *rl)
{
    if (!G || !rl || pts_per_branch < 2) return -1;
    memset(rl,0,sizeof(root_locus_t));
    transfer_function_t L;
    if (H) tf_series(G,H,&L); else L=*G;
    pole_zero_t pz;
    tf_find_poles(&L,&pz);
    rl->num_poles=pz.num_poles;
    memcpy(rl->open_poles,pz.poles,pz.num_poles*sizeof(double complex));
    tf_find_zeros(&L,&pz);
    rl->num_zeros=pz.num_zeros;
    memcpy(rl->open_zeros,pz.zeros,pz.num_zeros*sizeof(double complex));
    int nb = (rl->num_poles > rl->num_zeros) ? rl->num_poles : rl->num_zeros;
    rl->num_branches=nb;
    rl->branches=calloc(nb,sizeof(rl_branch_t));
    if (!rl->branches) return -1;
    double K_step=K_max/(pts_per_branch-1);
    for (int b=0;b<nb;b++) {
        rl->branches[b].num_points=pts_per_branch;
        rl->branches[b].K_values=malloc(pts_per_branch*sizeof(double));
        rl->branches[b].locus_points=malloc(pts_per_branch*sizeof(double complex));
        if (!rl->branches[b].K_values||!rl->branches[b].locus_points) { root_locus_free(rl); return -1; }
        if (b<rl->num_poles) {
            rl->branches[b].locus_points[0]=rl->open_poles[b];
            rl->branches[b].branch_start=b;
        } else {
            rl->branches[b].locus_points[0]=-1e6+(b-rl->num_poles)*100.0;
            rl->branches[b].branch_start=-1;
        }
        rl->branches[b].K_values[0]=0.0;
        double complex sc=rl->branches[b].locus_points[0];
        for (int p=1;p<pts_per_branch;p++) {
            double Kp=p*K_step;
            double complex GH_s=tf_evaluate(&L,sc);
            if (H) GH_s*=tf_evaluate(H,sc);
            double complex dsdK=0;
            if (cabs(GH_s)>1e-15) {
                double complex ds=1e-6, Gp=tf_evaluate(&L,sc+ds);
                if (H) Gp*=tf_evaluate(H,sc+ds);
                double complex dGH=(Gp-GH_s)/ds;
                if (cabs(dGH)>1e-15) dsdK=-GH_s/(Kp*dGH);
            }
            sc+=dsdK*K_step;
            for (int nit=0;nit<5;nit++) {
                GH_s=tf_evaluate(&L,sc); if (H) GH_s*=tf_evaluate(H,sc);
                double ang=carg(GH_s), target2=M_PI;
                while (ang>target2+M_PI) ang-=2*M_PI;
                while (ang<target2-M_PI) ang+=2*M_PI;
                if (fabs(ang-target2)<1e-8) break;
                double complex ds2=1e-6, Gp2=tf_evaluate(&L,sc+ds2);
                if (H) Gp2*=tf_evaluate(H,sc+ds2);
                double complex dGH2=(Gp2-GH_s)/ds2;
                double complex dang=-I*dGH2/GH_s;
                if (cabs(dang)>1e-15) sc-=(ang-target2)/dang;
            }
            rl->branches[b].locus_points[p]=sc;
            rl->branches[b].K_values[p]=Kp;
        }
    }
    return 0;
}
