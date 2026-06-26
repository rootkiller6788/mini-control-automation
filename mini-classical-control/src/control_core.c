#include "control_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Polynomial Operations ─── */

double complex poly_eval(const double *coeff, int order, double complex s)
{
    double complex result = coeff[0];
    for (int i = 1; i <= order; i++)
        result = result * s + coeff[i];
    return result;
}

int poly_mul(const double *a, int oa, const double *b, int ob,
             double *c, int *oc)
{
    *oc = oa + ob;
    if (*oc > CTRL_MAX_ORDER) return -1;
    memset(c, 0, (*oc + 1) * sizeof(double));
    for (int i = 0; i <= oa; i++)
        for (int j = 0; j <= ob; j++)
            c[i + j] += a[i] * b[j];
    return 0;
}

int poly_add(const double *a, int oa, const double *b, int ob,
             double *c, int *oc)
{
    *oc = (oa > ob) ? oa : ob;
    if (*oc > CTRL_MAX_ORDER) return -1;
    int da = *oc - oa, db = *oc - ob;
    for (int i = 0; i <= *oc; i++) {
        double va = (i >= da) ? a[i - da] : 0.0;
        double vb = (i >= db) ? b[i - db] : 0.0;
        c[i] = va + vb;
    }
    return 0;
}

/* ─── TF Init & Management ─── */

int tf_init(transfer_function_t *G, const double *num, int num_order,
             const double *den, int den_order)
{
    if (!G || !num || !den) return -1;
    if (num_order < 0 || den_order < 0) return -1;
    if (num_order > CTRL_MAX_ORDER || den_order > CTRL_MAX_ORDER) return -1;
    if (den_order == 0 && fabs(den[0]) < 1e-15) return -1;
    G->num_order = num_order;
    G->den_order = den_order;
    memset(G->num, 0, sizeof(G->num));
    memset(G->den, 0, sizeof(G->den));
    for (int i = 0; i <= num_order; i++) G->num[i] = num[i];
    for (int i = 0; i <= den_order; i++) G->den[i] = den[i];
    tf_normalize(G);
    return 0;
}

void tf_normalize(transfer_function_t *G)
{
    if (!G || G->den_order < 0) return;
    if (fabs(G->den[0]) < 1e-15) return;
    double d0 = G->den[0];
    for (int i = 0; i <= G->den_order; i++) G->den[i] /= d0;
    G->gain = G->num[0];
    for (int i = 0; i <= G->num_order; i++) G->num[i] /= d0;
}

int tf_is_proper(const transfer_function_t *G)
    { return (!G) ? 0 : (G->den_order >= G->num_order ? 1 : 0); }

int tf_is_strictly_proper(const transfer_function_t *G)
    { return (!G) ? 0 : (G->den_order > G->num_order ? 1 : 0); }

double tf_dc_gain(const transfer_function_t *G)
{
    if (!G) return 0.0;
    double nd = G->num[G->num_order], dd = G->den[G->den_order];
    if (fabs(dd) < 1e-15) return HUGE_VAL;
    return nd / dd;
}

void tf_scale(transfer_function_t *G, double factor)
{
    if (!G) return;
    for (int i = 0; i <= G->num_order; i++) G->num[i] *= factor;
    G->gain *= factor;
}

/* ─── L2: System Analysis ─── */

system_type_t tf_system_type(const transfer_function_t *G)
{
    if (!G) return SYSTEM_TYPE_0;
    int type = 0;
    for (int i = G->den_order; i >= 0; i--) {
        if (fabs(G->den[i]) < 1e-15) type++; else break;
    }
    return (system_type_t)((type > 3) ? 3 : type);
}

int compute_step_specs(double omega_n, double zeta, step_specs_t *specs)
{
    if (!specs || omega_n <= 0.0 || zeta <= 0.0) return -1;
    memset(specs, 0, sizeof(step_specs_t));
    specs->damping_ratio = zeta;
    specs->natural_freq = omega_n;
    specs->final_value = 1.0;
    specs->steady_state_err = 0.0;
    if (zeta >= 1.0) {
        double sigma = zeta * omega_n;
        specs->rise_time = 2.2 / sigma;
        specs->settling_time = 4.0 / sigma;
        specs->peak_time = HUGE_VAL;
        specs->overshoot_pct = 0.0;
        specs->delay_time = 1.1 / sigma;
        return 0;
    }
    double wd = omega_n * sqrt(1.0 - zeta * zeta);
    double sigma = zeta * omega_n;
    specs->peak_time = M_PI / wd;
    specs->overshoot_pct = 100.0 * exp(-M_PI * zeta / sqrt(1.0 - zeta * zeta));
    specs->settling_time = 4.0 / sigma;
    specs->rise_time = (M_PI - acos(zeta)) / wd;
    specs->delay_time = 1.0 / sigma;
    return 0;
}

double complex tf_evaluate(const transfer_function_t *G, double complex s)
{
    if (!G) return 0.0;
    double complex nv = poly_eval(G->num, G->num_order, s);
    double complex dv = poly_eval(G->den, G->den_order, s);
    if (cabs(dv) < 1e-15) return INFINITY;
    return nv / dv;
}

int tf_freq_response(const transfer_function_t *G, double omega,
                     double *mag_db, double *phase_deg)
{
    if (!G || !mag_db || !phase_deg) return -1;
    double complex s = omega * I;
    double complex Gjw = tf_evaluate(G, s);
    double mag = cabs(Gjw);
    *mag_db = (mag < 1e-15) ? -300.0 : 20.0 * log10(mag);
    *phase_deg = atan2(cimag(Gjw), creal(Gjw)) * 180.0 / M_PI;
    return 0;
}

/* ─── L3: TF Algebra ─── */

int tf_series(const transfer_function_t *G1, const transfer_function_t *G2,
              transfer_function_t *result)
{
    if (!G1 || !G2 || !result) return -1;
    int nc, dc;
    double num[CTRL_MAX_ORDER+1], den[CTRL_MAX_ORDER+1];
    if (poly_mul(G1->num, G1->num_order, G2->num, G2->num_order, num, &nc) < 0) return -1;
    if (poly_mul(G1->den, G1->den_order, G2->den, G2->den_order, den, &dc) < 0) return -1;
    return tf_init(result, num, nc, den, dc);
}

int tf_parallel(const transfer_function_t *G1, const transfer_function_t *G2,
                transfer_function_t *result)
{
    if (!G1 || !G2 || !result) return -1;
    double n1d2[CTRL_MAX_ORDER+1], n2d1[CTRL_MAX_ORDER+1], cd[CTRL_MAX_ORDER+1];
    int on1, on2, oc;
    if (poly_mul(G1->num, G1->num_order, G2->den, G2->den_order, n1d2, &on1) < 0) return -1;
    if (poly_mul(G2->num, G2->num_order, G1->den, G1->den_order, n2d1, &on2) < 0) return -1;
    if (poly_mul(G1->den, G1->den_order, G2->den, G2->den_order, cd, &oc) < 0) return -1;
    int maxo = (on1 > on2) ? on1 : on2;
    double ns[CTRL_MAX_ORDER+1];
    memset(ns, 0, sizeof(ns));
    int off1 = maxo - on1, off2 = maxo - on2;
    for (int i = 0; i <= on1; i++) ns[off1 + i] += n1d2[i];
    for (int i = 0; i <= on2; i++) ns[off2 + i] += n2d1[i];
    return tf_init(result, ns, maxo, cd, oc);
}

int tf_unity_feedback(const transfer_function_t *G, transfer_function_t *T)
{
    if (!G || !T) return -1;
    double dcl[CTRL_MAX_ORDER+1]; int od;
    if (poly_add(G->den, G->den_order, G->num, G->num_order, dcl, &od) < 0) return -1;
    return tf_init(T, G->num, G->num_order, dcl, od);
}

int tf_feedback(const transfer_function_t *G, const transfer_function_t *H,
                transfer_function_t *T)
{
    if (!G || !H || !T) return -1;
    double nGH[CTRL_MAX_ORDER+1], dGH[CTRL_MAX_ORDER+1], nT[CTRL_MAX_ORDER+1], dT[CTRL_MAX_ORDER+1];
    int onG, odG, onT, odT;
    if (poly_mul(G->num, G->num_order, H->num, H->num_order, nGH, &onG) < 0) return -1;
    if (poly_mul(G->den, G->den_order, H->den, H->den_order, dGH, &odG) < 0) return -1;
    if (poly_add(dGH, odG, nGH, onG, dT, &odT) < 0) return -1;
    if (poly_mul(G->num, G->num_order, H->den, H->den_order, nT, &onT) < 0) return -1;
    return tf_init(T, nT, onT, dT, odT);
}

/* ─── L3: State-Space ↔ TF Conversion ─── */

int ss_init(state_space_t *ss, int n, int m, int p)
{
    if (!ss || n <= 0 || m <= 0 || p <= 0) return -1;
    ss->n = n; ss->m = m; ss->p = p;
    ss->A = calloc(n*n, sizeof(double));
    ss->B = calloc(n*m, sizeof(double));
    ss->C = calloc(p*n, sizeof(double));
    ss->D = calloc(p*m, sizeof(double));
    if (!ss->A || !ss->B || !ss->C || !ss->D) { ss_free(ss); return -1; }
    return 0;
}

void ss_free(state_space_t *ss)
{
    if (!ss) return;
    free(ss->A); ss->A = NULL;
    free(ss->B); ss->B = NULL;
    free(ss->C); ss->C = NULL;
    free(ss->D); ss->D = NULL;
}

int tf2ss(const transfer_function_t *G, state_space_t *ss)
{
    if (!G || !ss) return -1;
    int n = G->den_order;
    if (n <= 0) return -1;
    if (ss_init(ss, n, 1, 1) < 0) return -1;
    transfer_function_t Gn = *G;
    tf_normalize(&Gn);
    for (int i = 0; i < n; i++) {
        if (i < n-1) ss->A[(i+1)*n + i] = 1.0;
        ss->A[i] = -Gn.den[i+1];
    }
    ss->B[0] = 1.0;
    double b0 = (G->num_order == G->den_order) ? Gn.num[0] : 0.0;
    int offset = n - G->num_order;
    for (int i = 0; i < n; i++) {
        double bi = (i + 1 >= offset) ? Gn.num[i + 1 - offset] : 0.0;
        ss->C[i] = bi - Gn.den[i+1] * b0;
    }
    ss->D[0] = b0;
    return 0;
}

int ss2tf(const state_space_t *ss, transfer_function_t *G)
{
    /* Leverrier-Faddeeva: G(s)=C·(sI-A)⁻¹·B + D.
     * a_k = -tr(A·N_{k-1})/k, N_k = A·N_{k-1}+a_k·I, N₀=I */
    if (!ss || !G) return -1;
    int n = ss->n;
    if (ss->m != 1 || ss->p != 1) return -1;
    double *Na = calloc(n*n, sizeof(double)), *Nb = calloc(n*n, sizeof(double));
    double *At = calloc(n*n, sizeof(double)), *eye = calloc(n*n, sizeof(double));
    double a[CTRL_MAX_ORDER+1], b[CTRL_MAX_ORDER+1];
    if (!Na||!Nb||!At||!eye) { free(Na);free(Nb);free(At);free(eye); return -1; }
    for (int i=0;i<n;i++) eye[i*n+i]=1.0;
    memcpy(Na, eye, n*n*sizeof(double));
    a[0]=1.0;
    for (int k=1;k<=n;k++) {
        for (int i=0;i<n;i++) for (int j=0;j<n;j++) {
            double s=0.0;
            for (int l=0;l<n;l++) s+=ss->A[i*n+l]*Na[l*n+j];
            At[i*n+j]=s;
        }
        double tr=0.0;
        for (int i=0;i<n;i++) tr+=At[i*n+i];
        a[k]=-tr/k;
        for (int i=0;i<n;i++) for (int j=0;j<n;j++)
            Nb[i*n+j]=At[i*n+j]+((i==j)?a[k]:0.0);
        memcpy(Na,Nb,n*n*sizeof(double));
    }
    /* Compute b_k: numerator = D·det(sI-A) + C·adj(sI-A)·B
     * b₀ = D
     * b_k = D·a_k + C·N_{k-1}·B  for k=1..n */
    memcpy(Na,eye,n*n*sizeof(double));
    double Dval = ss->D[0];
    b[0] = Dval;
    for (int k = 1; k <= n; k++) {
        /* C·N_{k-1}·B */
        double CNkB = 0.0;
        for (int i = 0; i < n; i++) {
            double s = 0.0;
            for (int j = 0; j < n; j++) s += Na[i*n+j] * ss->B[j];
            CNkB += ss->C[i] * s;
        }
        b[k] = Dval * a[k] + CNkB;
        /* Update N_{k-1} → N_k = A·N_{k-1} + a_k·I */
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                double s = 0.0;
                for (int l = 0; l < n; l++) s += ss->A[i*n+l] * Na[l*n+j];
                Nb[i*n+j] = s + ((i == j) ? a[k] : 0.0);
            }
        memcpy(Na, Nb, n*n*sizeof(double));
    }
    free(Na); free(Nb); free(At); free(eye);
    /* Trim leading near-zero coefficients from numerator */
    if (fabs(b[0]) < 1e-14) {
        int fnz = 0;
        for (int i = 0; i <= n; i++)
            if (fabs(b[i]) > 1e-14) { fnz = i; break; }
        for (int i = 0; i <= n - fnz; i++) b[i] = b[i + fnz];
        return tf_init(G, b, n - fnz, a, n);
    }
    return tf_init(G, b, n, a, n);
}

/* ─── Pole-Zero Analysis via Companion Matrix + QR ─── */

static int poly_roots(const double *coeff, int order, double complex *roots)
{
    /* Polynomial roots = eigenvalues of companion matrix.
     * For order≤2: quadratic formula. For order≥3: QR iteration. */
    if (order <= 0) return 0;
    if (order == 1) { roots[0]=-coeff[1]/coeff[0]; return 1; }
    int n=order;
    double *C=calloc(n*n,sizeof(double));
    if (!C) return -1;
    for (int i=0;i<n;i++) {
        if (i<n-1) C[(i+1)*n+i]=1.0;
        C[i]=-coeff[i+1]/coeff[0];
    }
    if (n==2) {
        double a0=coeff[0],a1=coeff[1],a2=coeff[2];
        double disc=a1*a1-4.0*a0*a2;
        if (disc>=0) {
            double sd=sqrt(disc);
            roots[0]=(-a1+sd)/(2*a0); roots[1]=(-a1-sd)/(2*a0);
        } else {
            double re=-a1/(2*a0),im=sqrt(-disc)/(2*a0);
            roots[0]=re+im*I; roots[1]=re-im*I;
        }
        free(C); return 2;
    }
    for (int iter=0;iter<100;iter++) {
        int changed=0;
        for (int i=0;i<n-1;i++) {
            double a11=C[i*n+i],a12=C[i*n+i+1],a21=C[(i+1)*n+i],a22=C[(i+1)*n+i+1];
            double tr=a11+a22,det=a11*a22-a12*a21;
            double disc=tr*tr-4.0*det, mu;
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
            double tr=a+d,dt=a*d-b*c,disc2=tr*tr-4*dt;
            if (disc2<0) {
                double re=tr/2,im=sqrt(-disc2)/2;
                roots[i]=re+im*I; roots[i+1]=re-im*I;
            } else {
                double sd=sqrt(disc2);
                roots[i]=(tr+sd)/2; roots[i+1]=(tr-sd)/2;
            }
            i++;
        } else roots[i]=C[i*n+i];
    }
    free(C);
    return n;
}

int tf_find_poles(const transfer_function_t *G, pole_zero_t *pz)
{
    if (!G||!pz) return -1;
    int n=G->den_order;
    if (n<=0||n>CTRL_MAX_ORDER) return -1;
    pz->num_poles=poly_roots(G->den,n,pz->poles);
    pz->dc_gain=tf_dc_gain(G);
    return pz->num_poles;
}

int tf_find_zeros(const transfer_function_t *G, pole_zero_t *pz)
{
    if (!G||!pz) return -1;
    int m=G->num_order;
    if (m<=0) { pz->num_zeros=0; return 0; }
    if (m>CTRL_MAX_ORDER) return -1;
    pz->num_zeros=poly_roots(G->num,m,pz->zeros);
    pz->dc_gain=tf_dc_gain(G);
    return pz->num_zeros;
}

int tf_partial_fraction(const transfer_function_t *G,
                        double complex residues[], double complex roots[],
                        int *num_terms)
{
    if (!G||!residues||!roots||!num_terms) return -1;
    pole_zero_t pz;
    if (tf_find_poles(G,&pz)<0) return -1;
    int n=pz.num_poles; *num_terms=n;
    if (n<=0) return -1;
    int m=G->den_order;
    double dd[CTRL_MAX_ORDER+1];
    for (int i=0;i<m;i++) dd[i]=(m-i)*G->den[i];
    for (int k=0;k<n;k++) {
        roots[k]=pz.poles[k];
        double complex nv=poly_eval(G->num,G->num_order,roots[k]);
        double complex dv=poly_eval(dd,m-1,roots[k]);
        residues[k]=(cabs(dv)>1e-15)?nv/dv:0.0;
    }
    return 0;
}

/* ─── Controllability & Observability (Kalman 1960) ─── */

int controllability_matrix(const state_space_t *ss, double *Cmat, int *rank)
{
    if (!ss||!Cmat||!rank) return -1;
    int n=ss->n;
    double *col=calloc(n,sizeof(double)),*tmp=calloc(n,sizeof(double));
    if (!col||!tmp) { free(col);free(tmp); return -1; }
    for (int k=0;k<n;k++) {
        if (k==0) memcpy(col,ss->B,n*sizeof(double));
        else {
            for (int i=0;i<n;i++) {
                tmp[i]=0.0;
                for (int j=0;j<n;j++) tmp[i]+=ss->A[i*n+j]*col[j];
            }
            memcpy(col,tmp,n*sizeof(double));
        }
        for (int i=0;i<n;i++) Cmat[i*n+k]=col[i];
    }
    *rank=matrix_rank(Cmat,n,n,1e-10);
    free(col);free(tmp);
    return 0;
}

int observability_matrix(const state_space_t *ss, double *Omat, int *rank)
{
    if (!ss||!Omat||!rank) return -1;
    int n=ss->n;
    double *row=calloc(n,sizeof(double)),*tmp=calloc(n,sizeof(double));
    if (!row||!tmp) { free(row);free(tmp); return -1; }
    for (int k=0;k<n;k++) {
        if (k==0) memcpy(row,ss->C,n*sizeof(double));
        else {
            for (int j=0;j<n;j++) {
                tmp[j]=0.0;
                for (int i=0;i<n;i++) tmp[j]+=row[i]*ss->A[i*n+j];
            }
            memcpy(row,tmp,n*sizeof(double));
        }
        for (int j=0;j<n;j++) Omat[k*n+j]=row[j];
    }
    *rank=matrix_rank(Omat,n,n,1e-10);
    free(row);free(tmp);
    return 0;
}

int matrix_rank(const double *A, int rows, int cols, double tol)
{
    if (!A||rows<=0||cols<=0) return 0;
    double *M=malloc(rows*cols*sizeof(double));
    if (!M) return -1;
    memcpy(M,A,rows*cols*sizeof(double));
    int r=0;
    for (int j=0;j<cols&&r<rows;j++) {
        int prow=-1; double mv=tol;
        for (int i=r;i<rows;i++)
            if (fabs(M[i*cols+j])>mv) { mv=fabs(M[i*cols+j]); prow=i; }
        if (prow<0) continue;
        if (prow!=r)
            for (int c=j;c<cols;c++) {
                double t=M[r*cols+c]; M[r*cols+c]=M[prow*cols+c]; M[prow*cols+c]=t;
            }
        double piv=M[r*cols+j];
        for (int i=r+1;i<rows;i++) {
            double fac=M[i*cols+j]/piv;
            for (int c=j;c<cols;c++) M[i*cols+c]-=fac*M[r*cols+c];
        }
        r++;
    }
    free(M);
    return r;
}

/* ─── Convenience Constructors ─── */

int tf_first_order(transfer_function_t *G, double K, double tau)
    { double n[]={K},d[]={tau,1.0}; return tf_init(G,n,0,d,1); }

int tf_second_order(transfer_function_t *G, double K, double omega_n, double zeta)
{
    double n[]={K*omega_n*omega_n}, d[]={1.0,2.0*zeta*omega_n,omega_n*omega_n};
    return tf_init(G,n,0,d,2);
}

int tf_integrator(transfer_function_t *G)
    { double n[]={1.0},d[]={1.0,0.0}; return tf_init(G,n,0,d,1); }

int tf_pade_delay(double Td, transfer_function_t *G)
{
    double n[]={-Td/2.0,1.0}, d[]={Td/2.0,1.0};
    return tf_init(G,n,1,d,1);
}

int tf_snprint(char *buf, size_t size, const transfer_function_t *G)
{
    if (!buf||size==0||!G) return -1;
    int p=0;
    p+=snprintf(buf+p,size-p,"G(s)=(");
    for (int i=0;i<=G->num_order;i++) {
        if (i>0) p+=snprintf(buf+p,size-p,"+");
        p+=snprintf(buf+p,size-p,"%g",G->num[i]);
        int pw=G->num_order-i; if (pw>0) p+=snprintf(buf+p,size-p,"s^%d",pw);
    }
    p+=snprintf(buf+p,size-p,")/(");
    for (int i=0;i<=G->den_order;i++) {
        if (i>0) p+=snprintf(buf+p,size-p,"+");
        p+=snprintf(buf+p,size-p,"%g",G->den[i]);
        int pw=G->den_order-i; if (pw>0) p+=snprintf(buf+p,size-p,"s^%d",pw);
    }
    p+=snprintf(buf+p,size-p,")");
    return p;
}
