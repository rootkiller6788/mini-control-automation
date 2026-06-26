#include "control_analysis.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Routh-Hurwitz Stability Criterion (Routh 1874, Hurwitz 1895) ---- */

int routh_hurwitz(const double *coeff, int order, int *num_rhp_roots)
{
    if (!coeff || !num_rhp_roots || order <= 0 || order > CTRL_MAX_ORDER) return -1;
    int n = order, rows = n + 1, cols = (n + 2) / 2;
    double *R = calloc(rows * cols, sizeof(double));
    if (!R) return -1;
    for (int i = 0; i < cols; i++) {
        if (2*i <= n) R[0*cols + i] = coeff[2*i];
        if (2*i + 1 <= n) R[1*cols + i] = coeff[2*i + 1];
    }
    for (int i = 2; i <= n; i++) {
        for (int j = 0; j < cols - 1; j++) {
            double b = R[(i-1)*cols + 0];
            if (fabs(b) < 1e-15) b = 1e-10;
            R[i*cols + j] = (R[(i-1)*cols + 0] * R[(i-2)*cols + j+1]
                             - R[(i-2)*cols + 0] * R[(i-1)*cols + j+1]) / b;
        }
    }
    int sc = 0, first = 1;
    double pnz = 0.0;
    for (int i = 0; i <= n; i++) {
        double cur = R[i*cols + 0];
        if (fabs(cur) < 1e-15) continue;
        if (first) { pnz = cur; first = 0; continue; }
        if ((pnz > 0 && cur < 0) || (pnz < 0 && cur > 0)) sc++;
        pnz = cur;
    }
    *num_rhp_roots = sc;
    free(R);
    return 0;
}

int routh_2nd_order(double a0, double a1, double a2)
    { return (a0>0 && a1>0 && a2>0) ? 1 : 0; }

int routh_3rd_order(double a0, double a1, double a2, double a3)
    { return (a0>0 && a1>0 && a2>0 && a3>0 && a1*a2 > a0*a3) ? 1 : 0; }

int routh_4th_order(double a0, double a1, double a2, double a3, double a4)
{
    if (a0<=0||a1<=0||a2<=0||a3<=0||a4<=0) return 0;
    if (a1*a2 <= a0*a3) return 0;
    if (a1*a2*a3 <= a0*a3*a3 + a1*a1*a4) return 0;
    return 1;
}

double routh_critical_gain(const transfer_function_t *G)
{
    if (!G) return -1.0;
    double Klow = 0.0, Khigh = 1e6;
    {
        double apk[CTRL_MAX_ORDER+1], kn[CTRL_MAX_ORDER+1]; int oa;
        for (int i=0;i<=G->num_order;i++) kn[i]=Klow*G->num[i];
        poly_add(G->den, G->den_order, kn, G->num_order, apk, &oa);
        int rhp; routh_hurwitz(apk, oa, &rhp);
        if (rhp > 0) return 0.0;
    }
    for (int iter=0; iter<80; iter++) {
        double Km = (Klow+Khigh)/2.0;
        double apk[CTRL_MAX_ORDER+1], kn[CTRL_MAX_ORDER+1]; int oa;
        for (int i=0;i<=G->num_order;i++) kn[i]=Km*G->num[i];
        poly_add(G->den, G->den_order, kn, G->num_order, apk, &oa);
        int rhp; routh_hurwitz(apk, oa, &rhp);
        if (rhp==0) Klow=Km; else Khigh=Km;
        if (Khigh-Klow<1e-6) break;
    }
    return Klow;
}

int tf_is_stable_cl(const transfer_function_t *G)
{
    if (!G) return -1;
    double dcl[CTRL_MAX_ORDER+1]; int od;
    if (poly_add(G->den, G->den_order, G->num, G->num_order, dcl, &od) < 0) return -1;
    int rhp;
    if (routh_hurwitz(dcl, od, &rhp) < 0) return -1;
    return (rhp==0) ? 1 : 0;
}

/* ---- Nyquist Stability Criterion (Nyquist 1932) ---- */

int nyquist_contour(const transfer_function_t *G, double *omega, int np, double complex *G_jw)
{
    if (!G||!omega||!G_jw||np<2) return -1;
    for (int i=0;i<np;i++) G_jw[i]=tf_evaluate(G, omega[i]*I);
    return 0;
}

int nyquist_encirclements(const double complex *G_jw, int np)
{
    if (!G_jw||np<2) return 0;
    int N=0;
    for (int i=1;i<np;i++) {
        double rp=creal(G_jw[i-1]), ip=cimag(G_jw[i-1]);
        double rc=creal(G_jw[i]),   ic=cimag(G_jw[i]);
        if (ip*ic<0 && rp<-1.0 && rc<-1.0) {
            if (ip>0 && ic<0) N++; else if (ip<0 && ic>0) N--;
        }
        if (fabs(ip)>1e-10 && fabs(ic)<1e-10 && rc<-1.0) N += (ip>0)?1:-1;
        if (fabs(ip)<1e-10 && fabs(ic)>1e-10 && rp<-1.0) N += (ic<0)?1:-1;
    }
    return N;
}

int nyquist_stability(const transfer_function_t *G, int P)
{
    if (!G) return -1;
    int Np=200;
    double *om=malloc(Np*sizeof(double));
    double complex *Gj=malloc(Np*sizeof(double complex));
    if (!om||!Gj) { free(om);free(Gj); return -1; }
    for (int i=0;i<Np;i++) { double t=(double)i/(Np-1); om[i]=pow(10.0,-3.0+6.0*t); }
    nyquist_contour(G,om,Np,Gj);
    int N=nyquist_encirclements(Gj,Np);
    free(om);free(Gj);
    return (N+P==0)?1:0;
}

/* ---- Bode Plot & Margins ---- */

int bode_plot(const transfer_function_t *G, double wmin, double wmax, int np,
              double *omega, double *mag_db, double *phase_deg)
{
    if (!G||!omega||!mag_db||!phase_deg||np<2) return -1;
    if (wmin<=0||wmax<=wmin) return -1;
    double lmin=log10(wmin), lmax=log10(wmax);
    for (int i=0;i<np;i++) {
        double lw=lmin+(lmax-lmin)*(double)i/(np-1);
        omega[i]=pow(10.0,lw);
        tf_freq_response(G,omega[i],&mag_db[i],&phase_deg[i]);
    }
    return 0;
}

int compute_margins(const transfer_function_t *G, freq_specs_t *specs)
{
    if (!G||!specs) return -1;
    memset(specs,0,sizeof(freq_specs_t));
    int N=500;
    double *om=malloc(N*sizeof(double)),*mg=malloc(N*sizeof(double));
    double *ph=malloc(N*sizeof(double));
    if (!om||!mg||!ph) { free(om);free(mg);free(ph); return -1; }
    bode_plot(G,1e-3,1e4,N,om,mg,ph);
    int pci=-1;
    for (int i=1;i<N;i++)
        if ((ph[i-1]>-180.0&&ph[i]<=-180.0)||(ph[i-1]<-180.0&&ph[i]>=-180.0)) { pci=i; break; }
    if (pci>=0) { specs->phase_crossover=om[pci]; specs->gain_margin_db=-mg[pci]; }
    else specs->gain_margin_db=INFINITY;
    int gci=-1;
    for (int i=1;i<N;i++)
        if ((mg[i-1]>0.0&&mg[i]<=0.0)||(mg[i-1]<0.0&&mg[i]>=0.0)) { gci=i; break; }
    if (gci>=0) { specs->gain_crossover=om[gci]; specs->phase_margin_deg=180.0+ph[gci]; }
    else specs->phase_margin_deg=INFINITY;
    specs->dc_gain_db=mg[0];
    specs->bandwidth=bandwidth_find(G);
    double mm=-INFINITY; int ri=0;
    for (int i=0;i<N;i++) if (mg[i]>mm) { mm=mg[i]; ri=i; }
    specs->resonant_peak_db=mm; specs->resonant_freq=om[ri];
    free(om);free(mg);free(ph);
    return 0;
}

int sensitivity_function(const transfer_function_t *G, double omega, double *Sm, double *Tm)
{
    if (!G||!Sm||!Tm) return -1;
    double complex Gj=tf_evaluate(G,omega*I);
    double complex S=1.0/(1.0+Gj), T=Gj/(1.0+Gj);
    double ms=cabs(S), mt=cabs(T);
    *Sm=(ms>1e-15)?20.0*log10(ms):-300.0;
    *Tm=(mt>1e-15)?20.0*log10(mt):-300.0;
    return 0;
}

double bandwidth_find(const transfer_function_t *G)
{
    if (!G) return 0.0;
    transfer_function_t T;
    if (tf_unity_feedback(G,&T)<0) return 0.0;
    double wl=1e-6, wh=1e6, mg, ph;
    tf_freq_response(&T,wl,&mg,&ph);
    if (mg>-2.9) return INFINITY;
    for (int iter=0;iter<60;iter++) {
        double wm=sqrt(wl*wh);
        tf_freq_response(&T,wm,&mg,&ph);
        if (mg>-3.0) wl=wm; else wh=wm;
    }
    return (wl+wh)/2.0;
}

/* ---- Step & Impulse Response (RK4 via state-space) ---- */

int step_response(const transfer_function_t *G, double tf, double dt,
                  double *t, double *y, int maxp, int *np)
{
    if (!G||!t||!y||!np) return -1;
    state_space_t ss;
    if (tf2ss(G,&ss)<0) return -1;
    int n=ss.n, steps=(int)(tf/dt);
    if (steps>maxp) steps=maxp;
    double *x=calloc(n,sizeof(double)),*k1=calloc(n,sizeof(double));
    double *k2=calloc(n,sizeof(double)),*k3=calloc(n,sizeof(double));
    double *k4=calloc(n,sizeof(double)),*xt=calloc(n,sizeof(double));
    if (!x||!k1||!k2||!k3||!k4||!xt) {
        free(x);free(k1);free(k2);free(k3);free(k4);free(xt); ss_free(&ss); return -1;
    }
    for (int s=0;s<=steps;s++) {
        double tn=s*dt; t[s]=tn;
        double yout=ss.D[0];
        for (int i=0;i<n;i++) yout+=ss.C[i]*x[i];
        y[s]=yout;
        if (s==steps) break;
        for (int i=0;i<n;i++) { double v=ss.B[i]; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*x[j]; k1[i]=v; }
        for (int i=0;i<n;i++) xt[i]=x[i]+0.5*dt*k1[i];
        for (int i=0;i<n;i++) { double v=ss.B[i]; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*xt[j]; k2[i]=v; }
        for (int i=0;i<n;i++) xt[i]=x[i]+0.5*dt*k2[i];
        for (int i=0;i<n;i++) { double v=ss.B[i]; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*xt[j]; k3[i]=v; }
        for (int i=0;i<n;i++) xt[i]=x[i]+dt*k3[i];
        for (int i=0;i<n;i++) { double v=ss.B[i]; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*xt[j]; k4[i]=v; }
        for (int i=0;i<n;i++) x[i]+=dt/6.0*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
    }
    *np=steps+1;
    free(x);free(k1);free(k2);free(k3);free(k4);free(xt);
    ss_free(&ss);
    return 0;
}

int impulse_response(const transfer_function_t *G, double tf, double dt,
                     double *t, double *y, int maxp, int *np)
{
    if (!G||!t||!y||!np) return -1;
    state_space_t ss;
    if (tf2ss(G,&ss)<0) return -1;
    int n=ss.n, steps=(int)(tf/dt);
    if (steps>maxp) steps=maxp;
    double *x=calloc(n,sizeof(double)),*k1=calloc(n,sizeof(double));
    double *k2=calloc(n,sizeof(double)),*k3=calloc(n,sizeof(double));
    double *k4=calloc(n,sizeof(double)),*xt=calloc(n,sizeof(double));
    if (!x||!k1||!k2||!k3||!k4||!xt) {
        free(x);free(k1);free(k2);free(k3);free(k4);free(xt); ss_free(&ss); return -1;
    }
    for (int i=0;i<n;i++) x[i]=ss.B[i];
    for (int s=0;s<=steps;s++) {
        t[s]=s*dt; double yout=0;
        for (int i=0;i<n;i++) yout+=ss.C[i]*x[i];
        y[s]=yout;
        if (s==steps) break;
        for (int i=0;i<n;i++) { double v=0; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*x[j]; k1[i]=v; }
        for (int i=0;i<n;i++) xt[i]=x[i]+0.5*dt*k1[i];
        for (int i=0;i<n;i++) { double v=0; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*xt[j]; k2[i]=v; }
        for (int i=0;i<n;i++) xt[i]=x[i]+0.5*dt*k2[i];
        for (int i=0;i<n;i++) { double v=0; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*xt[j]; k3[i]=v; }
        for (int i=0;i<n;i++) xt[i]=x[i]+dt*k3[i];
        for (int i=0;i<n;i++) { double v=0; for (int j=0;j<n;j++) v+=ss.A[i*n+j]*xt[j]; k4[i]=v; }
        for (int i=0;i<n;i++) x[i]+=dt/6.0*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
    }
    *np=steps+1;
    free(x);free(k1);free(k2);free(k3);free(k4);free(xt);
    ss_free(&ss);
    return 0;
}

/* ---- Steady-State Errors & Error Constants ---- */

int steady_state_errors(const transfer_function_t *G, double *es, double *er, double *ep)
{
    if (!G||!es||!er||!ep) return -1;
    double Kp,Kv,Ka;
    error_constants(G,&Kp,&Kv,&Ka);
    *es=(Kp>1e-15)?1.0/(1.0+Kp):INFINITY;
    *er=(Kv>1e-15)?1.0/Kv:INFINITY;
    *ep=(Ka>1e-15)?1.0/Ka:INFINITY;
    return 0;
}

int error_constants(const transfer_function_t *G, double *Kp, double *Kv, double *Ka)
{
    if (!G||!Kp||!Kv||!Ka) return -1;
    *Kp=tf_dc_gain(G);
    system_type_t type=tf_system_type(G);
    if (type>=1) { double n0=G->num[G->num_order]; int dro=G->den_order-type; *Kv=n0/G->den[dro]; }
    else *Kv=0.0;
    if (type>=2) { double n0=G->num[G->num_order]; int dro=G->den_order-2; *Ka=n0/G->den[dro]; }
    else *Ka=0.0;
    return 0;
}

/* ---- Dominant Pole Analysis ---- */

int dominant_pole_params(const transfer_function_t *G, double *zeta, double *wn)
{
    if (!G||!zeta||!wn) return -1;
    pole_zero_t pz;
    if (tf_find_poles(G,&pz)<0) return -1;
    double best_re=-INFINITY; int best_i=-1;
    for (int i=0;i<pz.num_poles;i++) {
        double re=creal(pz.poles[i]), im=cimag(pz.poles[i]);
        if (fabs(im)>1e-10 && re>best_re)
            for (int j=i+1;j<pz.num_poles;j++)
                if (fabs(creal(pz.poles[j])-re)<1e-10 && fabs(cimag(pz.poles[j])+im)<1e-10)
                    { best_re=re; best_i=i; break; }
    }
    if (best_i<0) {
        double mr=-INFINITY;
        for (int i=0;i<pz.num_poles;i++)
            if (creal(pz.poles[i])>mr) { mr=creal(pz.poles[i]); best_i=i; }
        if (best_i>=0) { *wn=fabs(creal(pz.poles[best_i])); *zeta=1.0; return 0; }
        return -1;
    }
    pole_to_zeta_omega(pz.poles[best_i],zeta,wn);
    return 0;
}

stability_t pole_stability(const pole_zero_t *pz)
{
    if (!pz) return STABLE;
    int marg=0;
    for (int i=0;i<pz->num_poles;i++) {
        double re=creal(pz->poles[i]);
        if (re>1e-10) return UNSTABLE;
        if (fabs(re)<1e-10) marg=1;
    }
    if (marg) return MARGINALLY_STABLE;
    return STABLE;
}

void pole_to_zeta_omega(double complex pole, double *zeta, double *wn)
{
    double sigma=-creal(pole), wd=fabs(cimag(pole));
    *wn=hypot(sigma,wd);
    *zeta=(*wn>1e-15)?sigma/(*wn):0.0;
}
