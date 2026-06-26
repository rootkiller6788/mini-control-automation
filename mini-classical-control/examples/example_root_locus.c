#include "control_core.h"
#include "control_analysis.h"
#include "control_rootlocus.h"
#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{
    printf("=== Root Locus Analysis Example ===\n\n");

    /* Plant: G(s) = 1 / (s·(s+2)·(s+4))
     * This is a type-1 3rd-order system. */
    transfer_function_t G;
    double num[]={1.0};
    double den[]={1.0, 6.0, 8.0, 0.0}; /* s³+6s²+8s */
    tf_init(&G,num,0,den,3);

    printf("Open-loop: G(s) = 1 / [s·(s+2)·(s+4)]\n\n");

    /* Find open-loop poles */
    pole_zero_t pz;
    tf_find_poles(&G,&pz);
    printf("Open-loop poles: ");
    for (int i=0;i<pz.num_poles;i++)
        printf("%.1f%+.1fj  ",creal(pz.poles[i]),cimag(pz.poles[i]));
    printf("\n\n");

    /* Evans Rules */
    printf("--- Evans Root Locus Rules ---\n");

    /* Asymptotes */
    double angles[3]; int na;
    rl_asymptote_angles(pz.num_poles,0,angles,&na);
    double centroid = rl_centroid(pz.poles,pz.num_poles,NULL,0);
    printf("R4-R5: %d asymptotes at angles: ",na);
    for (int i=0;i<na;i++) printf("%.0f deg  ",angles[i]*180.0/M_PI);
    printf("\n       Centroid: %.2f\n",centroid);

    /* Real-axis segments */
    printf("R3: Real-axis locus on: ");
    for (double x=-10.0;x<=2.0;x+=0.5) {
        if (rl_on_real_axis(x,pz.poles,pz.num_poles,NULL,0))
            printf("[%.0f,",x);
    }
    printf("...]\n");

    /* jw-axis crossing */
    double wc[2], Kc[2]; int nc;
    rl_jw_crossing(&G,NULL,wc,Kc,&nc);
    if (nc>0)
        printf("R7: jw crossing at w=%.2f rad/s, K_crit=%.2f\n",wc[0],Kc[0]);

    /* Departure angles */
    for (int i=0;i<pz.num_poles;i++) {
        if (fabs(cimag(pz.poles[i]))>1e-10) {
            double dep = rl_departure_angle(i,pz.poles,pz.num_poles,NULL,0);
            printf("R8: Departure from %.1f%+.1fj at %.0f deg\n",
                creal(pz.poles[i]),cimag(pz.poles[i]),dep*180.0/M_PI);
        }
    }

    /* Find gain for specific damping */
    printf("\n--- Design for zeta=0.5 ---\n");
    double K_des; double complex pole_des;
    if (rl_gain_for_damping(&G,NULL,0.5,&K_des,&pole_des)==0) {
        printf("Required gain K = %.3f\n",K_des);
        printf("Dominant pole at: %.3f%+.3fj\n",
               creal(pole_des),cimag(pole_des));
        /* Find all closed-loop poles at this gain */
        double complex cl_poles[3]; int ncl;
        rl_poles_at_gain(&G,NULL,K_des,cl_poles,&ncl);
        printf("All closed-loop poles at K=%.3f:\n",K_des);
        for (int i=0;i<ncl;i++)
            printf("  s%d = %.3f%+.3fj\n",i+1,creal(cl_poles[i]),cimag(cl_poles[i]));
    }

    /* Stability check via Routh-Hurwitz */
    printf("\n--- Stability Analysis ---\n");
    double Kcrit = routh_critical_gain(&G);
    printf("Critical gain (Routh) K_crit = %.2f\n",Kcrit);
    printf("For K < %.2f: system is STABLE\n",Kcrit);

    return 0;
}
