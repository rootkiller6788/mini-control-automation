#include "control_core.h"
#include "control_analysis.h"
#include "control_design.h"
#include <stdio.h>
#include <math.h>

static void print_pid(const char *method, const pid_params_t *p)
{
    printf("%-25s Kp=%-10.4f Ki=%-10.4f Kd=%-10.4f\n",method,p->Kp,p->Ki,p->Kd);
}

int main(void)
{
    printf("=== PID Tuning Method Comparison ===\n\n");
    printf("Plant: FOPDT K=1.5, L=0.3, T=4.0\n\n");

    double K=1.5, L=0.3, T=4.0;
    pid_params_t pid;

    /* Ziegler-Nichols step response */
    zn_step_response(K,L,T,2,&pid);
    print_pid("Ziegler-Nichols (step)", &pid);

    /* Ziegler-Nichols ultimate gain (approximate Ku, Pu from model) */
    /* For FOPDT: Ku ≈ 2*T/(K*L), Pu ≈ 2*L */
    double Ku = 2.0*T/(K*L);
    double Pu = 2.0*L;
    printf("\nUltimate gain Ku=%.2f, period Pu=%.2f\n",Ku,Pu);
    zn_ultimate_gain(Ku,Pu,2,&pid);
    print_pid("Ziegler-Nichols (ult)", &pid);

    /* Cohen-Coon */
    cohen_coon(K,L,T,2,&pid);
    print_pid("Cohen-Coon", &pid);

    /* Pole placement */
    pid_pole_placement_fopdt(K,L,T,0.8,1.0,&pid);
    print_pid("Pole placement", &pid);

    printf("\nMethod Comparison Notes:\n");
    printf("  ZN-Step:  Aggressive, ~25%% overshoot, fast settling\n");
    printf("  ZN-Ult:   Requires oscillation test, aggressive\n");
    printf("  Cohen-Coon: Designed for 1/4 decay ratio\n");
    printf("  Pole-place: Matches desired zeta,omega_n\n");

    return 0;
}
