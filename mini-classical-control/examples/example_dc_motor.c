#include "control_core.h"
#include "control_analysis.h"
#include "control_design.h"
#include "control_applications.h"
#include <stdio.h>
#include <math.h>

int main(void)
{
    printf("=== DC Motor Speed Control Example ===\n\n");
    /* Motor parameters (small DC motor) */
    double J=0.01, b=0.1, Km=0.05, Kb=0.05, Ra=1.0, La=0.5;
    printf("Motor: J=%.3f, b=%.3f, Km=%.3f, Kb=%.3f, Ra=%.1f, La=%.1f\n\n",
           J,b,Km,Kb,Ra,La);

    /* Full 2nd-order model */
    transfer_function_t G_full;
    dc_motor_model(J,b,Km,Kb,Ra,La,&G_full);
    char buf[256];
    tf_snprint(buf,sizeof(buf),&G_full);
    printf("Full model: %s\n",buf);

    /* Simplified 1st-order model */
    transfer_function_t G_simp;
    dc_motor_simplified(J,b,Km,Kb,Ra,&G_simp);
    tf_snprint(buf,sizeof(buf),&G_simp);
    printf("Simplified: %s\n",buf);

    /* Design PI controller for simplified model */
    pid_params_t pid;
    dc_motor_pi_speed_control(J,b,Km,Kb,Ra,0.8,5.0,&pid);
    printf("\nPI Controller: Kp=%.4f, Ki=%.4f\n",pid.Kp,pid.Ki);

    /* Analyze closed-loop */
    transfer_function_t C, L, T;
    pid_to_tf(&pid,&C);
    loop_tf(&C,&G_simp,&L);
    tf_unity_feedback(&L,&T);

    /* Step response specs */
    double zeta, wn;
    dominant_pole_params(&T,&zeta,&wn);
    step_specs_t specs;
    compute_step_specs(wn,zeta,&specs);
    printf("\nClosed-loop performance:\n");
    printf("  Damping ratio zeta = %.3f\n",zeta);
    printf("  Natural freq wn    = %.3f rad/s\n",wn);
    printf("  Overshoot          = %.1f%%\n",specs.overshoot_pct);
    printf("  Settling time (2%%) = %.3f s\n",specs.settling_time);

    /* Frequency-domain */
    freq_specs_t fs;
    compute_margins(&L,&fs);
    printf("\nFrequency-domain:\n");
    printf("  Gain margin   = %.1f dB\n",fs.gain_margin_db);
    printf("  Phase margin  = %.1f deg\n",fs.phase_margin_deg);
    printf("  Bandwidth     = %.3f rad/s\n",fs.bandwidth);

    /* Stability check */
    int stable = tf_is_stable_cl(&L);
    printf("\nStability: %s\n",stable?"STABLE":"UNSTABLE");

    return 0;
}
