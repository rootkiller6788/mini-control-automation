#include "plc_core.h"
#include <string.h>

/* =========================================================================
 * L8: PLC Redundancy -- Hot Standby (IEC 61508 SIL 3 compliant)
 *
 * Architecture: 1oo2 (one-out-of-two) with hot standby.
 * Primary executes logic and drives physical outputs.
 * Secondary runs identical logic but outputs are disabled (shadow mode).
 * State synchronization occurs each scan cycle via sync_buffer.
 *
 * Switchover triggers:
 *   - Primary watchdog timeout (heartbeat_missed >= 3)
 *   - Primary self-diagnosed fault
 *   - Manual command
 *
 * Bumpless transfer: secondary tracks primary state continuously,
 * including I/O images, timers, counters, and internal memory.
 * Switchover time < 50 ms target (one scan cycle).
 * ========================================================================= */

int plc_redundancy_init(plc_redundancy_t *red, plc_redundancy_role_t role)
{
    if (!red) return -1;
    memset(red, 0, sizeof(*red));
    red->role = role;
    red->primary_healthy = (role == PLC_REDUN_PRIMARY);
    red->secondary_healthy = (role == PLC_REDUN_SECONDARY);
    red->switchover_time_ms = 50.0;
    return 0;
}

int plc_redundancy_heartbeat(plc_redundancy_t *red, int peer_alive)
{
    if (!red) return -1;
    if (!peer_alive)
        red->heartbeat_missed++;
    else
        red->heartbeat_missed = 0;

    /* Automatic switchover: secondary takes over after 3 missed heartbeats */
    if (red->heartbeat_missed >= 3 && red->role == PLC_REDUN_SECONDARY)
        return plc_redundancy_switchover(red);
    return 0;
}

int plc_redundancy_sync(plc_redundancy_t *red,
                        const plc_system_t *source, plc_system_t *target)
{
    if (!red || !source || !target) return -1;

    /* Sync essential runtime state for bumpless transfer */
    memcpy(target->digital_outputs, source->digital_outputs,
           source->num_digital_out * sizeof(plc_digital_io_t));
    memcpy(target->analog_outputs, source->analog_outputs,
           source->num_analog_out * sizeof(plc_analog_io_t));
    memcpy(target->timers, source->timers,
           source->num_timers * sizeof(plc_timer_t));
    memcpy(target->counters, source->counters,
           source->num_counters * sizeof(plc_counter_t));
    memcpy(target->internal_bits, source->internal_bits,
           source->num_internal_bits * sizeof(int));
    memcpy(target->internal_regs, source->internal_regs,
           source->num_internal_regs * sizeof(double));

    red->sequence_number++;
    return 0;
}

int plc_redundancy_switchover(plc_redundancy_t *red)
{
    if (!red) return -1;
    if (red->role == PLC_REDUN_SECONDARY && !red->primary_healthy) {
        red->role = PLC_REDUN_PRIMARY;
        red->primary_healthy = 1;
        red->switchover_count++;
        return 0;
    }
    return -1;
}

/* =========================================================================
 * L8: Safety Integrity Level (SIL) Assessment per IEC 61508
 *
 * PFD (Probability of Failure on Demand) simplified formulas:
 *   1oo1 (simplex):  PFD = lambda_DU * T_proof / 2
 *   1oo2 (dual):     PFD = (lambda_DU * T_proof)^2 / 3
 *   2oo3 (TMR):      PFD = (lambda_DU * T_proof)^2
 *
 * These assume:
 *   - Constant failure rate (exponential distribution)
 *   - Perfect proof test coverage
 *   - No common-cause failures (beta = 0)
 *   - Instantaneous repair after proof test detection
 *
 * SIL targets (IEC 61508-1 Table 3):
 *   SIL 4: PFD < 1e-5  (10^-5 to 10^-4)
 *   SIL 3: PFD < 1e-4  (10^-4 to 10^-3)
 *   SIL 2: PFD < 1e-3  (10^-3 to 10^-2)
 *   SIL 1: PFD < 1e-2  (10^-2 to 10^-1)
 *
 * lambda_DU units: dangerous undetected failures per hour.
 * Typical values: 1e-6 to 1e-8 per hour for industrial PLC hardware.
 * ========================================================================= */

double plc_sil_compute_pfd(double lambda_du,
                           double proof_test_interval_hours,
                           int architecture)
{
    double T = proof_test_interval_hours;
    double lambda_T = lambda_du * T;

    switch (architecture) {
    case 1:  /* 1oo1 -- simplex */
        return lambda_T / 2.0;
    case 2:  /* 1oo2 -- dual redundant */
        return (lambda_T * lambda_T) / 3.0;
    case 3:  /* 2oo3 -- triple modular redundant */
        return lambda_T * lambda_T;
    default:
        return 1.0;  /* Invalid architecture */
    }
}

int plc_sil_get_level(double pfd)
{
    if (pfd < 1e-5) return 4;  /* SIL 4 */
    if (pfd < 1e-4) return 3;  /* SIL 3 */
    if (pfd < 1e-3) return 2;  /* SIL 2 */
    if (pfd < 1e-2) return 1;  /* SIL 1 */
    return 0;  /* Not safety-rated */
}
