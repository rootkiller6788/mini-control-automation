#include "plc_core.h"
#include "plc_ladder.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

int main(void)
{
    plc_system_t plc;
    printf("PLC Test\n");

    /* L1: Init */
    assert(plc_init(&plc) == 0);
    assert(plc.mode == PLC_MODE_STOP);
    assert(plc_init(NULL) == -1);
    assert(plc_set_mode(&plc, PLC_MODE_RUN) == 0);
    assert(plc.mode == PLC_MODE_RUN);
    assert(plc_set_mode(&plc, PLC_MODE_STOP) == 0);
    assert(plc_set_mode(&plc, PLC_MODE_FAULT) == 0);
    assert(plc.fault_active == 1);
    plc_reset(&plc);
    assert(plc.fault_active == 0);

    /* L2: Digital I/O */
    plc_write_digital_output(&plc, 0, 1);
    assert(plc_read_digital_output(&plc, 0) == 1);
    plc_write_digital_output(&plc, 0, 0);
    assert(plc_read_digital_output(&plc, 0) == 0);
    plc_write_internal_bit(&plc, 0, 1);
    assert(plc_read_internal_bit(&plc, 0) == 1);

    /* L2: Analog I/O */
    plc_write_analog_output(&plc, 0, 3.14);
    assert(fabs(plc_read_analog_output(&plc, 0) - 3.14) < 1e-9);
    plc_write_internal_reg(&plc, 0, 42.0);
    assert(fabs(plc_read_internal_reg(&plc, 0) - 42.0) < 1e-9);

    /* L2: Timers */
    assert(plc_timer_config(&plc, 0, PLC_TIMER_TON, 100.0) == 0);
    plc_timer_set_input(&plc, 0, 1);
    plc_timer_update(&plc, 0, 50.0);
    assert(plc_timer_get_output(&plc, 0) == 0);
    plc_timer_update(&plc, 0, 60.0);
    assert(plc_timer_get_output(&plc, 0) == 1);

    /* L2: Counters */
    assert(plc_counter_config(&plc, 0, PLC_CTU, 3) == 0);
    plc_counter_update(&plc, 0, 1, 0, 0, 0);
    assert(plc_counter_get_cv(&plc, 0) == 1);

    /* L3: Edge Detection */
    plc_r_trig_t rt; plc_r_trig_reset(&rt);
    assert(plc_r_trig_update(&rt, 0) == 0);
    assert(plc_r_trig_update(&rt, 1) == 1);
    assert(plc_r_trig_update(&rt, 1) == 0);

    /* L3: Flip-Flops */
    plc_sr_ff_t sr; plc_sr_ff_reset(&sr);
    assert(plc_sr_ff_update(&sr, 1, 0) == 1);
    assert(plc_sr_ff_update(&sr, 0, 0) == 1);
    assert(plc_sr_ff_update(&sr, 0, 1) == 0);
    assert(plc_sr_ff_update(&sr, 1, 1) == 1);
    plc_rs_ff_t rs; plc_rs_ff_reset(&rs);
    assert(plc_rs_ff_update(&rs, 1, 1) == 0);

    /* L4: Timing */
    plc_timing_metrics_t m;
    plc_analyze_timing(&plc.scan_cfg, &m);
    assert(m.is_schedulable == 1);
    assert(plc_check_nyquist(500.0, 100.0) == 1);
    assert(plc_check_nyquist(150.0, 100.0) == 0);

    /* L5: RMS */
    double p[] = {1.0, 10.0, 100.0};
    double b[] = {0.2, 2.0, 10.0};
    assert(plc_rms_schedulable(p, b, 3) == 1);

    /* L5: PID */
    plc_pid_t pid; plc_pid_init(&pid, 2.0, 0.5, 0.1, 0.01, -100.0, 100.0);
    assert(fabs(pid.output - 0.0) < 1e-9);
    double u = plc_pid_update(&pid, 50.0, 40.0);
    assert(u > 0.0);

    /* L5: Filter */
    plc_iir_filter_t iir; plc_iir_filter_init(&iir, 0.5);
    assert(fabs(plc_iir_filter_update(&iir, 10.0) - 10.0) < 1e-9);

    /* L5: Ladder */
    ld_rung_t rung; ld_rung_init(&rung, 0);
    assert(ld_rung_add_contact_no(&rung, 0, PLC_IO_DISCRETE_INPUT) == 0);
    assert(ld_rung_add_coil_normal(&rung, 0, PLC_IO_DISCRETE_OUTPUT) == 0);
    assert(ld_rung_validate(&rung, &plc) == 1);

    /* L6: SFC */
    plc_sfc_engine_t sfc;
    assert(plc_sfc_init(&sfc, 4, 4) == 0);
    assert(plc_sfc_add_step(&sfc, 1, 1) == 0);
    assert(plc_sfc_add_step(&sfc, 2, 0) == 0);
    assert(plc_sfc_add_transition(&sfc, 10, 1, 2) == 0);
    assert(plc_sfc_get_active_step(&sfc) == 1);
    plc_sfc_set_condition(&sfc, 10, 1);
    plc_sfc_scan(&sfc, 10.0);
    assert(plc_sfc_get_active_step(&sfc) == 2);
    plc_sfc_free(&sfc);

    /* L7: Modbus CRC */
    uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    uint16_t crc = plc_modbus_crc16(data, 6);
    assert(crc != 0);
    uint8_t frame[8];
    memcpy(frame, data, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    assert(plc_modbus_crc16_check(frame, 8) == 1);

    /* L8: Redundancy */
    plc_redundancy_t red; plc_redundancy_init(&red, PLC_REDUN_PRIMARY);
    assert(red.role == PLC_REDUN_PRIMARY);

    /* L8: SIL */
    double pfd = plc_sil_compute_pfd(1e-6, 8760.0, 1);
    assert(pfd < 0.01);
    assert(plc_sil_get_level(pfd) >= 1);

    printf("All tests passed!\n");
    return 0;
}
