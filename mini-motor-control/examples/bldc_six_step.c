/**
 * bldc_six_step.c
 * L6 Canonical Problem: BLDC motor six-step commutation
 *
 * Demonstrates Hall-sensor decoding and six-step block commutation
 * for a Brushless DC motor.
 */

#include <stdio.h>
#include <string.h>
#include "motor_types.h"
#include "motor_control.h"

/* Complete hall pattern sequence for one electrical revolution */
static const uint8_t hall_sequence_cw[6][3] = {
    {0, 0, 1}, {0, 1, 0}, {0, 1, 1},
    {1, 0, 0}, {1, 0, 1}, {1, 1, 0}
};

static void print_switch_state(const char *label, const uint8_t sw[6])
{
    printf("%s: AH=%d AL=%d BH=%d BL=%d CH=%d CL=%d\n",
           label, sw[0], sw[1], sw[2], sw[3], sw[4], sw[5]);
}

int main(void)
{
    printf("========================================\n");
    printf(" BLDC Six-Step Commutation Demo\n");
    printf("========================================\n\n");

    printf("Hall Sensor Decoding Test:\n");
    printf("%-6s %-8s %-8s %-8s %-8s\n",
           "Sector", "H3", "H2", "H1", "Decoded");

    for (int i = 0; i < 6; i++) {
        uint8_t h1 = hall_sequence_cw[i][2];
        uint8_t h2 = hall_sequence_cw[i][1];
        uint8_t h3 = hall_sequence_cw[i][0];
        uint8_t sector = hall_to_sector(h1, h2, h3);

        printf("%-6d %-8d %-8d %-8d %-8d",
               i + 1, h3, h2, h1, sector);

        if (sector == 0) printf(" INVALID");
        printf("\n");
    }

    printf("\nInvalid code tests:\n");
    printf("000 -> sector %d (expected 0)\n", hall_to_sector(0,0,0));
    printf("111 -> sector %d (expected 0)\n\n", hall_to_sector(1,1,1));

    printf("Full Electrical Revolution (CW):\n");
    uint8_t current = 1;
    for (int step = 0; step < 6; step++) {
        uint8_t switches[6];
        bldc_commutation_pattern(current, switches);
        printf("Step %d (Sector %d): ", step + 1, current);
        print_switch_state("", switches);
        current = bldc_next_sector(current, MOTOR_DIR_CW);
    }

    printf("\nFull Electrical Revolution (CCW):\n");
    current = 1;
    for (int step = 0; step < 6; step++) {
        uint8_t switches[6];
        bldc_commutation_pattern(current, switches);
        printf("Step %d (Sector %d): ", step + 1, current);
        print_switch_state("", switches);
        current = bldc_next_sector(current, MOTOR_DIR_CCW);
    }

    printf("\nCommutation Verification:\n");
    printf("CW 6 steps returns to start: ");
    uint8_t test = 1;
    for (int i = 0; i < 6; i++) {
        test = bldc_next_sector(test, MOTOR_DIR_CW);
    }
    printf("%s (sector %d)\n", test == 1 ? "PASS" : "FAIL", test);

    printf("CW-CW-CCW returns to original: ");
    test = 3;
    test = bldc_next_sector(test, MOTOR_DIR_CW);
    test = bldc_next_sector(test, MOTOR_DIR_CW);
    test = bldc_next_sector(test, MOTOR_DIR_CCW);
    test = bldc_next_sector(test, MOTOR_DIR_CCW);
    printf("%s (sector %d)\n", test == 3 ? "PASS" : "FAIL", test);

    printf("\nBLDC Six-Step Commutation demo complete.\n");
    printf("For a 4-pole motor at 3000 RPM:\n");
    printf("  - Commutations per second: 3000/60 * 4 * 6 = %d Hz\n", 3000/60*4*6);
    printf("  - Commutation period: %.1f us\n", 1.0f/(3000.0f/60.0f*4.0f*6.0f)*1e6f);

    return 0;
}
