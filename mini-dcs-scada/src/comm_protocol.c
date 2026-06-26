/*
 * comm_protocol.c - Industrial Communication Protocol Implementations
 *
 * Implements Modbus RTU/TCP protocol data model, CRC-16/LRC checksums,
 * IEEE 754 floating-point encoding, Hamming(7,4) error correction,
 * and Bristol-Babcock swinging door compression.
 */

#include "comm_protocol.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * Modbus CRC-16
 *
 * L5: Cyclic Redundancy Check for error detection.
 *
 * Modbus uses CRC-16 with polynomial 0x8005 (reversed to 0xA001):
 *   x^16 + x^15 + x^2 + 1
 *
 * The CRC is computed over all bytes of the message (slave address
 * through last data byte). The result is appended in little-endian
 * order (CRC low byte first, then high byte).
 *
 * Algorithm: Table-driven CRC for O(1) per byte.
 *
 * Initial value: 0xFFFF
 * Final XOR: 0x0000 (no XOR-out for Modbus)
 *
 * Error detection capability:
 *   - All single-bit errors
 *   - All double-bit errors
 *   - All odd number of bit errors
 *   - All burst errors up to 16 bits
 *   - 99.997% of burst errors > 16 bits
 * ============================================================================
 */

/* Pre-computed CRC-16 table (polynomial 0xA001) */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t modbus_crc16(const uint8_t *data, size_t len) {
    if (!data || len == 0) return 0xFFFF;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)(crc ^ data[i]);
        crc = (uint16_t)((crc >> 8) ^ crc16_table[idx]);
    }
    return crc;
}

bool modbus_crc16_check(const uint8_t *frame, size_t frame_len) {
    if (!frame || frame_len < 4) return false;
    /* Last two bytes are CRC (little-endian).
     * Compute CRC over all bytes EXCEPT the CRC itself. */
    uint16_t computed = modbus_crc16(frame, frame_len - 2);
    uint16_t received = (uint16_t)frame[frame_len - 1] << 8 | frame[frame_len - 2];
    return computed == received;
}

/* ============================================================================
 * Modbus LRC (Longitudinal Redundancy Check) for ASCII mode
 *
 * L2: Simpler checksum used in Modbus ASCII.
 *
 * LRC = -(sum of all data bytes) mod 256 (two's complement)
 * Equivalent to: LRC = (~sum + 1) & 0xFF (sum over all message bytes)
 *
 * Used in Modbus ASCII mode (rare in modern installations).
 * Modbus RTU uses CRC-16 instead.
 * ============================================================================
 */

uint8_t modbus_lrc(const uint8_t *data, size_t len) {
    if (!data) return 0;
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)((~sum + 1) & 0xFF);
}

/* ============================================================================
 * Modbus PDU Builders
 * ============================================================================
 */

int modbus_read_holding_regs(modbus_adu_t *adu, uint8_t slave,
                              uint16_t start_addr, uint16_t quantity) {
    if (!adu) return -1;
    if (quantity < 1 || quantity > 125) return -1;

    memset(adu, 0, sizeof(modbus_adu_t));
    adu->slave_addr   = slave;
    adu->function_code = MB_FC_READ_HOLDING_REGISTERS;

    /* PDU: [FC=0x03] [Start_Hi] [Start_Lo] [Qty_Hi] [Qty_Lo] */
    adu->data[0] = (uint8_t)(start_addr >> 8);
    adu->data[1] = (uint8_t)(start_addr & 0xFF);
    adu->data[2] = (uint8_t)(quantity >> 8);
    adu->data[3] = (uint8_t)(quantity & 0xFF);
    adu->data_len = 4;

    return 4;
}

int modbus_write_single_reg(modbus_adu_t *adu, uint8_t slave,
                             uint16_t addr, uint16_t value) {
    if (!adu) return -1;

    memset(adu, 0, sizeof(modbus_adu_t));
    adu->slave_addr    = slave;
    adu->function_code  = MB_FC_WRITE_SINGLE_REGISTER;

    /* PDU: [FC=0x06] [Addr_Hi] [Addr_Lo] [Value_Hi] [Value_Lo] */
    adu->data[0] = (uint8_t)(addr >> 8);
    adu->data[1] = (uint8_t)(addr & 0xFF);
    adu->data[2] = (uint8_t)(value >> 8);
    adu->data[3] = (uint8_t)(value & 0xFF);
    adu->data_len = 4;

    return 4;
}

int modbus_write_multiple_regs(modbus_adu_t *adu, uint8_t slave,
                                uint16_t start_addr, uint16_t quantity,
                                const uint16_t *values) {
    if (!adu || !values) return -1;
    if (quantity < 1 || quantity > 123) return -1;

    memset(adu, 0, sizeof(modbus_adu_t));
    adu->slave_addr    = slave;
    adu->function_code  = MB_FC_WRITE_MULTIPLE_REGISTERS;

    /* PDU: [FC=0x10] [Start_Hi] [Start_Lo] [Qty_Hi] [Qty_Lo]
     *      [ByteCount] [Data_Hi] [Data_Lo] ... */
    adu->data[0] = (uint8_t)(start_addr >> 8);
    adu->data[1] = (uint8_t)(start_addr & 0xFF);
    adu->data[2] = (uint8_t)(quantity >> 8);
    adu->data[3] = (uint8_t)(quantity & 0xFF);
    adu->data[4] = (uint8_t)(quantity * 2); /* Byte count */

    for (uint16_t i = 0; i < quantity; i++) {
        adu->data[5 + i * 2]     = (uint8_t)(values[i] >> 8);
        adu->data[5 + i * 2 + 1] = (uint8_t)(values[i] & 0xFF);
    }
    adu->data_len = (uint8_t)(5 + quantity * 2);

    return adu->data_len;
}

int modbus_parse_response(const modbus_adu_t *adu, modbus_data_model_t *dm) {
    if (!adu || !dm) return -1;

    uint8_t fc = adu->function_code;

    /* Check for exception response (MSB set) */
    if (fc & 0x80) {
        return -(int)(fc & 0x7F); /* Return negative exception code */
    }

    switch (fc) {
        case MB_FC_READ_HOLDING_REGISTERS: {
            uint8_t byte_count = adu->data[0];
            uint16_t reg_count = byte_count / 2;
            for (uint16_t i = 0; i < reg_count && i < 1024; i++) {
                dm->holding_regs[i] = (uint16_t)(adu->data[1 + i*2] << 8)
                                    | adu->data[1 + i*2 + 1];
            }
            return (int)reg_count;
        }

        case MB_FC_READ_COILS: {
            uint8_t byte_count = adu->data[0];
            for (uint8_t i = 0; i < byte_count && i < 128; i++) {
                for (uint8_t b = 0; b < 8; b++) {
                    int idx = i * 8 + b;
                    if (idx < 1024) {
                        dm->coils[idx] = (adu->data[1 + i] >> b) & 0x01;
                    }
                }
            }
            return byte_count * 8;
        }

        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return 0; /* Echo response, no data to decode */

        default:
            return -1;
    }
}

/* ============================================================================
 * Modbus Register <-> IEEE 754 Float Conversion
 *
 * L2: Industrial protocols commonly transport floating-point values
 *     as pairs of 16-bit registers (32 bits total).
 *
 * IEEE 754 single-precision (float32):
 *   Bit 31:     Sign (0=positive, 1=negative)
 *   Bits 30-23: Exponent (biased by 127)
 *   Bits 22-0:  Mantissa/Significand (with implicit leading 1)
 *
 * Value = (-1)^sign * 2^(exponent-127) * (1 + mantissa/2^23)
 *
 * Special values:
 *   - exponent=0, mantissa=0: +/- zero
 *   - exponent=0, mantissa!=0: subnormal numbers
 *   - exponent=255, mantissa=0: +/- infinity
 *   - exponent=255, mantissa!=0: NaN
 *
 * In Modbus, registers are transmitted big-endian (register order).
 * The byte order within each register may be big-endian or swapped
 * depending on the device (the infamous "Modbus byte swap" issue).
 * ============================================================================
 */

float modbus_regs_to_float(uint16_t reg_hi, uint16_t reg_lo) {
    uint32_t combined = ((uint32_t)reg_hi << 16) | reg_lo;
    float result;
    memcpy(&result, &combined, sizeof(float));
    return result;
}

void modbus_float_to_regs(float value, uint16_t *reg_hi, uint16_t *reg_lo) {
    if (!reg_hi || !reg_lo) return;
    uint32_t combined;
    memcpy(&combined, &value, sizeof(float));
    *reg_hi = (uint16_t)(combined >> 16);
    *reg_lo = (uint16_t)(combined & 0xFFFF);
}

int32_t modbus_regs_to_int32(uint16_t reg_hi, uint16_t reg_lo) {
    return (int32_t)(((uint32_t)reg_hi << 16) | reg_lo);
}

void modbus_int32_to_regs(int32_t value, uint16_t *reg_hi, uint16_t *reg_lo) {
    if (!reg_hi || !reg_lo) return;
    uint32_t uval = (uint32_t)value;
    *reg_hi = (uint16_t)(uval >> 16);
    *reg_lo = (uint16_t)(uval & 0xFFFF);
}

/* ============================================================================
 * OPC UA Variable
 * ============================================================================
 */

void opc_variable_init(opc_variable_t *var, uint16_t ns, uint32_t id,
                       const char *display_name, double initial_value) {
    if (!var) return;
    memset(var, 0, sizeof(opc_variable_t));
    var->node_id.namespace_idx = ns;
    var->node_id.identifier_numeric = id;
    var->node_id.is_numeric = true;
    if (display_name) {
        strncpy(var->display_name, display_name, sizeof(var->display_name) - 1);
    }
    var->value = initial_value;
    var->data_type = 11; /* OPC UA Double type ID */
    var->status_code = 0; /* Good */
}

/* ============================================================================
 * IEEE 754 Float Packing/Unpacking
 * ============================================================================
 */

void pack_float32_be(float value, uint8_t *buf) {
    if (!buf) return;
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    buf[0] = (uint8_t)(raw >> 24);
    buf[1] = (uint8_t)(raw >> 16);
    buf[2] = (uint8_t)(raw >> 8);
    buf[3] = (uint8_t)(raw);
}

float unpack_float32_be(const uint8_t *buf) {
    if (!buf) return 0.0f;
    uint32_t raw = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                 | ((uint32_t)buf[2] << 8) | buf[3];
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}

void pack_float32_le(float value, uint8_t *buf) {
    if (!buf) return;
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    buf[0] = (uint8_t)(raw);
    buf[1] = (uint8_t)(raw >> 8);
    buf[2] = (uint8_t)(raw >> 16);
    buf[3] = (uint8_t)(raw >> 24);
}

float unpack_float32_le(const uint8_t *buf) {
    if (!buf) return 0.0f;
    uint32_t raw = buf[0] | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}

void pack_double64_be(double value, uint8_t *buf) {
    if (!buf) return;
    uint64_t raw;
    memcpy(&raw, &value, sizeof(double));
    buf[0] = (uint8_t)(raw >> 56);
    buf[1] = (uint8_t)(raw >> 48);
    buf[2] = (uint8_t)(raw >> 40);
    buf[3] = (uint8_t)(raw >> 32);
    buf[4] = (uint8_t)(raw >> 24);
    buf[5] = (uint8_t)(raw >> 16);
    buf[6] = (uint8_t)(raw >> 8);
    buf[7] = (uint8_t)(raw);
}

double unpack_double64_be(const uint8_t *buf) {
    if (!buf) return 0.0;
    uint64_t raw = ((uint64_t)buf[0] << 56) | ((uint64_t)buf[1] << 48)
                 | ((uint64_t)buf[2] << 40) | ((uint64_t)buf[3] << 32)
                 | ((uint64_t)buf[4] << 24) | ((uint64_t)buf[5] << 16)
                 | ((uint64_t)buf[6] << 8) | buf[7];
    double value;
    memcpy(&value, &raw, sizeof(double));
    return value;
}

/* ============================================================================
 * Hamming(7,4) Error Correction Code
 *
 * L4: Forward Error Correction for noisy channels.
 *
 * Hamming(7,4) encodes 4 data bits (d1,d2,d3,d4) into 7 bits
 * (p1,p2,d1,p3,d2,d3,d4) using even parity:
 *
 * Parity bit p1 covers bits 1,3,5,7: p1 = d1 ^ d2 ^ d4
 * Parity bit p2 covers bits 2,3,6,7: p2 = d1 ^ d3 ^ d4
 * Parity bit p3 covers bits 4,5,6,7: p3 = d2 ^ d3 ^ d4
 *
 * The syndrome (s1,s2,s3) computed at the receiver points to the
 * bit position of a single-bit error:
 *
 *   s1 = p1 ^ d1 ^ d2 ^ d4
 *   s2 = p2 ^ d1 ^ d3 ^ d4
 *   s3 = p3 ^ d2 ^ d3 ^ d4
 *
 * Syndrome 0 = no error. Syndrome k (1-7) = error in bit k.
 *
 * This is a fundamental result in coding theory (Hamming, 1950)
 * and the simplest non-trivial error-correcting code.
 *
 * Minimum distance d_min = 3:
 *   - Can correct 1-bit errors
 *   - Can detect 2-bit errors (but not correct)
 *
 * Applications in SCADA: wireless sensor networks operating in
 * electromagnetically noisy industrial environments may use
 * Hamming codes at the link layer for robust telemetry.
 * ============================================================================
 */

uint8_t hamming74_encode(uint8_t data_nibble) {
    uint8_t d = data_nibble & 0x0F;

    /* Extract data bits (1-indexed positions 3,5,6,7) */
    uint8_t d1 = (d >> 0) & 1; /* Position 3 */
    uint8_t d2 = (d >> 1) & 1; /* Position 5 */
    uint8_t d3 = (d >> 2) & 1; /* Position 6 */
    uint8_t d4 = (d >> 3) & 1; /* Position 7 */

    /* Compute parity bits */
    uint8_t p1 = d1 ^ d2 ^ d4;       /* Covers 1,3,5,7 */
    uint8_t p2 = d1 ^ d3 ^ d4;       /* Covers 2,3,6,7 */
    uint8_t p3 = d2 ^ d3 ^ d4;       /* Covers 4,5,6,7 */

    /* Build 7-bit codeword: [p1][p2][d1][p3][d2][d3][d4] */
    uint8_t code = (p1 << 0) | (p2 << 1) | (d1 << 2)
                 | (p3 << 3) | (d2 << 4) | (d3 << 5) | (d4 << 6);

    return code;
}

uint8_t hamming74_decode(uint8_t code) {
    int err_pos = hamming74_error_pos(code);

    if (err_pos > 0) {
        /* Correct single-bit error by flipping the erroneous bit */
        code ^= (uint8_t)(1 << (err_pos - 1));
    }

    /* Extract data bits from corrected codeword */
    uint8_t d1 = (code >> 2) & 1;
    uint8_t d2 = (code >> 4) & 1;
    uint8_t d3 = (code >> 5) & 1;
    uint8_t d4 = (code >> 6) & 1;

    return (d1 | (d2 << 1) | (d3 << 2) | (d4 << 3));
}

int hamming74_error_pos(uint8_t code) {
    /* Extract data and parity bits */
    uint8_t p1 = (code >> 0) & 1;
    uint8_t p2 = (code >> 1) & 1;
    uint8_t d1 = (code >> 2) & 1;
    uint8_t p3 = (code >> 3) & 1;
    uint8_t d2 = (code >> 4) & 1;
    uint8_t d3 = (code >> 5) & 1;
    uint8_t d4 = (code >> 6) & 1;

    /* Syndrome computation */
    uint8_t s1 = p1 ^ d1 ^ d2 ^ d4;
    uint8_t s2 = p2 ^ d1 ^ d3 ^ d4;
    uint8_t s3 = p3 ^ d2 ^ d3 ^ d4;

    uint8_t syndrome = (s3 << 2) | (s2 << 1) | s1;

    /* Syndrome 0 = no error
     * Syndrome 1-7 = error at bit position (1-indexed)
     * Syndrome > 0 with invalid bit position = uncorrectable (>=2 errors) */
    return (syndrome == 0) ? 0 : (int)syndrome;
}

/* ============================================================================
 * Swinging Door Compression (Bristol-Babcock)
 *
 * L5: Lossy time-series compression algorithm used in OSIsoft PI System
 *     and other process historians.
 *
 * Algorithm:
 *   1. Archive the first data point.
 *   2. For each subsequent point, maintain "doors" from the last archived
 *      point. One door swings up (max slope), one swings down (min slope).
 *   3. As new points arrive, the doors swing inward (narrowing the corridor).
 *   4. When a point falls outside the corridor, archive the PREVIOUS point
 *      and reset the doors.
 *
 * The deviation parameter controls compression fidelity:
 *   - Smaller deviation = higher fidelity, lower compression
 *   - Larger deviation = lower fidelity, higher compression
 *
 * Compression ratio for typical process data (T = 1s, dev = 1%):
 *   - Steady operation: 20:1 to 100:1
 *   - Process upsets: 2:1 to 5:1
 *   - Overall average: 5:1 to 20:1
 *
 * This algorithm exploits the fact that process data is typically
 * smooth between upsets, making it highly compressible.
 * ============================================================================
 */

void swinging_door_init(swinging_door_t *sd, double deviation) {
    if (!sd) return;
    memset(sd, 0, sizeof(swinging_door_t));
    sd->deviation = fabs(deviation);
}

bool swinging_door_should_archive(swinging_door_t *sd, double value, double time_s) {
    if (!sd) return false;

    if (!sd->initialized) return true; /* Always archive first point */

    double dt = time_s - sd->last_time;
    if (dt <= 0.0) return false; /* No time progress */

    /* Check if new point falls between the two doors */
    double upper = sd->last_archived + sd->slope_upper * dt + sd->deviation;
    double lower = sd->last_archived + sd->slope_lower * dt - sd->deviation;

    if (value > upper || value < lower) {
        /* Point is outside the door corridor -> archive it */
        return true;
    }

    /* Update door slopes to narrow the corridor.
     * slope_to_point = (value - last_archived) / dt is the nominal slope.
     * It's used implicitly in the upper/lower door slopes below. */
    double slope_upper_new = (value + sd->deviation - sd->last_archived) / dt;
    double slope_lower_new = (value - sd->deviation - sd->last_archived) / dt;

    /* Doors only swing inward (narrower corridor) */
    if (slope_upper_new < sd->slope_upper || !sd->initialized) {
        sd->slope_upper = slope_upper_new;
    }
    if (slope_lower_new > sd->slope_lower || !sd->initialized) {
        sd->slope_lower = slope_lower_new;
    }

    return false;
}

void swinging_door_archive(swinging_door_t *sd, double value, double time_s) {
    if (!sd) return;

    sd->last_archived = value;
    sd->last_time     = time_s;

    /* Reset doors: wide open for the next segment */
    sd->slope_upper = +1e12; /* Effectively infinite */
    sd->slope_lower = -1e12;
    sd->initialized = true;
}