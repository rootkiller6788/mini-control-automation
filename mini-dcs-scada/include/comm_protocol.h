/*
 * comm_protocol.h - Industrial Communication Protocols
 *
 * Implements core protocol data structures for industrial communication
 * standards used in DCS/SCADA systems: Modbus RTU/TCP, OPC UA concepts,
 * and fieldbus abstractions.
 *
 * References:
 *   - Modbus Application Protocol Specification V1.1b3
 *   - Modbus over Serial Line Specification V1.02
 *   - IEC 62541 OPC Unified Architecture
 *   - IEC 61158 Industrial Communication Networks - Fieldbus
 *
 * Course Alignment:
 *   MIT 6.02 - Digital Communication Systems
 *   Stanford EE379 - Digital Communication
 *   Tsinghua - Industrial Communication and Fieldbus
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * L1: Modbus Protocol - The most common industrial serial protocol
 * ============================================================================
 */

#define MODBUS_MAX_DATA  252
#define MODBUS_ADU_MAX   256

/** Modbus function codes */
typedef enum {
    MB_FC_READ_COILS              = 0x01,
    MB_FC_READ_DISCRETE_INPUTS    = 0x02,
    MB_FC_READ_HOLDING_REGISTERS  = 0x03,
    MB_FC_READ_INPUT_REGISTERS    = 0x04,
    MB_FC_WRITE_SINGLE_COIL       = 0x05,
    MB_FC_WRITE_SINGLE_REGISTER   = 0x06,
    MB_FC_WRITE_MULTIPLE_COILS    = 0x0F,
    MB_FC_WRITE_MULTIPLE_REGISTERS = 0x10,
    MB_FC_READ_FIFO_QUEUE         = 0x18
} modbus_fc_t;

/** Modbus exception codes */
typedef enum {
    MB_EX_NONE               = 0x00,
    MB_EX_ILLEGAL_FUNCTION   = 0x01,
    MB_EX_ILLEGAL_DATA_ADDR  = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_FAILURE      = 0x04,
    MB_EX_ACKNOWLEDGE        = 0x05,
    MB_EX_SLAVE_BUSY         = 0x06,
    MB_EX_MEMORY_PARITY      = 0x08,
    MB_EX_GATEWAY_PATH       = 0x0A,
    MB_EX_GATEWAY_TARGET     = 0x0B
} modbus_exception_t;

/**
 * Modbus Application Data Unit (ADU) for RTU framing.
 *
 * RTU Framing (Serial):
 *   [SlaveAddr][PDU][CRC16_Lo][CRC16_Hi]
 *   Inter-frame gap >= 3.5 character times
 *
 * TCP Framing (Ethernet):
 *   [TransactionID][ProtocolID][Length][UnitID][PDU]
 *
 * PDU:
 *   [FunctionCode][Data...]
 */
typedef struct {
    uint8_t  slave_addr;          /* 1-247 for slaves, 0 = broadcast */
    uint8_t  function_code;       /* Modbus function code */
    uint8_t  data[MODBUS_MAX_DATA];
    uint8_t  data_len;           /* Length of data field */
    uint16_t crc;                /* CRC-16 for RTU mode */
    modbus_exception_t exception; /* Exception if function_code & 0x80 */
} modbus_adu_t;

/* Modbus data model: the logical data map of a device */
typedef struct {
    bool     coils[1024];            /* 0x0000-0x03FF: Read-write bits */
    bool     discrete_inputs[1024];  /* 0x0000-0x03FF: Read-only bits */
    uint16_t holding_regs[1024];    /* 0x0000-0x03FF: Read-write 16-bit */
    uint16_t input_regs[1024];      /* 0x0000-0x03FF: Read-only 16-bit */
} modbus_data_model_t;

/* ============================================================================
 * L2: Modbus Protocol Operations
 * ============================================================================
 */

/**
 * Compute CRC-16 for Modbus RTU.
 * Polynomial: 0xA001 (reversed 0x8005).
 * Initial value: 0xFFFF.
 *
 * Algorithm: Table-driven CRC for speed, or bitwise for clarity.
 */
uint16_t modbus_crc16(const uint8_t *data, size_t len);

/** Validate CRC-16 of a received Modbus RTU frame */
bool modbus_crc16_check(const uint8_t *frame, size_t frame_len);

/**
 * Build a Modbus PDU for reading holding registers.
 * PDU: [FC=0x03][StartAddr_Hi][StartAddr_Lo][Qty_Hi][Qty_Lo]
 *
 * @param adu       Output ADU
 * @param slave     Slave address
 * @param start_addr Starting register address (0-65535)
 * @param quantity  Number of registers to read (1-125)
 * @return          PDU length, or -1 on error
 */
int modbus_read_holding_regs(modbus_adu_t *adu, uint8_t slave,
                              uint16_t start_addr, uint16_t quantity);

/**
 * Build a Modbus PDU for writing a single register.
 * PDU: [FC=0x06][Addr_Hi][Addr_Lo][Value_Hi][Value_Lo]
 */
int modbus_write_single_reg(modbus_adu_t *adu, uint8_t slave,
                             uint16_t addr, uint16_t value);

/**
 * Build a Modbus PDU for writing multiple registers.
 * PDU: [FC=0x10][Start_Hi][Start_Lo][Qty_Hi][Qty_Lo][ByteCount][Data...]
 */
int modbus_write_multiple_regs(modbus_adu_t *adu, uint8_t slave,
                                uint16_t start_addr, uint16_t quantity,
                                const uint16_t *values);

/**
 * Parse a Modbus response PDU.
 * Decodes the response and extracts register values.
 *
 * @param adu       Received ADU
 * @param data_model Target data model to update
 * @return          0 on success, negative on error
 */
int modbus_parse_response(const modbus_adu_t *adu, modbus_data_model_t *dm);

/** Convert two Modbus registers (big-endian) to IEEE 754 float32 */
float modbus_regs_to_float(uint16_t reg_hi, uint16_t reg_lo);

/** Convert IEEE 754 float32 to two Modbus registers (big-endian) */
void modbus_float_to_regs(float value, uint16_t *reg_hi, uint16_t *reg_lo);

/** Convert two Modbus registers to int32 (big-endian) */
int32_t modbus_regs_to_int32(uint16_t reg_hi, uint16_t reg_lo);

/** Convert int32 to two Modbus registers (big-endian) */
void modbus_int32_to_regs(int32_t value, uint16_t *reg_hi, uint16_t *reg_lo);

/* ============================================================================
 * L2: LRC Checksum (Modbus ASCII mode)
 * ============================================================================
 */

/**
 * Longitudinal Redundancy Check (LRC) for Modbus ASCII.
 * LRC = two's complement of the sum of all bytes in the message (excluding
 * the colon, CR, and LF).
 */
uint8_t modbus_lrc(const uint8_t *data, size_t len);

/* ============================================================================
 * L2: OPC UA Data Types
 * ============================================================================
 */

/** OPC UA Node ID ? the fundamental addressing primitive in OPC UA */
typedef struct {
    uint16_t namespace_idx;
    uint32_t identifier_numeric;
    char     identifier_string[64];
    bool     is_numeric;
} opc_node_id_t;

/** OPC UA Variable node ? a data value with metadata */
typedef struct {
    opc_node_id_t node_id;
    char          display_name[64];
    double        value;
    uint32_t      data_type;      /* OPC UA TypeId for Double = 11 */
    int32_t       status_code;    /* OPC UA StatusCode (0 = Good) */
    uint64_t      source_timestamp;
    uint64_t      server_timestamp;
} opc_variable_t;

/** OPC UA Read Request */
typedef struct {
    opc_node_id_t nodes[128];
    uint32_t      count;
    uint64_t      request_handle;
} opc_read_request_t;

/** OPC UA Read Response */
typedef struct {
    opc_variable_t results[128];
    uint32_t       count;
    uint64_t       response_handle;
} opc_read_response_t;

/** Initialize an OPC UA variable node */
void opc_variable_init(opc_variable_t *var, uint16_t ns, uint32_t id,
                       const char *display_name, double initial_value);

/* ============================================================================
 * L2: Data Encoding Utilities
 * ============================================================================
 */

/**
 * Pack a 32-bit IEEE 754 float into 4 bytes (big-endian).
 * Used in Modbus, DNP3, and many fieldbus protocols.
 */
void pack_float32_be(float value, uint8_t *buf);

/** Unpack a 32-bit IEEE 754 float from 4 bytes (big-endian) */
float unpack_float32_be(const uint8_t *buf);

/** Pack a 32-bit IEEE 754 float into 4 bytes (little-endian, common in PLCs) */
void pack_float32_le(float value, uint8_t *buf);

/** Unpack a 32-bit IEEE 754 float from 4 bytes (little-endian) */
float unpack_float32_le(const uint8_t *buf);

/** Pack 64-bit double as 8 bytes (big-endian) */
void pack_double64_be(double value, uint8_t *buf);

/** Unpack 64-bit double from 8 bytes (big-endian) */
double unpack_double64_be(const uint8_t *buf);

/* ============================================================================
 * L4: Hamming Code Error Detection & Correction
 * ============================================================================
 */

/**
 * Hamming(7,4) code: encodes 4 data bits + 3 parity bits = 7 bits.
 * Can correct 1-bit errors and detect 2-bit errors.
 *
 * Parity bit positions:
 *   p1 covers bits 1,3,5,7
 *   p2 covers bits 2,3,6,7
 *   p4 covers bits 4,5,6,7
 *
 * Used in some industrial wireless sensor networks for robustness.
 */
uint8_t hamming74_encode(uint8_t data_nibble);  /* data in low 4 bits -> 7-bit code */
uint8_t hamming74_decode(uint8_t code);          /* 7-bit code -> 4-bit data (corrects 1-bit error) */
int    hamming74_error_pos(uint8_t code);        /* Returns error bit position, 0 = no error, -1 = uncorrectable */

/* ============================================================================
 * L5: Data Compression for SCADA Historian
 * ============================================================================
 */

/**
 * Swinging Door Compression (Bristol-Babcock algorithm).
 *
 * Widely used in OSIsoft PI System for lossy time-series compression.
 * Preserves the signal envelope within a specified deviation.
 *
 * The algorithm draws two "doors" from the last archived point:
 * one with slope +deviation/dt, one with -deviation/dt.
 * A new point is archived only when it falls outside the intersection
 * of all doors from all intermediate points.
 *
 * Compression ratio typically 5:1 to 20:1 for process data.
 */
typedef struct {
    double deviation;       /* Maximum allowed deviation */
    double last_archived;   /* Last archived value */
    double last_time;       /* Time of last archived value */
    double slope_upper;     /* Upper door slope */
    double slope_lower;     /* Lower door slope */
    bool   initialized;
} swinging_door_t;

void swinging_door_init(swinging_door_t *sd, double deviation);
bool swinging_door_should_archive(swinging_door_t *sd, double value, double time_s);
void swinging_door_archive(swinging_door_t *sd, double value, double time_s);

#endif /* COMM_PROTOCOL_H */