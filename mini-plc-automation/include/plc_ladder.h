#ifndef PLC_LADDER_H
#define PLC_LADDER_H
#include "plc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Ladder Diagram (LD) - IEC 61131-3
 *
 * Ladder Diagram is a graphical programming language resembling
 * electrical relay logic diagrams. Two vertical power rails (left=L,
 * right=neutral) with horizontal rungs of contacts and coils.
 *
 * Boolean algebra foundation:
 *   Series contacts   = AND (conjunction)
 *   Parallel contacts = OR  (disjunction)
 *   Normally-closed    = NOT (negation)
 *   Coil               = assignment (output = rung condition)
 * ========================================================================= */

/** Contact types in ladder logic */
typedef enum {
    LD_CONTACT_NO  = 0,  /* Normally Open:  -| |-   (examine if ON)  */
    LD_CONTACT_NC  = 1,  /* Normally Closed: -|/|-   (examine if OFF) */
    LD_CONTACT_POS = 2,  /* Positive edge:   -|P|-   (one-shot rise) */
    LD_CONTACT_NEG = 3   /* Negative edge:   -|N|-   (one-shot fall) */
} ld_contact_type_t;

/** Coil types in ladder logic */
typedef enum {
    LD_COIL_NORMAL   = 0,  /* -( )-   Standard output coil          */
    LD_COIL_NEGATED  = 1,  /* -(/)    Negated (inverted) output     */
    LD_COIL_SET      = 2,  /* -(S)-   Set (latch) coil              */
    LD_COIL_RESET    = 3,  /* -(R)-   Reset (unlatch) coil          */
    LD_COIL_POS      = 4,  /* -(P)-   Positive edge triggered       */
    LD_COIL_NEG      = 5   /* -(N)-   Negative edge triggered       */
} ld_coil_type_t;

/** Function block types available within ladder rungs */
typedef enum {
    LD_FB_TIMER_TON  = 0,
    LD_FB_TIMER_TOF  = 1,
    LD_FB_TIMER_TP   = 2,
    LD_FB_COUNTER_CTU= 3,
    LD_FB_COUNTER_CTD= 4,
    LD_FB_COMPARE_EQ = 5,
    LD_FB_COMPARE_GT = 6,
    LD_FB_COMPARE_LT = 7,
    LD_FB_COMPARE_GE = 8,
    LD_FB_COMPARE_LE = 9,
    LD_FB_MATH_ADD   = 10,
    LD_FB_MATH_SUB   = 11,
    LD_FB_MATH_MUL   = 12,
    LD_FB_MATH_DIV   = 13,
    LD_FB_MOVE       = 14
} ld_fb_type_t;

/* =========================================================================
 * L1: Ladder Element (contact, coil, or function block)
 * ========================================================================= */

typedef enum {
    LD_ELEM_CONTACT       = 0,
    LD_ELEM_COIL          = 1,
    LD_ELEM_HORIZ_LINK    = 2,
    LD_ELEM_VERT_LINK     = 3,
    LD_ELEM_FUNCTION_BLOCK = 4
} ld_element_type_t;

/** A single ladder element (node in the rung graph) */
typedef struct {
    ld_element_type_t elem_type;
    union {
        struct { ld_contact_type_t ctype; uint16_t ref_addr; plc_io_type_t ref_type; } contact;
        struct { ld_coil_type_t    ctype; uint16_t ref_addr; plc_io_type_t ref_type; } coil;
        struct { ld_fb_type_t      fbtype; uint16_t instance_id; } fb;
        struct { int dummy; } link;
    } data;
    int input_value;    /* Boolean input from left-side logic    */
    int output_value;   /* Boolean result propagating right      */
} ld_element_t;

/* =========================================================================
 * L2: Ladder Rung — Core Unit of LD Logic
 *
 * A rung is a series-parallel network of contacts driving a coil.
 * Power flow: left rail (TRUE) → contact network → coil → right rail.
 *
 * Graph representation: contacts form nodes, horizontal/vertical links
 * form edges. The engine evaluates connectivity from left to right.
 * ========================================================================= */

/** Maximum elements per rung */
#define LD_MAX_ELEMENTS_PER_RUNG 64

/** Ladder rung structure */
typedef struct {
    ld_element_t elements[LD_MAX_ELEMENTS_PER_RUNG];
    size_t       num_elements;
    int          rung_id;
    int          enabled;       /* Master control relay (MCR) state */
} ld_rung_t;

/** Initialize a rung */
void ld_rung_init(ld_rung_t *rung, int rung_id);

/**
 * @brief Add a normally-open contact to the rung
 *
 * Logical effect: output = input AND (ref_bit_value == 1)
 * Series connection: output passes to next element's input.
 *
 * @param ref_addr  I/O address referenced by contact
 * @param ref_type  %I, %Q, or %M
 * @return 0 on success, -1 if rung full
 */
int ld_rung_add_contact_no(ld_rung_t *rung, uint16_t ref_addr,
                           plc_io_type_t ref_type);

/** Add a normally-closed contact (output = input AND ref_bit == 0) */
int ld_rung_add_contact_nc(ld_rung_t *rung, uint16_t ref_addr,
                           plc_io_type_t ref_type);

/** Add a positive-edge contact (output = 1 for one scan on rise) */
int ld_rung_add_contact_pos(ld_rung_t *rung, uint16_t ref_addr,
                            plc_io_type_t ref_type);

/** Add a negative-edge contact */
int ld_rung_add_contact_neg(ld_rung_t *rung, uint16_t ref_addr,
                            plc_io_type_t ref_type);

/** Add a standard output coil */
int ld_rung_add_coil_normal(ld_rung_t *rung, uint16_t ref_addr,
                            plc_io_type_t ref_type);

/** Add a negated coil */
int ld_rung_add_coil_negated(ld_rung_t *rung, uint16_t ref_addr,
                             plc_io_type_t ref_type);

/** Add a SET (latch) coil */
int ld_rung_add_coil_set(ld_rung_t *rung, uint16_t ref_addr,
                         plc_io_type_t ref_type);

/** Add a RESET (unlatch) coil */
int ld_rung_add_coil_reset(ld_rung_t *rung, uint16_t ref_addr,
                           plc_io_type_t ref_type);

/** Add a horizontal link (wire connection) */
int ld_rung_add_horiz_link(ld_rung_t *rung);

/** Add a vertical link (OR branch / parallel path) */
int ld_rung_add_vert_link(ld_rung_t *rung);

/** Add a TON timer function block */
int ld_rung_add_fb_ton(ld_rung_t *rung, uint16_t timer_id,
                       double preset_ms);

/** Add a CTU counter function block */
int ld_rung_add_fb_ctu(ld_rung_t *rung, uint16_t counter_id,
                       int32_t preset);

/** Add a comparator block (==) */
int ld_rung_add_fb_compare_eq(ld_rung_t *rung, uint16_t reg_a,
                              uint16_t reg_b);

/** Add a math ADD block */
int ld_rung_add_fb_math_add(ld_rung_t *rung, uint16_t reg_src1,
                            uint16_t reg_src2, uint16_t reg_dst);

/** Add a MOVE block */
int ld_rung_add_fb_move(ld_rung_t *rung, uint16_t reg_src,
                        uint16_t reg_dst);

/* =========================================================================
 * L2: Ladder Diagram Engine
 *
 * Evaluates all rungs left-to-right, top-to-bottom each scan.
 *
 * Signal flow per rung:
 *   1. Start with power = 1 (left rail energized)
 *   2. For each element in series: power = f(element, power, plc_state)
 *   3. Coil at rung end assigns: output = power (or power-derived)
 *
 * Parallel paths (OR): multiple series branches feeding same point.
 * Implemented via vertical links that create OR junctions.
 * ========================================================================= */

/** Maximum rungs in a program */
#define LD_MAX_RUNGS 256

/** Ladder diagram program container */
typedef struct {
    ld_rung_t  rungs[LD_MAX_RUNGS];
    size_t     num_rungs;
    int        mcr_enabled;       /* Master Control Relay global  */
    plc_r_trig_t pos_edge_states[PLC_MAX_DIGITAL_IO]; /* Per-bit edge memory  */
    plc_f_trig_t neg_edge_states[PLC_MAX_DIGITAL_IO];
    plc_sr_ff_t  latch_states[PLC_MAX_DIGITAL_IO];    /* SET/RESET coil state */
} ld_program_t;

/** Initialize ladder program */
void ld_program_init(ld_program_t *prog);

/** Add a rung to the program, returns rung index or -1 if full */
int ld_program_add_rung(ld_program_t *prog, ld_rung_t *rung);

/**
 * @brief Execute one complete scan of the ladder program
 *
 * Algorithm: For each rung, evaluate element chain left-to-right.
 * Power flow propagates through contacts; final element (coil)
 * writes result to PLC I/O or internal memory.
 *
 * Rung evaluation equation:
 *   power[0] = 1 (start from left rail)
 *   power[i] = E_i(power[i-1], plc, edge_states)  for each element E_i
 *   if last element is coil: apply coil action(plc, power[last])
 *
 * @param plc      PLC system state (read/write I/O)
 * @param dt_ms    Delta time for timer updates in FBs
 * @return 0 on success, -1 on error
 */
int ld_program_scan(ld_program_t *prog, plc_system_t *plc, double dt_ms);

/**
 * @brief Evaluate a single rung and return power flow to right rail
 *
 * Used for testing individual rung logic.
 */
int ld_rung_evaluate(const ld_rung_t *rung, plc_system_t *plc,
                     ld_program_t *prog);

/* =========================================================================
 * L5: Ladder Logic Boolean Expression Compiler
 *
 * Converts a rung's contact network to Boolean expression.
 * Series → AND, parallel (vert links) → OR.
 *
 * Example rung:  -| |-I0--|/|-I1---( )-Q0
 *   → Q0 = I0 AND (NOT I1)
 *
 * Example with OR:  -| |-I0--+--( )-Q0
 *                  -| |-I1--+
 *   → Q0 = I0 OR I1
 * ========================================================================= */

/**
 * @brief Convert a rung to a Boolean expression string
 *
 * @param rung   The ladder rung
 * @param buf    Output buffer for expression string
 * @param bufsz  Buffer size
 * @return Number of characters written (excluding null)
 */
int ld_rung_to_boolean_expr(const ld_rung_t *rung, char *buf, size_t bufsz);

/**
 * @brief Validate rung: check that contacts precede coil and
 *        all references are within valid I/O range
 *
 * @param rung  The rung to validate
 * @return 1 if valid, 0 if invalid
 */
int ld_rung_validate(const ld_rung_t *rung, const plc_system_t *plc);

/* =========================================================================
 * L3: Matrix Representation of Ladder Networks
 *
 * A ladder rung can be represented as an adjacency matrix A where:
 *   A[i][j] = 1 if element j is directly connected to element i
 *
 * Power flow = transitive closure of A from left rail node.
 * This enables formal analysis (reachability, dead rung detection).
 * ========================================================================= */

/** Build adjacency matrix for a rung (caller allocates NxN int array) */
void ld_rung_adjacency_matrix(const ld_rung_t *rung, int *matrix,
                              size_t max_dim);

/** Detect dead rungs (no path from left rail to any coil) */
int ld_rung_is_dead(const ld_rung_t *rung);

/** Compute topological sort order for rung elements */
int ld_rung_topological_sort(const ld_rung_t *rung, int *order,
                             size_t max_order);

#ifdef __cplusplus
}
#endif

#endif /* PLC_LADDER_H */