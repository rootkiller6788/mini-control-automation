#ifndef PLC_FBD_H
#define PLC_FBD_H
#include "plc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Function Block Diagram (FBD) - IEC 61131-3
 *
 * FBD expresses logic as a network of interconnected function blocks.
 * Signals flow from left to right through block inputs/outputs.
 * Each block encapsulates a function (AND, OR, ADD, PID, etc.).
 *
 * Mathematical model: Directed Acyclic Graph (DAG)
 *   Nodes = function block instances
 *   Edges = signal connections (output→input)
 * ========================================================================= */

/** Standard FBD function block types */
typedef enum {
    FBD_AND       = 0,   /* Boolean AND:  Q = I1 & I2 & ... & In */
    FBD_OR        = 1,   /* Boolean OR:   Q = I1 | I2 | ... | In */
    FBD_XOR       = 2,   /* Boolean XOR:  Q = I1 ^ I2           */
    FBD_NOT       = 3,   /* Boolean NOT:  Q = !I1               */
    FBD_NAND      = 4,   /* NAND gate                           */
    FBD_NOR       = 5,   /* NOR gate                            */
    FBD_RS        = 6,   /* Reset-Set flip-flop                 */
    FBD_SR        = 7,   /* Set-Reset flip-flop                 */
    FBD_R_TRIG    = 8,   /* Rising edge detector                */
    FBD_F_TRIG    = 9,   /* Falling edge detector               */
    FBD_TON       = 10,  /* Timer ON Delay                      */
    FBD_TOF       = 11,  /* Timer OFF Delay                     */
    FBD_TP        = 12,  /* Pulse timer                         */
    FBD_CTU       = 13,  /* Up counter                          */
    FBD_CTD       = 14,  /* Down counter                        */
    FBD_CTUD      = 15,  /* Up/Down counter                     */
    FBD_ADD       = 16,  /* Arithmetic ADD:  Q = I1 + I2        */
    FBD_SUB       = 17,  /* Arithmetic SUB:  Q = I1 - I2        */
    FBD_MUL       = 18,  /* Arithmetic MUL:  Q = I1 * I2        */
    FBD_DIV       = 19,  /* Arithmetic DIV:  Q = I1 / I2        */
    FBD_MOD       = 20,  /* Modulo            Q = I1 % I2       */
    FBD_ABS       = 21,  /* Absolute value    Q = |I1|           */
    FBD_SQRT      = 22,  /* Square root       Q = sqrt(I1)       */
    FBD_EXP       = 23,  /* Exponential       Q = exp(I1)        */
    FBD_LN        = 24,  /* Natural log       Q = ln(I1)         */
    FBD_SIN       = 25,  /* Sine              Q = sin(I1)        */
    FBD_COS       = 26,  /* Cosine            Q = cos(I1)        */
    FBD_EQ        = 27,  /* Equal:            Q = (I1 == I2)     */
    FBD_GT        = 28,  /* Greater than:     Q = (I1 > I2)      */
    FBD_LT        = 29,  /* Less than:        Q = (I1 < I2)      */
    FBD_GE        = 30,  /* Greater or equal: Q = (I1 >= I2)     */
    FBD_LE        = 31,  /* Less or equal:    Q = (I1 <= I2)     */
    FBD_NE        = 32,  /* Not equal:        Q = (I1 != I2)     */
    FBD_SEL       = 33,  /* Selector:        Q = G ? I1 : I2    */
    FBD_MUX       = 34,  /* Multiplexer:     Q = I[K]            */
    FBD_LIMIT     = 35,  /* Limiter:         Q = clamp(I, MN,MX)*/
    FBD_HYSTERESIS= 36,  /* Hysteresis comparator               */
    FBD_MOVE      = 37,  /* Value assignment: OUT = IN          */
    FBD_PID       = 38,  /* PID controller block                */
    FBD_INTEGRAL  = 39,  /* Integrator                          */
    FBD_DERIV     = 40   /* Differentiator                      */
} fbd_block_type_t;

/* =========================================================================
 * L1: FBD Connector Types (data flow interface)
 * ========================================================================= */

/** Data type carried on FBD signal wires */
typedef enum {
    FBD_SIG_BOOL  = 0,
    FBD_SIG_INT   = 1,
    FBD_SIG_REAL  = 2,
    FBD_SIG_TIME  = 3
} fbd_signal_type_t;

/** A single input pin on a function block */
typedef struct {
    char     name[16];           /* Pin label (e.g., "IN", "PV")    */
    fbd_signal_type_t sig_type;  /* Data type expected              */
    union {
        int      b;              /* Boolean constant value          */
        int32_t  i;              /* Integer constant value          */
        double   r;              /* Real constant value             */
    } default_value;
    int      is_connected;       /* True if driven by another block */
    int      source_block_id;    /* Block driving this pin (-1=none)*/
    int      source_pin_idx;     /* Output pin index on source      */
} fbd_input_pin_t;

/** A single output pin on a function block */
typedef struct {
    char              name[16];
    fbd_signal_type_t sig_type;
    union {
        int      b;
        int32_t  i;
        double   r;
    } value;                      /* Computed output value           */
} fbd_output_pin_t;

/* =========================================================================
 * L1: Function Block Instance
 * ========================================================================= */

#define FBD_MAX_INPUTS  8
#define FBD_MAX_OUTPUTS 4

/**
 * @brief A single function block instance in the diagram
 *
 * Each block implements a specific mathematical function.
 * Execution order determined by topological sort of the DAG.
 *
 * Block semantics:
 *   outputs = f(inputs, internal_state, parameters)
 *
 * where f is the function identified by block_type.
 */
typedef struct {
    int               block_id;
    fbd_block_type_t  block_type;
    fbd_input_pin_t   inputs[FBD_MAX_INPUTS];
    fbd_output_pin_t  outputs[FBD_MAX_OUTPUTS];
    int               num_inputs;
    int               num_outputs;
    /* Internal state for stateful blocks (timers, integrators) */
    double            state_vars[4];
    int               executed_this_scan;  /* Prevent double-execution */
} fbd_block_t;

/* =========================================================================
 * L1: FBD Network — the complete diagram
 * ========================================================================= */

#define FBD_MAX_BLOCKS 128

/**
 * @brief Complete Function Block Diagram
 *
 * Topological execution order:
 *   1. Identify source blocks (only constant inputs)
 *   2. Topological sort (Kahn's algorithm on DAG)
 *   3. Execute blocks in sorted order
 *
 * Cycle detection: if a cycle exists, the diagram is invalid
 * (feedback must go through an explicit delay/state block).
 */
typedef struct {
    fbd_block_t blocks[FBD_MAX_BLOCKS];
    size_t      num_blocks;
    int         exec_order[FBD_MAX_BLOCKS];
    size_t      exec_order_len;
    int         sorted;           /* 1 if topological sort valid */
    int         has_cycle;        /* 1 if cyclic dependency found */
} fbd_network_t;

/* =========================================================================
 * L2: FBD Network API
 * ========================================================================= */

/** Initialize empty FBD network */
void fbd_network_init(fbd_network_t *net);

/**
 * @brief Add a function block to the network
 *
 * @param block_type  The function this block performs
 * @param num_inputs  Number of input pins to allocate
 * @param num_outputs Number of output pins to allocate
 * @param name        Block instance name
 * @return block_id (≥0) on success, -1 if network full
 */
int fbd_add_block(fbd_network_t *net, fbd_block_type_t block_type,
                  int num_inputs, int num_outputs, const char *name);

/**
 * @brief Connect an output pin to an input pin
 *
 * Creates a data flow edge in the DAG.
 *
 * @param src_block  Source block ID
 * @param src_pin    Output pin index on source
 * @param dst_block  Destination block ID
 * @param dst_pin    Input pin index on destination
 * @return 0 on success, -1 on invalid connection
 */
int fbd_connect(fbd_network_t *net, int src_block, int src_pin,
                int dst_block, int dst_pin);

/**
 * @brief Set a constant value on an input pin
 *
 * Overrides default value. Only valid for unconnected pins.
 */
int fbd_set_constant_bool(fbd_network_t *net, int block_id,
                          int pin_idx, int value);
int fbd_set_constant_int(fbd_network_t *net, int block_id,
                         int pin_idx, int32_t value);
int fbd_set_constant_real(fbd_network_t *net, int block_id,
                          int pin_idx, double value);

/**
 * @brief Topological sort: compute execution order
 *
 * Uses Kahn's algorithm:
 *   1. Compute in-degree for each block (connected input count)
 *   2. Queue blocks with in-degree 0
 *   3. Dequeue, append to order, decrement successors' in-degrees
 *   4. Enqueue successors that reach in-degree 0
 *   5. If processed < total blocks, cycle exists → mark has_cycle=1
 *
 * Complexity: O(V + E)
 *
 * @return 0 on success, -1 if cycle detected
 */
int fbd_topological_sort(fbd_network_t *net);

/**
 * @brief Execute one complete scan of the FBD network
 *
 * Blocks are executed in topological order. Each block reads
 * its inputs (from preceding blocks' outputs or constants),
 * computes its function, and writes outputs.
 *
 * Boolean logic blocks implement:
 *   AND:  Q = I1 & I2 & ... & In
 *   OR:   Q = I1 | I2 | ... | In
 *   XOR:  Q = I1 ^ I2
 *   NOT:  Q = !I1
 *
 * Arithmetic blocks (L3 Math Structures):
 *   ADD:  Q = I1 + I2
 *   SUB:  Q = I1 - I2
 *   MUL:  Q = I1 * I2
 *   DIV:  Q = I1 / I2  (division by zero → fault)
 *
 * @param plc     PLC system for I/O and timer access
 * @param dt_ms   Delta time for time-based blocks
 * @return 0 on success, -1 on execution error
 */
int fbd_network_scan(fbd_network_t *net, plc_system_t *plc, double dt_ms);

/**
 * @brief Execute a single function block
 *
 * Implements the block's mathematical function.
 *
 * @param block   The block to execute
 * @param plc     PLC system reference
 * @param dt_ms   Time delta for stateful blocks
 * @return 0 on success, -1 on error (e.g., div by zero)
 */
int fbd_execute_block(fbd_block_t *block, plc_system_t *plc, double dt_ms);

/**
 * @brief Read an output pin value (for connecting to I/O)
 *
 * After scan, output pins hold computed values.
 */
int fbd_read_output_bool(const fbd_network_t *net, int block_id,
                         int pin_idx, int *val);
int fbd_read_output_int(const fbd_network_t *net, int block_id,
                        int pin_idx, int32_t *val);
int fbd_read_output_real(const fbd_network_t *net, int block_id,
                         int pin_idx, double *val);

/**
 * @brief Wire a block input pin to a PLC I/O address
 *
 * During scan, reads PLC I/O value into input pin before block execution.
 */
int fbd_bind_input_to_plc(fbd_network_t *net, int block_id, int pin_idx,
                          plc_io_type_t io_type, uint16_t io_addr);

/**
 * @brief Wire a block output pin to a PLC I/O address
 *
 * After block execution, writes block output to PLC I/O.
 */
int fbd_bind_output_to_plc(fbd_network_t *net, int block_id, int pin_idx,
                           plc_io_type_t io_type, uint16_t io_addr);

/* =========================================================================
 * L3: DAG Analysis Utilities
 * ========================================================================= */

/** Detect cycles in the FBD network (DFS-based) */
int fbd_detect_cycle(const fbd_network_t *net);

/** Compute critical path length (longest path from any source to sink) */
int fbd_critical_path_length(const fbd_network_t *net);

/** Get block execution order (valid after topological sort) */
const int* fbd_get_execution_order(const fbd_network_t *net, size_t *len);

/** Print the network as a DOT graph (for visualization) */
void fbd_export_dot(const fbd_network_t *net, char *buf, size_t bufsz);

#ifdef __cplusplus
}
#endif

#endif /* PLC_FBD_H */