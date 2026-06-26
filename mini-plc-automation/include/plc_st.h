#ifndef PLC_ST_H
#define PLC_ST_H
#include "plc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Structured Text (ST) - IEC 61131-3
 *
 * ST is a high-level procedural language similar to Pascal.
 * Used for complex algorithms that are cumbersome in LD or FBD.
 *
 * Core constructs:
 *   IF/THEN/ELSIF/ELSE/END_IF  — conditional execution
 *   CASE/OF/ELSE/END_CASE      — multi-way branch
 *   FOR/TO/BY/DO/END_FOR       — counted loop
 *   WHILE/DO/END_WHILE         — pre-test loop
 *   REPEAT/UNTIL/END_REPEAT    — post-test loop
 *   RETURN                     — early exit from function
 * ========================================================================= */

/** ST token types (lexer) */
typedef enum {
    ST_TOK_EOF = 0,   ST_TOK_IDENT,     ST_TOK_NUMBER,
    ST_TOK_STRING,    ST_TOK_PLUS,      ST_TOK_MINUS,
    ST_TOK_STAR,      ST_TOK_SLASH,     ST_TOK_MOD,
    ST_TOK_EQ,        ST_TOK_NE,        ST_TOK_LT,
    ST_TOK_GT,        ST_TOK_LE,        ST_TOK_GE,
    ST_TOK_ASSIGN,    ST_TOK_LPAREN,    ST_TOK_RPAREN,
    ST_TOK_LBRACKET,  ST_TOK_RBRACKET,  ST_TOK_SEMI,
    ST_TOK_COLON,     ST_TOK_COMMA,     ST_TOK_DOT,
    ST_TOK_AND,       ST_TOK_OR,        ST_TOK_XOR,
    ST_TOK_NOT,       ST_TOK_IF,        ST_TOK_THEN,
    ST_TOK_ELSIF,     ST_TOK_ELSE,      ST_TOK_END_IF,
    ST_TOK_CASE,      ST_TOK_OF,        ST_TOK_END_CASE,
    ST_TOK_FOR,       ST_TOK_TO,        ST_TOK_BY,
    ST_TOK_DO,        ST_TOK_END_FOR,   ST_TOK_WHILE,
    ST_TOK_END_WHILE, ST_TOK_REPEAT,    ST_TOK_UNTIL,
    ST_TOK_END_REPEAT,ST_TOK_RETURN,    ST_TOK_TRUE,
    ST_TOK_FALSE,     ST_TOK_EXIT,      ST_TOK_CONTINUE
} st_token_type_t;

/** A single lexical token */
typedef struct {
    st_token_type_t type;
    char            lexeme[64];  /* String representation           */
    double          num_value;   /* Value if ST_TOK_NUMBER          */
    int             line;        /* Source line number              */
    int             col;         /* Source column                   */
} st_token_t;

/* =========================================================================
 * L1: ST AST Node Types
 * ========================================================================= */

typedef enum {
    ST_AST_PROGRAM     = 0,  /* Root: list of statements           */
    ST_AST_ASSIGN      = 1,  /* variable := expression             */
    ST_AST_IF          = 2,  /* IF cond THEN ... END_IF            */
    ST_AST_CASE        = 3,  /* CASE expr OF ... END_CASE          */
    ST_AST_FOR         = 4,  /* FOR var := init TO final DO ...    */
    ST_AST_WHILE       = 5,  /* WHILE cond DO ... END_WHILE        */
    ST_AST_REPEAT      = 6,  /* REPEAT ... UNTIL cond END_REPEAT  */
    ST_AST_RETURN      = 7,  /* RETURN                             */
    ST_AST_EXPR_BINARY = 8,  /* Binary operation: a op b           */
    ST_AST_EXPR_UNARY  = 9,  /* Unary operation: op a              */
    ST_AST_EXPR_CALL   = 10, /* Function call: name(args)          */
    ST_AST_LITERAL_BOOL= 11, /* TRUE or FALSE                      */
    ST_AST_LITERAL_INT = 12, /* Integer literal                    */
    ST_AST_LITERAL_REAL= 13, /* Real literal                       */
    ST_AST_VARIABLE    = 14, /* Variable reference (%I0, %MW10...) */
    ST_AST_STMT_LIST   = 15  /* List of statements {stmt; stmt;}   */
} st_ast_node_type_t;

/** Variable reference descriptor */
typedef struct {
    plc_io_type_t io_type;        /* %I, %Q, %M, %MW, %T, %C      */
    uint16_t      address;        /* Numeric address               */
    int           is_constant;    /* Read-only variable            */
} st_variable_t;

/** Binary operator types */
typedef enum {
    ST_OP_ADD, ST_OP_SUB, ST_OP_MUL, ST_OP_DIV, ST_OP_MOD,
    ST_OP_AND, ST_OP_OR,  ST_OP_XOR,
    ST_OP_EQ,  ST_OP_NE,  ST_OP_LT,  ST_OP_GT, ST_OP_LE, ST_OP_GE
} st_binary_op_t;

/** Unary operator types */
typedef enum {
    ST_OP_NEG,   /* Arithmetic negation */
    ST_OP_NOT    /* Boolean complement  */
} st_unary_op_t;

/** Value union for ST evaluation */
typedef struct {
    enum { ST_VAL_BOOL, ST_VAL_INT, ST_VAL_REAL } type;
    union { int b; int32_t i; double r; } data;
} st_value_t;

/* =========================================================================
 * L2: ST AST Node
 * ========================================================================= */

#define ST_MAX_CHILDREN 4

typedef struct st_ast_node st_ast_node_t;

struct st_ast_node {
    st_ast_node_type_t node_type;
    union {
        struct { st_variable_t var; st_ast_node_t *expr; } assign;
        struct { st_ast_node_t *cond; st_ast_node_t *then_stmts;
                 st_ast_node_t *else_stmts; } if_stmt;
        struct { st_ast_node_t *expr; st_ast_node_t *cases; } case_stmt;
        struct { st_variable_t var; st_ast_node_t *init; st_ast_node_t *final;
                 st_ast_node_t *by; st_ast_node_t *body; } for_stmt;
        struct { st_ast_node_t *cond; st_ast_node_t *body; } while_stmt;
        struct { st_ast_node_t *body; st_ast_node_t *cond; } repeat_stmt;
        struct { st_binary_op_t op; st_ast_node_t *left; st_ast_node_t *right; } binary;
        struct { st_unary_op_t op; st_ast_node_t *operand; } unary;
        struct { char name[32]; st_ast_node_t *args; } call;
        struct { st_variable_t var; } variable;
        struct { st_value_t val; } literal;
        struct { int count; st_ast_node_t *items[ST_MAX_CHILDREN]; } stmt_list;
    } data;
    st_ast_node_t *next;  /* Sibling in statement list */
};

/* =========================================================================
 * L2: ST Program — Compiled representation
 * ========================================================================= */

#define ST_MAX_TOKENS    512
#define ST_MAX_VARIABLES 128

/** Compiled ST program (bytecode-like flat representation) */
typedef struct {
    st_ast_node_t   *ast_root;       /* Abstract syntax tree root  */
    st_token_t       tokens[ST_MAX_TOKENS];
    int              num_tokens;
    char            *source;         /* Original source code      */
    int              has_errors;
    char             error_msg[256];
    /* Symbol table: maps variable names to addresses */
    struct {
        char          name[32];
        st_variable_t var;
    } symbol_table[ST_MAX_VARIABLES];
    int              num_symbols;
} st_program_t;

/* =========================================================================
 * L5: ST Lexer
 * ========================================================================= */

/**
 * @brief Tokenize ST source code
 *
 * Lexer recognizes IEC 61131-3 keywords (case-insensitive),
 * operators, numbers (integer and real), string literals,
 * and identifiers (variable names, function names).
 *
 * Number format: [0-9]+ (integer) or [0-9]+.[0-9]+ (real)
 * Identifier: [a-zA-Z_][a-zA-Z0-9_]*
 * Comment: (* ... *)  (nestable per IEC 61131-3)
 *
 * @param source  Null-terminated ST source code
 * @param prog    Program struct to hold tokens
 * @return 0 on success, -1 on lexical error
 */
int st_lex(const char *source, st_program_t *prog);

/* =========================================================================
 * L5: ST Parser — Recursive Descent
 * ========================================================================= */

/**
 * @brief Parse token stream into AST
 *
 * Grammar (simplified):
 *   program       := { statement ";" }
 *   statement     := assignment | if_stmt | case_stmt | for_stmt
 *                  | while_stmt | repeat_stmt | return_stmt
 *   assignment    := variable ":=" expression
 *   if_stmt       := "IF" expression "THEN" statements
 *                    { "ELSIF" expression "THEN" statements }
 *                    [ "ELSE" statements ] "END_IF"
 *   for_stmt      := "FOR" variable ":=" expression "TO" expression
 *                    [ "BY" expression ] "DO" statements "END_FOR"
 *   expression    := and_expr { "OR" | "XOR" and_expr }
 *   and_expr      := compare_expr { "AND" compare_expr }
 *   compare_expr  := add_expr [ ("=" | "<>" | "<" | ">" | "<=" | ">=") add_expr ]
 *   add_expr      := mul_expr { ("+" | "-") mul_expr }
 *   mul_expr      := unary_expr { ("*" | "/" | "MOD") unary_expr }
 *   unary_expr    := [ "-" | "NOT" ] primary
 *   primary       := number | variable | "TRUE" | "FALSE" | "(" expression ")"
 *
 * @return 0 on success, -1 on parse error
 */
int st_parse(st_program_t *prog);

/* =========================================================================
 * L5: ST Interpreter
 * ========================================================================= */

/**
 * @brief Interpret (execute) the compiled ST program
 *
 * Walks the AST and evaluates each statement against PLC state.
 * Implements short-circuit evaluation for AND/OR.
 *
 * Execution is performed each PLC scan cycle.
 *
 * @param prog   Compiled ST program
 * @param plc    PLC system for I/O read/write
 * @param dt_ms  Scan cycle time (for timer operations)
 * @return 0 on success, -1 on runtime error
 */
int st_execute(const st_program_t *prog, plc_system_t *plc, double dt_ms);

/**
 * @brief Evaluate a single expression node against PLC state
 *
 * Arithmetic: standard C semantics with SATURATION on overflow
 *             (not wrap-around, per IEC 61131-3).
 * Boolean:    0 = FALSE, non-zero = TRUE.
 * Division:   Division by zero → PLC fault state.
 *
 * @param node  AST expression node
 * @param plc   PLC system for variable lookup
 * @param result Output value
 * @return 0 on success, -1 on error
 */
int st_eval_expr(const st_ast_node_t *node, const plc_system_t *plc,
                 st_value_t *result);

/**
 * @brief Free all memory associated with ST program
 */
void st_program_free(st_program_t *prog);

/**
 * @brief Register a variable in the symbol table
 *
 * Maps a variable name (e.g., "motor_start", "tank_level") to
 * a PLC I/O address internally.
 *
 * @param name     Variable name in ST source
 * @param io_type  I/O type (%I, %Q, %M, %MW, etc.)
 * @param addr     Numeric address
 * @return 0 on success, -1 if symbol table full
 */
int st_register_variable(st_program_t *prog, const char *name,
                         plc_io_type_t io_type, uint16_t addr);

/**
 * @brief Look up a variable in the symbol table
 *
 * @param name  Variable name
 * @param var   Output: filled if found
 * @return 0 if found, -1 if not found
 */
int st_lookup_variable(const st_program_t *prog, const char *name,
                       st_variable_t *var);

/* =========================================================================
 * L3: Short-circuit Boolean Evaluation
 *
 * IEC 61131-3 requires short-circuit evaluation:
 *   IF (a AND b) THEN ...  — if a is FALSE, b is not evaluated
 *   IF (a OR b)  THEN ...  — if a is TRUE,  b is not evaluated
 *
 * This prevents division-by-zero in expressions like:
 *   IF (denom <> 0) AND (num/denom > 10) THEN ...
 * ========================================================================= */

/**
 * @brief Short-circuit AND evaluation
 *
 * @param left   Pre-evaluated left operand
 * @param right  AST node for right operand (evaluated only if left is TRUE)
 * @param plc    PLC state
 * @param result Output: left AND right
 */
int st_short_circuit_and(st_value_t left, const st_ast_node_t *right,
                         const plc_system_t *plc, st_value_t *result);

/**
 * @brief Short-circuit OR evaluation
 */
int st_short_circuit_or(st_value_t left, const st_ast_node_t *right,
                        const plc_system_t *plc, st_value_t *result);

/* =========================================================================
 * L6: Built-in Standard Functions (IEC 61131-3)
 * ========================================================================= */

/* Math: ABS, SQRT, LN, LOG, EXP, SIN, COS, TAN, ASIN, ACOS, ATAN */
int st_builtin_abs(const st_value_t *in, st_value_t *out);
int st_builtin_sqrt(const st_value_t *in, st_value_t *out);
int st_builtin_ln(const st_value_t *in, st_value_t *out);
int st_builtin_exp(const st_value_t *in, st_value_t *out);
int st_builtin_sin(const st_value_t *in, st_value_t *out);
int st_builtin_cos(const st_value_t *in, st_value_t *out);

/* Selection: SEL(G, IN0, IN1) = G ? IN0 : IN1 */
int st_builtin_sel(int g, const st_value_t *in0, const st_value_t *in1,
                   st_value_t *out);

/* Limiter: LIMIT(MN, IN, MX) = clamp(IN, MN, MX) */
int st_builtin_limit(const st_value_t *mn, const st_value_t *in,
                     const st_value_t *mx, st_value_t *out);

#ifdef __cplusplus
}
#endif

#endif /* PLC_ST_H */