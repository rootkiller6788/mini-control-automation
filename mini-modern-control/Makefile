# Makefile for mini-modern-control
# Modern Control Theory Library
#
# References: Ogata (2010), Chen (2013), Kalman (1960-1963)
#
# Targets:
#   make          - build library and tests
#   make test     - build and run tests
#   make examples - build all examples
#   make clean    - remove build artifacts

CC      := gcc
CFLAGS  := -Wall -Wextra -std=c11 -O2 -I./include
LDFLAGS := -lm

# Source files
SRC := src/matrix_operations.c \
       src/state_space.c \
       src/controllability.c \
       src/pole_placement.c \
       src/eigenvalues.c \
       src/lqr_solver.c \
       src/lyapunov_solver.c \
       src/observer_design.c \
       src/kalman_filter.c \
       src/canonical_problems.c

OBJ := $(SRC:.c=.o)

# Test
TEST_SRC := tests/test_modern_control.c
TEST_BIN := test_modern_control

# Examples
EX1_SRC := examples/example_inverted_pendulum.c
EX2_SRC := examples/example_dc_motor_lqr.c
EX3_SRC := examples/example_kalman_observer.c
EX1_BIN := example_inverted_pendulum
EX2_BIN := example_dc_motor_lqr
EX3_BIN := example_kalman_observer

.PHONY: all test examples clean

all: $(OBJ) $(TEST_BIN)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_BIN): $(OBJ) $(TEST_SRC)
	$(CC) $(CFLAGS) $(TEST_SRC) $(OBJ) -o $(TEST_BIN) $(LDFLAGS)

examples: $(EX1_BIN) $(EX2_BIN) $(EX3_BIN)

$(EX1_BIN): $(OBJ) $(EX1_SRC)
	$(CC) $(CFLAGS) $(EX1_SRC) $(OBJ) -o $(EX1_BIN) $(LDFLAGS)

$(EX2_BIN): $(OBJ) $(EX2_SRC)
	$(CC) $(CFLAGS) $(EX2_SRC) $(OBJ) -o $(EX2_BIN) $(LDFLAGS)

$(EX3_BIN): $(OBJ) $(EX3_SRC)
	$(CC) $(CFLAGS) $(EX3_SRC) $(OBJ) -o $(EX3_BIN) $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(OBJ) $(TEST_BIN) $(EX1_BIN) $(EX2_BIN) $(EX3_BIN)
	rm -f src/*.o tests/*.o
