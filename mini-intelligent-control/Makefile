# Makefile for mini-intelligent-control
# Intelligent Control Theory Library
# Reference: Astrom & Wittenmark (2013), Slotine & Li (1991), Sutton & Barto (2018)

CC      := gcc
CFLAGS  := -Wall -Wextra -std=c11 -O2 -I./include
LDFLAGS := -lm

SRC_FILES := src/intelligent_control_core.c \
             src/fuzzy_control.c \
             src/neural_control.c \
             src/adaptive_control.c \
             src/mpc_control.c \
             src/reinforcement_control.c \
             src/sliding_mode_control.c \
             src/genetic_algorithm.c

OBJ_FILES := $(SRC_FILES:.c=.o)

TEST_SRC := tests/test_intelligent_control.c
TEST_BIN := test_intelligent_control

EX1_SRC := examples/example_fuzzy_pendulum.c
EX2_SRC := examples/example_neural_motor.c
EX3_SRC := examples/example_rl_cartpole.c
EX1_BIN := example_fuzzy_pendulum
EX2_BIN := example_neural_motor
EX3_BIN := example_rl_cartpole

.PHONY: all test examples clean

all: $(OBJ_FILES) $(TEST_BIN)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_BIN): $(OBJ_FILES) $(TEST_SRC)
	$(CC) $(CFLAGS) $(TEST_SRC) $(OBJ_FILES) -o $(TEST_BIN) $(LDFLAGS)

examples: $(EX1_BIN) $(EX2_BIN) $(EX3_BIN)

$(EX1_BIN): $(OBJ_FILES) $(EX1_SRC)
	$(CC) $(CFLAGS) $(EX1_SRC) $(OBJ_FILES) -o $(EX1_BIN) $(LDFLAGS)

$(EX2_BIN): $(OBJ_FILES) $(EX2_SRC)
	$(CC) $(CFLAGS) $(EX2_SRC) $(OBJ_FILES) -o $(EX2_BIN) $(LDFLAGS)

$(EX3_BIN): $(OBJ_FILES) $(EX3_SRC)
	$(CC) $(CFLAGS) $(EX3_SRC) $(OBJ_FILES) -o $(EX3_BIN) $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(OBJ_FILES) $(TEST_BIN) $(EX1_BIN) $(EX2_BIN) $(EX3_BIN)
	rm -f src/*.o tests/*.o
