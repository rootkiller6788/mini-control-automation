# Makefile for mini-robot-control
# Robot Control Library -- Kinematics, Dynamics, Control, Trajectory
#
# References:
#   Craig (2018), Siciliano et al. (2010), Spong et al. (2006)
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
CORE_SRC   := src/robot_core.c
MATH_SRC   := src/robot_math3d.c
KIN_SRC    := src/robot_kinematics.c
DYN_SRC    := src/robot_dynamics.c
CTRL_SRC   := src/robot_control.c
TRAJ_SRC   := src/robot_trajectory.c

ALL_SRC := $(CORE_SRC) $(MATH_SRC) $(KIN_SRC) $(DYN_SRC) $(CTRL_SRC) $(TRAJ_SRC)
ALL_OBJ := $(ALL_SRC:.c=.o)

# Test
TEST_SRC := tests/test_robot_control.c
TEST_BIN := test_robot_control

# Examples
EX1_SRC := examples/ex1_planar_2dof_control.c
EX2_SRC := examples/ex2_scara_pick_place.c
EX3_SRC := examples/ex3_mobile_robot.c
EX1_BIN := ex1_planar_2dof_control
EX2_BIN := ex2_scara_pick_place
EX3_BIN := ex3_mobile_robot

.PHONY: all test examples clean

all: $(ALL_OBJ) $(TEST_BIN)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_BIN): $(ALL_OBJ) $(TEST_SRC)
	$(CC) $(CFLAGS) $(TEST_SRC) $(ALL_OBJ) -o $(TEST_BIN) $(LDFLAGS)

examples: $(EX1_BIN) $(EX2_BIN) $(EX3_BIN)

$(EX1_BIN): $(ALL_OBJ) $(EX1_SRC)
	$(CC) $(CFLAGS) $(EX1_SRC) $(ALL_OBJ) -o $(EX1_BIN) $(LDFLAGS)

$(EX2_BIN): $(ALL_OBJ) $(EX2_SRC)
	$(CC) $(CFLAGS) $(EX2_SRC) $(ALL_OBJ) -o $(EX2_BIN) $(LDFLAGS)

$(EX3_BIN): $(ALL_OBJ) $(EX3_SRC)
	$(CC) $(CFLAGS) $(EX3_SRC) $(ALL_OBJ) -o $(EX3_BIN) $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f src/*.o $(TEST_BIN) $(EX1_BIN) $(EX2_BIN) $(EX3_BIN) *.exe