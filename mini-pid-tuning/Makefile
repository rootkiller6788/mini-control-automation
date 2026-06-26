# Makefile ? mini-pid-tuning
# PID Controller Tuning Library
#
# Targets:
#   make          ? build library (libpid.a) and examples
#   make test     ? build and run tests
#   make examples ? build all examples
#   make clean    ? remove build artifacts
#   make all      ? build everything (lib + tests + examples)

CC       := gcc
CFLAGS   := -std=c11 -Wall -Wextra -O2 -g \
            -Iinclude \
            -Wno-unused-parameter -Wno-sign-compare
LDFLAGS  := -lm

AR       := ar
ARFLAGS  := rcs

# Source files
SRC_CORE        := src/pid_core.c
SRC_TUNING      := src/pid_tuning.c
SRC_ANALYSIS    := src/pid_analysis.c
SRC_ADVANCED    := src/pid_advanced.c
SRC_APPLICATIONS := src/pid_applications.c

SRCS := $(SRC_CORE) $(SRC_TUNING) $(SRC_ANALYSIS) $(SRC_ADVANCED) $(SRC_APPLICATIONS)
OBJS := $(SRCS:.c=.o)

# Library
LIB := libpid.a

# Test
TEST_SRC := tests/test_pid.c
TEST_BIN := test_pid

# Examples
EXAMPLE_SRCS := examples/example_dc_motor.c \
                examples/example_temperature.c \
                examples/example_quadrotor.c
EXAMPLE_BINS := $(patsubst examples/%.c,%,$(EXAMPLE_SRCS))

# Default target
.PHONY: all
all: $(LIB) examples

# Build static library
$(LIB): $(OBJS)
	@echo "  AR    $@"
	@$(AR) $(ARFLAGS) $@ $^

# Compile C sources
%.o: %.c
	@echo "  CC    $<"
	@$(CC) $(CFLAGS) -c -o $@ $<

# Build and run tests
.PHONY: test
test: $(TEST_BIN)
	@echo ""
	./$(TEST_BIN)
	@echo ""

$(TEST_BIN): $(LIB) $(TEST_SRC)
	@echo "  CC    $@"
	@$(CC) $(CFLAGS) -o $@ $(TEST_SRC) -L. -lpid $(LDFLAGS)

# Build examples
.PHONY: examples
examples: $(EXAMPLE_BINS)

example_%: examples/example_%.c $(LIB)
	@echo "  CC    $@"
	@$(CC) $(CFLAGS) -o $@ $< -L. -lpid $(LDFLAGS)

# Run a specific example
example_dc: example_dc_motor
	./example_dc_motor

example_temp: example_temperature
	./example_temperature

example_quad: example_quadrotor
	./example_quadrotor

.PHONY: run_all_examples
run_all_examples: $(EXAMPLE_BINS)
	@echo "\n=== Running All Examples ===\n"
	./example_dc_motor
	./example_temperature
	./example_quadrotor
	@echo "\n=== All Examples Complete ===\n"

# Clean
.PHONY: clean
clean:
	@echo "  CLEAN"
	@rm -f $(OBJS) $(LIB) $(TEST_BIN) $(EXAMPLE_BINS)
	@rm -f tests/*.o examples/*.o
	@echo "  Done."

# Help
.PHONY: help
help:
	@echo "mini-pid-tuning Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make             ? build library and examples"
	@echo "  make test        ? build and run test suite"
	@echo "  make examples    ? build all examples"
	@echo "  make example_dc  ? build and run DC motor example"
	@echo "  make example_temp? build and run temperature example"
	@echo "  make example_quad? build and run quadrotor example"
	@echo "  make run_all_examples ? run all examples"
	@echo "  make clean       ? remove build artifacts"
	@echo "  make help        ? show this help"
