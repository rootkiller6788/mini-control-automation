CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -g -Iinclude
LDFLAGS = -lm

LIB_SRCS = src/control_core.c src/control_analysis.c src/control_design.c \
           src/control_rootlocus.c src/control_applications.c
LIB_OBJS = $(LIB_SRCS:.c=.o)

TEST_SRC = tests/test_control.c
TEST_BIN = tests/test_control

EX_SRC1 = examples/example_dc_motor.c
EX_SRC2 = examples/example_pid_tuning.c
EX_SRC3 = examples/example_root_locus.c
EX_BIN1 = examples/example_dc_motor
EX_BIN2 = examples/example_pid_tuning
EX_BIN3 = examples/example_root_locus

.PHONY: all clean test examples

all: test examples

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_BIN): $(TEST_SRC) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

$(EX_BIN1): $(EX_SRC1) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(EX_BIN2): $(EX_SRC2) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(EX_BIN3): $(EX_SRC3) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

examples: $(EX_BIN1) $(EX_BIN2) $(EX_BIN3)

clean:
	rm -f src/*.o $(TEST_BIN) $(EX_BIN1) $(EX_BIN2) $(EX_BIN3)
