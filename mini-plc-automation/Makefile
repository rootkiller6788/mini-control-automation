CC      := gcc
CFLAGS  := -Wall -Wextra -std=c11 -O2 -I./include
LDFLAGS := -lm

CORE_SRC := src/plc_core5_scan_cycle.c src/plc_core1_io.c src/plc_core2_timer.c src/plc_core3_counter.c
EDGE_SRC := src/plc_core4_edge_sfc.c src/plc_core6_timing.c src/plc_core6_pid.c
FILTER_SRC := src/plc_filter.c src/plc_modbus.c src/plc_redundancy.c
LADDER_SRC := src/plc_ladder1.c src/plc_ladder2.c
ALL_SRC := $(CORE_SRC) $(EDGE_SRC) $(FILTER_SRC) $(LADDER_SRC)
ALL_OBJ := $(ALL_SRC:.c=.o)
TEST_SRC := tests/test_plc.c
TEST_BIN := test_plc
EX1_SRC := examples/example_conveyor.c
EX2_SRC := examples/example_traffic_light.c
EX3_SRC := examples/example_tank_level.c
EX1_BIN := example_conveyor
EX2_BIN := example_traffic_light
EX3_BIN := example_tank_level

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
	rm -f $(ALL_OBJ) $(TEST_BIN) $(EX1_BIN) $(EX2_BIN) $(EX3_BIN)
