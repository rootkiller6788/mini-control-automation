# mini-dcs-scada Makefile
# DCS/SCADA — Distributed Control System & Supervisory Control And Data Acquisition
# Implements industrial process control, signal processing, communication, and alarm management

CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -pedantic -O2 -g
INC_DIR  = include
SRC_DIR  = src
TEST_DIR = tests
EX_DIR   = examples
DEMO_DIR = demos
BENCH_DIR = benches
BUILD_DIR = build

INC = -I$(INC_DIR)

CORE_SRC    = $(SRC_DIR)/dcs_core.c $(SRC_DIR)/pid_controller.c $(SRC_DIR)/signal_chain.c
COMM_SRC    = $(SRC_DIR)/comm_protocol.c
ALARM_SRC   = $(SRC_DIR)/alarm_manager.c
LOGGER_SRC  = $(SRC_DIR)/data_logger.c
ALL_SRC     = $(CORE_SRC) $(COMM_SRC) $(ALARM_SRC) $(LOGGER_SRC)
ALL_OBJ     = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(ALL_SRC))

TEST_SRC  = $(TEST_DIR)/test_pid.c $(TEST_DIR)/test_signal.c $(TEST_DIR)/test_alarm.c $(TEST_DIR)/test_core.c
TEST_BIN  = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/%, $(TEST_SRC))

EX_SRC  = $(EX_DIR)/example_water_treatment.c $(EX_DIR)/example_chemical_reactor.c $(EX_DIR)/example_power_grid.c
EX_BIN  = $(patsubst $(EX_DIR)/%.c, $(BUILD_DIR)/%, $(EX_SRC))

DEMO_SRC  = $(DEMO_DIR)/demo_control_loop.c
DEMO_BIN  = $(BUILD_DIR)/demo_control_loop

BENCH_SRC = $(BENCH_DIR)/bench_pid.c
BENCH_BIN = $(BUILD_DIR)/bench_pid

.PHONY: all clean test examples demos bench lines check

all: $(ALL_OBJ)
	@echo "[DCS-SCADA] Library objects built: $(words $(ALL_OBJ)) files"

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

test: $(ALL_OBJ) $(TEST_BIN)
	@echo "=== Running all tests ==="
	@passed=0; failed=0; \
	for t in $(TEST_BIN); do \
		if [ -f $$t ]; then \
			echo "--- $$t ---"; \
			if ./$$t; then passed=$$((passed+1)); \
			else failed=$$((failed+1)); fi; \
		fi; \
	done; \
	echo "=== Results: $$passed passed, $$failed failed ==="; \
	if [ $$failed -gt 0 ]; then exit 1; fi

$(BUILD_DIR)/test_%: $(TEST_DIR)/test_%.c $(ALL_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -o $@ $< $(ALL_OBJ) -lm

examples: $(ALL_OBJ) $(EX_BIN)
	@echo "=== Running all examples ==="
	@for e in $(EX_BIN); do \
		if [ -f $$e ]; then \
			echo "--- $$e ---"; ./$$e; \
		fi; \
	done

$(BUILD_DIR)/example_%: $(EX_DIR)/example_%.c $(ALL_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -o $@ $< $(ALL_OBJ) -lm

demos: $(ALL_OBJ) $(DEMO_BIN)
	@echo "--- Demo ---"
	./$(DEMO_BIN)

$(BUILD_DIR)/demo_%: $(DEMO_DIR)/demo_%.c $(ALL_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -o $@ $< $(ALL_OBJ) -lm

benches: $(ALL_OBJ) $(BENCH_BIN)
	@echo "--- Benchmarks ---"
	./$(BENCH_BIN)

$(BUILD_DIR)/bench_%: $(BENCH_DIR)/bench_%.c $(ALL_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -o $@ $< $(ALL_OBJ) -lm

lines:
	@echo "include/ + src/ line count:"
	@find include src -name "*.h" -o -name "*.c" | xargs wc -l | tail -1
	@echo "include/*.h count: $$(ls include/*.h 2>/dev/null | wc -l)"
	@echo "src/*.c count: $$(ls src/*.c 2>/dev/null | wc -l)"

check:
	@echo "=== Filler detection scan ==="
	@echo -n "_fn[0-9]: "; grep -rn "_fn[0-9]" include/ src/ --include="*.c" --include="*.h" 2>/dev/null | wc -l
	@echo -n "_aux[0-9]: "; grep -rn "_aux[0-9]" include/ src/ --include="*.c" --include="*.h" 2>/dev/null | wc -l
	@echo -n "TODO/FIXME/stub: "; grep -rn "TODO\|FIXME\|stub\|placeholder" include/ src/ tests/ 2>/dev/null | wc -l
	@echo -n "sorry (lean): "; grep -rn "\bsorry\b" src/ --include="*.lean" 2>/dev/null | wc -l

clean:
	rm -rf $(BUILD_DIR)
