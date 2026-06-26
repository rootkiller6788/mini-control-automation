# Makefile for mini-motor-control
#
# Builds the motor control library, tests, and examples.
# make         ? build library + tests + examples
# make test    ? build and run all tests
# make clean   ? remove build artifacts

CC       = gcc
CFLAGS   = -Wall -Wextra -O2 -std=c11 -Iinclude
LDFLAGS  = -lm
AR       = ar
ARFLAGS  = rcs

SRC_DIR  = src
INC_DIR  = include
TST_DIR  = tests
EXA_DIR  = examples
BLD_DIR  = build

LIB_SRCS = src/transforms.c src/motor_model.c src/motor_control.c src/pwm_modulation.c src/sensorless.c src/svpwm.c src/motor_types.c
LIB_OBJS = build/transforms.o build/motor_model.o build/motor_control.o build/pwm_modulation.o build/sensorless.o build/svpwm.o build/motor_types.o
LIB_NAME = build/libmotorcontrol.a

TEST_BINS = build/test_transforms build/test_motor_model build/test_pid build/test_svpwm build/test_sensorless
EXAMPLE_BINS = build/example_dc_motor_speed_control build/example_bldc_six_step build/example_pmsm_foc

.PHONY: all clean test lib examples

all: lib examples

lib: $(LIB_NAME)

build:
	mkdir -p build

build/transforms.o: src/transforms.c | build
	$(CC) $(CFLAGS) -c src/transforms.c -o build/transforms.o

build/motor_model.o: src/motor_model.c | build
	$(CC) $(CFLAGS) -c src/motor_model.c -o build/motor_model.o

build/motor_control.o: src/motor_control.c | build
	$(CC) $(CFLAGS) -c src/motor_control.c -o build/motor_control.o

build/pwm_modulation.o: src/pwm_modulation.c | build
	$(CC) $(CFLAGS) -c src/pwm_modulation.c -o build/pwm_modulation.o

build/sensorless.o: src/sensorless.c | build
	$(CC) $(CFLAGS) -c src/sensorless.c -o build/sensorless.o

build/svpwm.o: src/svpwm.c | build
	$(CC) $(CFLAGS) -c src/svpwm.c -o build/svpwm.o

build/motor_types.o: src/motor_types.c | build
	$(CC) $(CFLAGS) -c src/motor_types.c -o build/motor_types.o

$(LIB_NAME): $(LIB_OBJS)
	$(AR) $(ARFLAGS) build/libmotorcontrol.a $(LIB_OBJS)

build/test_transforms: tests/test_transforms.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) tests/test_transforms.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/test_transforms

build/test_motor_model: tests/test_motor_model.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) tests/test_motor_model.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/test_motor_model

build/test_pid: tests/test_pid.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) tests/test_pid.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/test_pid

build/test_svpwm: tests/test_svpwm.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) tests/test_svpwm.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/test_svpwm

build/test_sensorless: tests/test_sensorless.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) tests/test_sensorless.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/test_sensorless

build/example_dc_motor_speed_control: examples/dc_motor_speed_control.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) examples/dc_motor_speed_control.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/example_dc_motor_speed_control

build/example_bldc_six_step: examples/bldc_six_step.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) examples/bldc_six_step.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/example_bldc_six_step

build/example_pmsm_foc: examples/pmsm_foc.c $(LIB_NAME) | build
	$(CC) $(CFLAGS) examples/pmsm_foc.c -Lbuild -lmotorcontrol $(LDFLAGS) -o build/example_pmsm_foc

tests: $(TEST_BINS)
	@echo "Running tests..."
	@./build/test_transforms && echo "PASS: test_transforms" || echo "FAIL: test_transforms"
	@./build/test_motor_model && echo "PASS: test_motor_model" || echo "FAIL: test_motor_model"
	@./build/test_pid && echo "PASS: test_pid" || echo "FAIL: test_pid"
	@./build/test_svpwm && echo "PASS: test_svpwm" || echo "FAIL: test_svpwm"
	@./build/test_sensorless && echo "PASS: test_sensorless" || echo "FAIL: test_sensorless"

examples: $(EXAMPLE_BINS)

test: tests

clean:
	rm -rf build

help:
	@echo "Targets:"
	@echo "  make          Build library and examples"
	@echo "  make test     Build and run all tests"
	@echo "  make clean    Remove build artifacts"
