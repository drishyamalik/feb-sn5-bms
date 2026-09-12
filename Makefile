CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -O2

CORE = bms_fsm.c can_bus.c

all: bms_demo bms_test

bms_demo: main.c $(CORE) bms_fsm.h can_bus.h bms_types.h
	$(CC) $(CFLAGS) -o $@ main.c $(CORE)

bms_test: test_bms_fsm.c $(CORE) bms_fsm.h can_bus.h bms_types.h
	$(CC) $(CFLAGS) -o $@ test_bms_fsm.c $(CORE)

run: bms_demo
	./bms_demo

test: bms_test
	./bms_test

clean:
	rm -f bms_demo bms_test *.o

.PHONY: all run test clean
