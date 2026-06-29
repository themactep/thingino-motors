# Simple Makefile for ingenic-motor
# - Builds motors and motors-daemon
# - Uses the local json_config compatibility layer under src/
#
# Usage examples:
#   make
#   make clean
#   make CFLAGS='-Wall -Wextra -O2 -g'

CC       ?= cc
CFLAGS   ?= -Wall -Wextra -O2
CPPFLAGS ?= -Isrc
LDFLAGS  ?=

SRC_DIR  := src
BINARIES := motors motors-daemon
OBJS     := $(SRC_DIR)/motor.o $(SRC_DIR)/motor-daemon.o

.PHONY: all clean distclean format

all: $(BINARIES)

motors: $(SRC_DIR)/motor.o
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDFLAGS)

motors-daemon: $(SRC_DIR)/motor-daemon.o
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LDFLAGS)

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

format:
	@if command -v clang-format >/dev/null 2>&1; then \
		clang-format -i $(SRC_DIR)/motor.c $(SRC_DIR)/motor-daemon.c; \
	else \
		echo "clang-format not found; skipping format"; \
	fi

clean:
	rm -f $(OBJS) $(BINARIES)
	rm -f *.o *.a *.so $(SRC_DIR)/*.o

distclean: clean
	rm -rf third_party
