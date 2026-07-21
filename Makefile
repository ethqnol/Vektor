# Vektor Library Makefile

CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Iinclude

TEST_SRCS = tests/test_main.cpp \
            tests/test_units.cpp \
            tests/test_pose.cpp \
            tests/test_pid.cpp \
            tests/test_feedforward.cpp \
            tests/test_actions.cpp \
            tests/test_mcl.cpp \
            src/Vektor/PID.cpp \
            src/Vektor/MCL.cpp

TEST_BIN = run_tests

.PHONY: all test clean

all: test

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(TEST_SRCS)
	$(CXX) $(CXXFLAGS) $(TEST_SRCS) -o $(TEST_BIN)

clean:
	rm -f $(TEST_BIN)
