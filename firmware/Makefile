# Native build of the hardware-independent flight core + tests.
#   make -C firmware test
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
CORE := $(wildcard src/core/*.cpp)

build/test_native: $(CORE) test/native/test_main.cpp $(wildcard src/core/*.h) test/native/golden.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Isrc/core -Itest/native $(CORE) test/native/test_main.cpp -o $@

test: build/test_native
	./build/test_native

clean:
	rm -rf build

.PHONY: test clean

# Type-check the Teensy firmware (HAL + main) on the host against stub
# Arduino headers. Catches compile errors without the Teensy toolchain; it
# is NOT a substitute for `pio run -e teensy41`.
compile-check:
	@for f in src/main.cpp src/hal/*.cpp src/core/*.cpp; do \
		$(CXX) -std=c++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
			-Itest/arduino_stub -Isrc $$f || exit 1; \
	done; echo "compile-check OK"

.PHONY: compile-check
