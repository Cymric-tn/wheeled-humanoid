# Host targets. The Arduino build itself uses PlatformIO:  pio run  (and  pio run -t upload)
CORE := lib/robot_core/src
SRC := $(CORE)/drive.cpp $(CORE)/gestures.cpp $(CORE)/robot_core.cpp
CXX ?= c++
CXXFLAGS := -std=c++11 -O2 -Wall -Wextra -Wno-unused-parameter -I$(CORE)
UNAME := $(shell uname)
LIBEXT := $(if $(filter Darwin,$(UNAME)),dylib,so)

.PHONY: test lib clean

test: build/test_core
	./build/test_core

build/test_core: test/test_core.cpp $(SRC) $(wildcard $(CORE)/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ test/test_core.cpp $(SRC)

# shared library used by the digital twin (sim/firmware_sim.py)
lib: build/librobot_core.$(LIBEXT)

build/librobot_core.$(LIBEXT): $(SRC) $(CORE)/sim_api.cpp $(wildcard $(CORE)/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -fPIC -shared -o $@ $(SRC) $(CORE)/sim_api.cpp

clean:
	rm -rf build
