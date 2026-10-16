# Тесты ядра и симулятор на ПК (нужен g++ / clang++; на Windows — WSL или MSYS2).
#   make -C firmware/test          — собрать и запустить тесты
#   make -C firmware/test sim      — собрать симулятор firmware/sim/cyclone_sim
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter
CORE = ../lib/core/protocol.cpp ../lib/core/robot.cpp

.PHONY: all test sim clean syntax
all: test

test_core: test_core.cpp $(CORE) ../lib/core/*.h ../sim/plant.h
	$(CXX) $(CXXFLAGS) -o $@ test_core.cpp $(CORE)

test: test_core
	./test_core

sim: ../sim/cyclone_sim
../sim/cyclone_sim: ../sim/sim_main.cpp $(CORE) ../lib/core/*.h ../sim/plant.h
	$(CXX) $(CXXFLAGS) -o $@ ../sim/sim_main.cpp $(CORE)

clean:
	rm -f test_core ../sim/cyclone_sim

# Проверка, что src/main.cpp синтаксически корректен (на заглушках Arduino, без ESP32-тулчейна)
syntax:
	$(CXX) -std=gnu++17 -fsyntax-only -Wall -Imock -I../lib/core ../src/main.cpp
