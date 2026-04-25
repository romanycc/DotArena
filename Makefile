PYTHON = python3
PYBIND_INCLUDE = $(shell $(PYTHON) -m pybind11 --includes)
PYTHON_INCLUDE = $(shell $(PYTHON)-config --includes)
PYTHON_SUFFIX  = $(shell $(PYTHON)-config --extension-suffix)
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -fPIC $(PYBIND_INCLUDE) $(PYTHON_INCLUDE)
LDFLAGS = -shared
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S), Darwin)
    LDFLAGS += -undefined dynamic_lookup
endif

TARGET = _dotarena$(PYTHON_SUFFIX)
SRC = $(wildcard *.cpp)
OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TARGET)
	$(PYTHON) -m pytest -v

clean:
	rm -f $(TARGET) $(OBJ)
	rm -f *.so