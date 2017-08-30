# Compilers for gcc and intel
GCC = g++
ICC = icpc
# C++ standards for gcc and intel
GCSTD = -std=c++11
ICSTD = -std=c++14
# Optimization for gcc and intel
ICCOPTFLAGS = -xHost -O3 -no-prec-div -static -restrict -funroll-loops -no-inline-max-size
GCCOPTFLAGS = -O3 -static -funroll-loops -ffast-math -ffinite-math-only -freciprocal-math -ftree-vectorize -flto

# Compilation for gcc or intel
ICOMP = $(ICC) $(ICSTD) -g $(ICCOPTFLAGS)
GCOMP = $(GCC) $(GCSTD) -g $(GCCOPTFLAGS)

# -- CHOOSE COMPILER: choose whether to use gcc or intel --
# -- For intel,
#	CC   = $(ICC)
#   	COMP = $(ICOMP)
# -- For gcc,
#	CC   = $(GCC)
#	COMP = $(GCOMP)
CC   = $(ICC)
COMP = $(ICOMP)

# All object files needed
SRC_OBJECTS = build/Field.o build/System.o build/Field.o build/VField.o build/DataRecord.o build/FieldGenerator.o
INC_OBJECTS = build/vec2d.o

ALL_OBJECTS = $(SRC_OBJECTS) $(INC_OBJECTS)

all: bin/driver bin/DataVsNP bin/DataVsTau bin/DataVsConsumption

###### Executables #####
bin/driver: build/driver.o $(ALL_OBJECTS)
	$(CC) $^ -o $@

bin/DataVsNP: build/DataVsNP.o $(ALL_OBJECTS)
	$(CC) $^ -o $@

bin/DataVsTau: build/DataVsTau.o $(ALL_OBJECTS)
	$(CC) $^ -o $@

bin/DataVsConsumption: build/DataVsConsumption.o $(ALL_OBJECTS)
	$(CC) $^ -o $@

#####   Objects   #####
build/%.o : %.cpp
	$(COMP) -c -o $@ $<

build/%.o: applications/%.cpp
	$(COMP) -c -o $@ $<

build/%.o: src/%.cpp
	$(COMP) -c -o $@ $<

build/%.o: include/%.cpp
	$(COMP) -c -o $@ $<

#####   Clean    #####
.PHONY : clean
clean:
	rm -f build/*.o build/*.optrpt *.o *.optrpt bin/*
