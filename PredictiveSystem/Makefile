# Compilers for gcc and intel
GCC = g++
ICC = icpc
# C++ standards for gcc and intel
GCSTD = -std=c++11
ICSTD = -std=c++14
# Optimization for gcc and intel
ICCOPTFLAGS = $(ICSTD) -xHost -O3 -no-prec-div -static -fp-model fast=2 -restrict -funroll-loops -no-inline-max-size
GCCOPTFLAGS = $(GCSTD) -O3 -static -funroll-loops -ffast-math -ffinite-math-only -freciprocal-math -ftree-vectorize -flto
# -vectorize-slp -mllvm -bb-vectorize-aligned-only -fslp-vectorize-aggressive -fstrict-vtable-pointers -fwhole-program-vtables

# Compilation for gcc or intel
GCOMP = $(GCC) $(GCCOPTFLAGS)
ICOMP = $(ICC) $(ICCOPTFLAGS)

# -- Choose whether to use gcc or intel --
# -- For intel,
#	CC   = $(ICC)
#   	COMP = $(ICOMP)
# -- For gcc,
#	CC   = $(GCC)
#	COMP = $(GCOMP)
CC   = $(ICC)
COMP = $(ICOMP)

# All object files needed
SRC_OBJECTS   = build/Field.o build/System.o build/Field.o build/VField.o build/vec2d.o

ALL_OBJECTS     = $(SRC_OBJECTS)

all: bin/driver 

###### Executables #####
bin/driver: build/driver.o $(ALL_OBJECTS)
	$(CC) $^ -o $@

#####   Objects   #####
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
