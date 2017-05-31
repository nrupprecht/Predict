CC = icpc
OPT =
FLAGS = -std=c++14 -g -O3 $(OPT)
targets = driver
files = Prediction.o Field.o

all: $(targets)

# Executables
driver: driver.o $(files)
	$(CC) $(OPT) $^ -o $@

# Object files
%.o : %.cpp
	$(CC) -c $(FLAGS) $^

.PHONY: clean
clean:
	rm *.o $(targets)
