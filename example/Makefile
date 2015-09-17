CC = g++
CC_FLAGS = -g -std=c++11 -I/opt/eigen/3.2.5/ -I../KalmanFilter/

all: test

.PHONY: test
test: main.cxx Predictor.cxx
	$(CC) $(CC_FLAGS) $^ -o $@

.PHONY: clean
clean:
	@rm -rf test
