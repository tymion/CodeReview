SRC=$(shell find src -name '*.c')
SRC_TEST=$(shell find test -name '*.cpp')
SRC_TEST+=src/lib/math_pack.c
INCLUDES=-Iheader
LIBS=-lm

.PHONY: test

compile:
	gcc -Wall $(SRC) $(INCLUDES) $(LIBS) -o build/adcsApp

test:
	g++ -Wall $(SRC_TEST) $(INCLUDES) $(LIBS) -lgtest -lpthread -o build/adcsTest
