test.out : test.o rmp.o rmp_leafs.o
CC = g++
INCLUDE = ./Include
SRC = ./src
CPPFLAGS = --std=c++11 -I$(INCLUDE)

VPATH = ./src : ./Include
DEP = test.o rmp.o rmp_leafs.o


test.out : ${DEP}
	${CC} -o test.out ${CPPFLAGS} ${DEP}

${filter %.o ,${DEP}}: %.o :%.cpp
	${CC} -c ${CPPFLAGS} $< -o $@



clean:
	rm -rf test.o rmp.o rmp_leafs.o test.out
