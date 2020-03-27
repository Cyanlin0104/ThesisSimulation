all: test.out
CC = g++
INCLUDE = -I./Include -I./Robot
CPPFLAGS = --std=c++11 
VPATH = ./Include
#DEP = test.o rmp.o rmp_leafs.o rmp_robot.o rmp_robot_v4_warpper.o
SRC = ${wildcard ./src/*.cpp ./Robot/*.cpp}

test.out : ${SRC}
	${CC} -o $@ $^ ${CPPFLAGS}

#${filter %.o ,${DEP}}: %.o :%.cpp
#	${CC} -c ${CPPFLAGS} $< -o $@



clean:
	rm -rf test.o rmp.o rmp_leafs.o rmp_robot.o rmp_robot_v4.o test.out 
