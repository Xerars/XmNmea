# [Makefile]

# Author   : William Hsiao


# [Option]:
CC = gcc
AR = ar -rcs
RM = rm -f
SHARED = -shared -fpic

CFLAGS = -std=c99 -Wall -Wextra -O3
MAIN = -DXMAIN
DEBUG = -DDEBUG

RMEXE = *.exe *.stackdump *.o
RMCOV = *.gcov *.gcda *.gcno
RMLIB = *.dll *.so *.a

SRC = XmNmea.c
OBJ1 = XmNmea_main.o
OBJ2 = XmNmea.o
EXE = Sample

# [Content]
.PHONY:clean

# Sample Code Tag(make Sample)
$(EXE):$(OBJ1)
	$(CC) $(CFLAGS) $(DEBUG) $(MAIN) $(SRC) -o $(EXE)
	$(RM) *.o

$(OBJ1):$(SRC)
	$(CC) $(CFLAGS) $(DEBUG) $(MAIN) $(SRC) -c -o $(OBJ1)

$(OBJ2):$(SRC)
	$(CC) $(CFLAGS) $(SRC) -c -o $(OBJ2)

# Remove File
clean:
	$(RM) $(RMEXE) $(RMCOV) $(RMLIB)