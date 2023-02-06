#source file search paths
VPATH = ./src ../autocell/src ../raven-set \
	../pathfinderfeb2023/src

	# compiler include search paths
INCS = -I./include \
	-I../windex/include \
	-I../raven-set \
	-I../autocell/src \
	-I../pathfinderfeb2023/src

LIBS = \
	-lgdiplus -lgdi32 -lcomdlg32 -lstdc++fs -lComctl32 \
	#-mwindows

static_link_flags = -static-libstdc++ -static-libgcc -static

# folders
ODIR = ./.vscode/obj

OPT = -g
CCflags = -std=c++17 $(OPT) -D_mingw_

# sources
OBJ_starter = \
	$(ODIR)/main.o $(ODIR)/cObstacle.o \
	$(ODIR)/autocell.o \
	$(ODIR)/cGUI.o $(ODIR)/cxy.o \
	$(ODIR)/cGraphData.o

OBJ_test = \
	$(ODIR)/test.o \
	$(ODIR)/cObstacle.o $(ODIR)/autocell.o  $(ODIR)/cxy.o \
	$(ODIR)/cutest.o

### COMPILE

$(ODIR)/cGraphData.o : ../pathfinderfeb2023/src/cGraphData.cpp
	g++  $(CCflags) \
	-c -o $@ $< $(INCS)

$(ODIR)/%.o: %.cpp  cObstacle.h cGUI.h
	g++  $(CCflags) \
	-c -o $@ $< $(INCS)

starter: $(OBJ_starter)
	g++ $(OPT) \
	-o bin/obstacle.exe $^  \
	$(static_link_flags) $(LIBS)

test: $(OBJ_test)
	g++ -o bin/test.exe \
	$(OPT) $^  \
	$(static_link_flags) $(LIBS)


.PHONY: clean
clean:
	del /q .vscode\\obj\\*.o

rebuild: clean starter test

