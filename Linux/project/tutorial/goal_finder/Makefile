###############################################################
#
# Purpose: Makefile for "goal_finder"
# Author.: robotis && Andreas Ekadinata Widodo
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = goal_finder

INCLUDE_DIRS = -I../../../include -I../../../../Framework/include `pkg-config --cflags opencv`

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg `pkg-config --libs opencv`

OBJECTS =   main2.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/color_filtering_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
