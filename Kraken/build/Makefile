LIBS = -lwiringPi -lsweep -lpthread -lSDL2 -O2 -lrt -lstdc++ -funsigned-char -I/home/pi/Desktop/ELME2002/Kraken/IO/RPLIDAR/include -I/home/pi/Desktop/ELME2002/Kraken/IO/RPLIDAR/src
CC = g++
OPTS = -Wall
# Directories
OBJDIR = ../obj
SRCDIR = ..


# Files and folders
SRCS    = $(filter-out *main.cc,$(shell find $(SRCDIR) -name '*.cc'))

SRCDIRS = $(shell find . -name '*.cc' | dirname {} | sort | uniq | sed 's/\/$(SRCDIR)//g' )
OBJS    = $(patsubst $(SRCDIR)/%.cc,$(OBJDIR)/%.o,$(SRCS))

.PHONY: clean start

# Targets
main: $(OBJS)
	$(CC) $(OBJS) ../IO/RPLIDAR/librplidar_sdk.a $(LIBS) -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cc
	$(CC) $(LIBS) $(OPTS) -c $< -o $@

clean:
	rm $(OBJS) -rf

start: start
	$(shell find $(SRCDIR) -exec touch {} +)

