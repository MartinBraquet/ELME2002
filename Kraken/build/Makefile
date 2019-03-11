LIBS = -lwiringPi -lsweep -lpthread
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
	echo $(SRCS)
	$(CC) $(OBJS) $(LIBS) -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cc
	$(CC) $(OPTS) -c $< -o $@

clean:
	rm $(OBJS) -rf

start: start
	$(shell find $(SRCDIR) -exec touch {} +)

