OBJs := $(patsubst %.cpp, %.o, $(wildcard *.cpp))
BIN := ledu-imu

CC := g++

CPPFLAGS += -I.

# Use the Eigen library.
EIGEN_NAME = eigen3
CPPFLAGS += $(shell pkg-config --cflags $(EIGEN_NAME))

# All warnings
CPPFLAGS += -Wall

# Use a modern language
CPPFLAGS += -std=gnu++11

# Use boost libraries
LDLIBS += -lboost_program_options

# Use wiringpi
LDLIBS += -lwiringPi 
LDLIBS += -lwiringPiDev

# Put debugging info in there so we can get stack traces.
#CPPFLAGS += -g -rdynamic

# Optimize the code.
CPPFLAGS += -O2

# Generate .d files with dependency info
CPPFLAGS += -MD -MP

all: vector.h.gch $(BIN)

$(BIN) : $(OBJs)

DEPs := $(OBJs:%.o=%.d)

vector.h.gch: vector.h
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@

.PHONY: clean
clean:
	@rm -fv $(BIN) $(OBJs) $(DEPs) *.o *.gch *.d
	@rm -fr docs

.PHONY: docs
docs:
	doxygen

prefix = $(DESTDIR)/usr/local
bindir = $(prefix)/bin
sharedir = $(prefix)/share
mandir = $(sharedir)/man
man1dir = $(mandir)/man1


.PHONY: install
install: $(BIN)
	install $(BIN) $(bindir)

-include $(DEPs) vector.h.d
