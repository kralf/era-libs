CFLAGS += -I../posix/config -I../posix/timer -I../drivers/vanilla/can \
	-I../drivers/vanilla/epos -I../
LDFLAGS += -L../base -lera-base -L../posix/timer -ltimer

PREFIX = /usr/local

OBJS = era_init era_tool_state era_joint_state era_velocity_state era_state \
	era_home

all: $(OBJS)

install: $(OBJS)
	install -c -m 755 $(OBJS) $(PREFIX)/bin

clean:
	rm -f *.o $(OBJS)
