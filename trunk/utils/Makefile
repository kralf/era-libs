LD_LIBRARY_PATH = ../errors:../kinematics:../drivers/vanilla/epos:../drivers/vanilla/can
CFLAGS += -I../kinematics -I../controller -I../drivers/vanilla/epos
LDFLAGS += -L../controller -lera-controller

PREFIX = /usr/local

OBJS = era_init era_home era_limits era_move_tool era_move_arm era_print \
	era_kinematics era_velocity era_trajectory era_simulate
SRCS = $(OBJ)

all: $(OBJS)

install: $(OBJS)
	install -c -m 755 $(OBJS) $(PREFIX)/bin

clean:
	rm -f *.o $(OBJS)
