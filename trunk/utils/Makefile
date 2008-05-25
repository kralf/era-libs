LD_LIBRARY_PATH = ../kinematics:../drivers/vanilla/epos:../drivers/vanilla/can
CFLAGS += -I../kinematics -I../controller
LDFLAGS += -L../controller -lera-controller

OBJS = era_init era_home era_limits era_move_tool era_move_arm era_print \
	era_trajectory
SRCS = $(OBJ)

all: $(OBJS)

install: $(OBJS)
	install -c -o root -g root -m 755 $(OBJS) /usr/bin

clean:
	rm -f *.o $(OBJS)
