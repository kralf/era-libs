CFLAGS += -I../posix/config -I../posix/timer -I../posix/thread \
	-I../posix/spline -I../drivers/vanilla/can -I../drivers/vanilla/epos -I../
LDFLAGS += -L../base -lera-base -L../control -lera-control

PREFIX = /usr/local

OBJS = era_init era_tool_state era_joint_state era_velocity_state era_state \
	era_home era_move_joints era_move_tool era_move_home era_sensors \
	era_trajectory_eval era_open_loop era_closed_loop

all: $(OBJS)

install: $(OBJS)
	install -c -m 755 $(OBJS) $(PREFIX)/bin

clean:
	rm -f *.o $(OBJS)
