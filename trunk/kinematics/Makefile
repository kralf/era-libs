CFLAGS += -I../errors
LDFLAGS += -L../errors

PREFIX = /usr/local

libera-kinematics.so: kinematics.c trajectory.c velocity_profile.c io.c
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -o libera-kinematics.so \
	kinematics.c velocity.c trajectory.c velocity_profile.c io.c \
	-lera-errors -lm

install: libera-kinematics.so
	install -c -m 644 libera-kinematics.so $(PREFIX)/lib

clean:
	rm -f *.o *.a *.so
