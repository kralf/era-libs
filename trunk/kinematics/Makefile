CFLAGS += -I../errors
LDFLAGS += -L../errors

libera-kinematics.so: kinematics.c trajectory.c
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -o libera-kinematics.so \
	kinematics.c velocity.c trajectory.c -lera-errors -lm

install: libera-kinematics.so
	install -c -o root -g root -m 644 libera-kinematics.so /usr/lib

clean:
	rm -f *.o *.a *.so
