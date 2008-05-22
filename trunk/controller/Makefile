CFLAGS += -I../kinematics -I../drivers/vanilla/epos
LDFLAGS += -L../kinematics -L../drivers/vanilla/epos

libera-controller.so: errors.c motors.c controller.c
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -o libera-controller.so errors.c \
	motors.c controller.c -lera-kinematics -lepos

install: libera-controller.so
	install -c -o root -g root -m 644 libera-controller.so /usr/lib

clean:
	rm -f *.o *.a *.so
