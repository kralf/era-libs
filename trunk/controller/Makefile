CAN_COMM = serial

CFLAGS += -I../kinematics -I../drivers/vanilla/epos
LDFLAGS += -L../kinematics -L../drivers/vanilla/epos -L../drivers/vanilla/can-$(CAN_COMM)

libera-controller.so: motors.c
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -o libera-controller.so motors.c
	-lepos-kinematics -lepos -lcan-$(CAN_COMM)

install: libera-controller.so
	install -c -o root -g root -m 644 libera-controller.so /usr/lib

clean:
	rm -f *.o *.a *.so
