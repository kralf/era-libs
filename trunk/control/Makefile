CFLAGS += -I../drivers/vanilla/epos -I../drivers/vanilla/can \
	-I../posix/config -I../posix/thread -I../posix/timer -I../posix/spline \
	-I../ -fPIC
LDFLAGS += -L../posix/thread -lthread -L../base -lera-base

PREFIX = /usr/local
OBJECTS = sensors.o open_loop.o closed_loop.o

%.o: %.c %.h
	$(CC) -c $(CFLAGS) $< -o $@

libera-control.so: $(OBJECTS)
	$(CC) $(LDFLAGS) -shared -o libera-control.so $(OBJECTS)

install: libera-control.so
	install -c -m 755 libera-control.so $(PREFIX)/lib

clean:
	rm -f *.o
	rm -f *.a
	rm -f *.so
