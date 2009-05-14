CFLAGS += -I../drivers/vanilla/epos -I../drivers/vanilla/can \
	-I../posix/config -I../ -fPIC
LDFLAGS += -L../drivers/vanilla/epos -lepos -L../config -lera-config \
	-L../security -lera-security

PREFIX = /usr/local
OBJECTS = motors.o

%.o: %.c %.h
	$(CC) -c $(CFLAGS) $< -o $@

libera-motors.so: $(OBJECTS)
	$(CC) $(LDFLAGS) -shared -o libera-motors.so $(OBJECTS)

install: libera-motors.so
	install -c -m 755 libera-motors.so $(PREFIX)/lib

clean:
	rm -f *.o
	rm -f *.a
	rm -f *.so
