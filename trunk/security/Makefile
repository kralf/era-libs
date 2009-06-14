CFLAGS += -I../drivers/vanilla/epos -I../drivers/vanilla/can \
	-I../posix/config -I../ -fPIC
LDFLAGS +=

PREFIX = /usr/local
OBJECTS = security.o

%.o: %.c %.h
	$(CC) -c $(CFLAGS) $< -o $@

libera-security.so: $(OBJECTS)
	$(CC) $(LDFLAGS) -shared -o libera-security.so $(OBJECTS)

install: libera-security.so
	install -c -m 755 libera-security.so $(PREFIX)/lib

clean:
	rm -f *.o
	rm -f *.a
	rm -f *.so
