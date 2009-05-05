CFLAGS += -I../ -fPIC
LDFLAGS += -L../base -lera-base

PREFIX = /usr/local
OBJECTS = limits.o dynamics.o

%.o: %.c %.h
	$(CC) -c $(CFLAGS) $< -o $@

libera-dynamics.so: $(OBJECTS)
	$(CC) $(LDFLAGS) -shared -o libera-dynamics.so $(OBJECTS)

install: libera-dynamics.so
	install -c -m 755 libera-dynamics.so $(PREFIX)/lib

clean:
	rm -f *.o
	rm -f *.a
	rm -f *.so
