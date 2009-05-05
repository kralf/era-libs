CFLAGS += -fPIC
LDFLAGS += -lm

PREFIX = /usr/local
OBJECTS = geometry.o tool.o joint.o velocity.o

%.o: %.c %.h
	$(CC) -c $(CFLAGS) $< -o $@

libera-base.so: $(OBJECTS)
	$(CC) $(LDFLAGS) -shared -o libera-base.so $(OBJECTS)

install: libera-base.so
	install -c -m 755 libera-base.so $(PREFIX)/lib

clean:
	rm -f *.o
	rm -f *.a
	rm -f *.so
