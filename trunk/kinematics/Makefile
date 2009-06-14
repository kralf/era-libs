CFLAGS += -I../ -fPIC
LDFLAGS +=

PREFIX = /usr/local
OBJECTS = kinematics.o limits.o

%.o: %.c %.h
	$(CC) -c $(CFLAGS) $< -o $@

libera-kinematics.so: $(OBJECTS)
	$(CC) $(LDFLAGS) -shared -o libera-kinematics.so $(OBJECTS)

install: libera-kinematics.so
	install -c -m 755 libera-kinematics.so $(PREFIX)/lib

clean:
	rm -f *.o
	rm -f *.a
	rm -f *.so
