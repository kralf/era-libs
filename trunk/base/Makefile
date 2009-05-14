CFLAGS += -I../drivers/vanilla/epos -I../drivers/vanilla/can \
	-I../posix/config -I../ -fPIC
LDFLAGS += -L../config -lera-config -L../motors -lera-motors \
 	-L../security -lera-security -L../kinematics -lera-kinematics \
	-L../dynamics -lera-dynamics -lm

PREFIX = /usr/local
OBJECTS = geometry.o tool.o joint.o velocity.o era.o

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
