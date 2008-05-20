libera-kinematics.so: kinematics.c trajectory.c
	$(CC) -shared -o libera-kinematics.so kinematics.c trajectory.c -lm

install: libera-kinematics.so
	install -c -o root -g root -m 644 libera-kinematics.so /usr/lib

clean:
	rm -f *.o *.a *.so
