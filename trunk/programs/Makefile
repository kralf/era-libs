
all: main test nullposition testmode inverse inverse_vel test_error test_function_time

main: main.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o main main.c libserial.o libepos.o

nullposition: nullposition.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o nullposition nullposition.c libserial.o libepos.o

testmode: testmode.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o testmode testmode.c libserial.o libepos.o
			
libserial.o: libserial.c libserial.h
	gcc -Wall -c libserial.c -o libserial.o
	
libepos.o: libepos.c
	gcc -Wall -c libepos.c -o libepos.o
		
test: test.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o test test.c libserial.o libepos.o

test_error: test_error.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o test_error test_error.c libserial.o libepos.o

test_function_time: test_function_time.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o test_function_time test_function_time.c libserial.o libepos.o
		
foo: foo.c
	gcc -Wall -o foo foo.c

datafile: datafile.c
	gcc -Wall -o datafile datafile.c

clean_datafile:
	rm datafile
	
kin.o: kin.c kin.h
	gcc -Wall -c kin.c -o kin.o

kin2serial.o: kin2serial.c kin2serial.h kin.c kin.h libserial.c libserial.h
	gcc -Wall -c kin2serial.c -o kin2serial.o

trajectory.o: trajectory.c trajectory.h kin.c kin.h 
	gcc -Wall -c trajectory.c -o trajectory.o

inverse: inverse.c kin.o kin2serial.o kin.c kin.h libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -lm -o inverse inverse.c kin.o kin2serial.o libserial.o libepos.o

inverse_vel: inverse_vel.c kin.o kin2serial.o kin.c kin.h libserial.o libserial.h libepos.h libepos.o libepos_type.h trajectory.h trajectory.c trajectory.o
	gcc -Wall -lm -o inverse_vel inverse_vel.c kin.o kin2serial.o libserial.o libepos.o trajectory.o



clean:
	rm  *.o
	rm test
	rm test_function_time
	rm test_error
	rm testmode
	rm main 
	rm nullposition
	rm inverse
	rm inverse_vel
	
	
	