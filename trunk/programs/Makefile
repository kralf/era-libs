
all: main test nullposition testmode inverse kin_main test_error test_function_time

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
	
libkin.o: libkin.c libkin.h
	gcc -Wall -c libkin.c -o libkin.o

libkin2epos.o: libkin2epos.c libkin2epos.h libkin.c libkin.h libserial.c libserial.h
	gcc -Wall -c libkin2epos.c -o libkin2epos.o

libtrajectory.o: libtrajectory.c libtrajectory.h libkin.c libkin.h 
	gcc -Wall -c libtrajectory.c -o libtrajectory.o

inverse: inverse.c libkin.o libkin2epos.o libkin.c libkin.h libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -lm -o inverse inverse.c libkin.o libkin2epos.o libserial.o libepos.o

kin_main: kin_main.c libkin.o libkin2epos.o libkin.c libkin.h libserial.o libserial.h libepos.h libepos.o libepos_type.h libtrajectory.h libtrajectory.c libtrajectory.o
	gcc -Wall -lm -o kin_main kin_main.c libkin.o libkin2epos.o libserial.o libepos.o libtrajectory.o



clean:
	rm  *.o
	rm test
	rm test_function_time
	rm test_error
	rm testmode
	rm main 
	rm nullposition
	rm inverse
	rm kin_main
	
	
	