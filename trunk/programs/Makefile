
all: main test nullposition testmode inverse

main: main.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o main main.c libserial.o libepos.o

nullposition: nullposition.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o nullposition nullposition.c libserial.o libepos.o

testmode: testmode.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o testmode testmode.c libserial.o libepos.o
			
libserial.o: libserial.c 
	gcc -Wall -c libserial.c -o libserial.o
	
libepos.o: libepos.c
	gcc -Wall -c libepos.c -o libepos.o
		
test: test.c libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -o test test.c libserial.o libepos.o
	
kin.o: kin.c kin.h
	gcc -Wall -c kin.c -o kin.o

inverse: inverse.c kin.o kin.c kin.h libserial.o libserial.h libepos.h libepos.o libepos_type.h
	gcc -Wall -lm -o inverse inverse.c kin.o libserial.o libepos.o



clean:
	rm main 
	rm nullposition
	rm testmode
	rm test
	rm inverse
	rm  *.o