
all: main test nullposition testmode

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
	
era_kin.o: era_kin.c era_kin.h
	gcc -Wall -c era_kin.c -o era_kin.o

clean:
	rm main 
	rm nullposition
	rm testmode
	rm test
	rm  *.o