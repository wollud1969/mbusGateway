CC=gcc

CFLAGS=-Wall
LDFLAGS=-lwiringPi

mbusgw: mbusgw.o
	$(CC) -o $@ $(LDFLAGS) $^

mbusgw.o:	mbusgw.c mbusgw.h

%.o : %.c
	$(CC) $(CFLAGS) -c $<

.PHONY: all
all:    mbusgw

.PHONY: clean
clean:
	-rm -f *.o mbusgw
