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

.PHONY: install
install: all
	sudo cp mbusgw /usr/local/bin/

.PHONY: clean
clean:
	-rm -f *.o mbusgw
