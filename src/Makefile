CC=gcc

CFLAGS=-Wall
LDFLAGS=-lwiringPi

mbusgw: mbusgw.o
	$(CC) -o $@ $(LDFLAGS) $^

.c.o:
	$(CC) $(CFLAGS) -c $<

.PHONY: all
all:    mbusgw

.PHONY: clean
clean:
	-rm -f *.o mbusgw
